# Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0

'''West runner for Rockchip devices using the rockutil flashing tool.

Supports the Rockchip MaskROM → Loader → flash workflow for SPI NAND targets
(e.g. Luckfox Pico Plus / RV1106).  The device must be held in MaskROM mode
(BOOT button + power cycle) before running ``west flash``.

Flash sequence
--------------
1. rockutil LD — confirm a MaskROM or Loader device is present.
2. If MaskROM: ``rockutil UL <idblock>`` — upload DDR-init + miniloader,
   wait for re-enumeration as Loader (PID 0x110D for RV1106).
3. ``rockutil WL 0x200 <idblock>`` — write idblock to NAND LBA 0x200.
4. ``rockutil WL 0x400 <zephyr.itb>`` — write FIT image to NAND LBA 0x400.
5. ``rockutil RD`` — reboot (unless --no-reboot is passed).
'''

import os
import subprocess
import sys
import tempfile
from pathlib import Path

from runners.core import RunnerCaps, ZephyrBinaryRunner

# Default NAND LBA offsets matching the Luckfox Pico Plus partition table.
_IDBLOCK_LBA_DEFAULT = '0x200'
_ITB_LBA_DEFAULT = '0x400'


class RockutilBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for the rockutil Rockchip flashing tool.'''

    def __init__(self, cfg, *, rockutil='rockutil', idblock, itb_file=None,
                 idblock_lba=_IDBLOCK_LBA_DEFAULT,
                 itb_lba=_ITB_LBA_DEFAULT,
                 no_reboot=False):
        super().__init__(cfg)
        self.rockutil = rockutil
        self.idblock = idblock
        self.itb_file = itb_file
        self.idblock_lba = idblock_lba
        self.itb_lba = itb_lba
        self.no_reboot = no_reboot

    @classmethod
    def name(cls):
        return 'rockutil'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'})

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument(
            '--rockutil', default='rockutil',
            help='rockutil executable; default "rockutil"')
        parser.add_argument(
            '--idblock', required=True,
            help='Path to the Rockchip idblock binary (RKNS magic, '
                 'contains DDR-init + miniloader).')
        parser.add_argument(
            '--zephyr-itb',
            help='Path to the FIT image (zephyr.itb). '
                 'Defaults to <build>/zephyr/zephyr.itb.')
        parser.add_argument(
            '--idblock-lba', default=_IDBLOCK_LBA_DEFAULT,
            help=f'NAND LBA address for idblock; default {_IDBLOCK_LBA_DEFAULT}')
        parser.add_argument(
            '--itb-lba', default=_ITB_LBA_DEFAULT,
            help=f'NAND LBA address for zephyr.itb; default {_ITB_LBA_DEFAULT}')
        parser.add_argument(
            '--no-reboot', default=False, action='store_true',
            help='Do not reboot the device after flashing.')

    @classmethod
    def do_create(cls, cfg, args):
        itb = args.zephyr_itb
        if itb is None:
            itb = os.path.join(cfg.build_dir, 'zephyr', 'zephyr.itb')

        return cls(
            cfg,
            rockutil=args.rockutil,
            idblock=args.idblock,
            itb_file=itb,
            idblock_lba=args.idblock_lba,
            itb_lba=args.itb_lba,
            no_reboot=args.no_reboot,
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_itb_from_bin(self, bin_path, itb_path):
        '''Fall-back: generate zephyr.itb from zephyr.bin using mkimage.'''
        mkimage = self.require('mkimage')
        its_path = Path(itb_path).with_suffix('.its')
        load_addr = '0x00200000'

        its_content = f'''/dts-v1/;
/ {{
    description = "Zephyr FIT for Rockchip miniloader";
    #address-cells = <1>;
    images {{
        zephyr {{
            description = "Zephyr Cortex-A7";
            data = /incbin/("{bin_path}");
            type = "standalone";
            arch = "arm";
            os = "u-boot";
            compression = "none";
            load = <{load_addr}>;
            entry = <{load_addr}>;
            hash-1 {{ algo = "sha256"; }};
        }};
    }};
    configurations {{
        default = "conf";
        conf {{
            description = "Zephyr only";
            loadables = "zephyr";
        }};
    }};
}};
'''
        its_path.write_text(its_content)
        self.check_call([mkimage, '-f', str(its_path), '-E', str(itb_path)])

    def _rockutil_ld(self):
        '''Return the rockutil LD output as a string.'''
        try:
            out = self.check_output([self.rockutil, 'LD'])
            return out.decode(sys.getdefaultencoding(), errors='replace')
        except subprocess.CalledProcessError:
            return ''

    def _is_maskrom(self, ld_output):
        return 'MaskRom' in ld_output or 'Maskrom' in ld_output or \
               'maskrom' in ld_output or 'MASKROM' in ld_output or \
               '350A' in ld_output

    def _is_loader(self, ld_output):
        return 'Loader' in ld_output or 'loader' in ld_output or \
               'LOADER' in ld_output or \
               '110D' in ld_output or '110B' in ld_output or \
               '350B' in ld_output

    # ------------------------------------------------------------------
    # Flash
    # ------------------------------------------------------------------

    def do_run(self, command, **kwargs):
        self.require(self.rockutil)

        idblock = str(Path(self.idblock).resolve())
        itb = str(Path(self.itb_file).resolve())

        if not Path(idblock).is_file():
            raise RuntimeError(
                f'idblock not found: {idblock}\n'
                'Pass --idblock=<path> or set it in board.cmake.')

        # Build the ITB if it does not already exist.
        if not Path(itb).is_file():
            bin_path = os.path.join(self.cfg.build_dir, 'zephyr', 'zephyr.bin')
            if not Path(bin_path).is_file():
                raise RuntimeError(
                    f'zephyr.itb not found ({itb}) and zephyr.bin also '
                    f'missing ({bin_path}).  Build the project first.')
            print(f'zephyr.itb not found; building from {bin_path}')
            self._build_itb_from_bin(bin_path, itb)

        # Detect device mode.
        print('Detecting Rockchip device...')
        ld_out = self._rockutil_ld()
        print(ld_out.strip())

        if not ld_out or (not self._is_maskrom(ld_out) and
                          not self._is_loader(ld_out)):
            raise RuntimeError(
                'No Rockchip device found.\n'
                'Put the board in MaskROM mode: hold BOOT while '
                'connecting USB, then run west flash again.')

        if self._is_maskrom(ld_out):
            print('MaskROM detected — uploading idblock loader...')
            self.check_call([self.rockutil, 'UL', idblock])
            # After UL, re-detect to confirm Loader mode.
            ld_out = self._rockutil_ld()
            print(ld_out.strip())
            if not self._is_loader(ld_out):
                raise RuntimeError(
                    'Device did not switch to Loader mode after UL command.\n'
                    'Check that the idblock blob is valid for this SoC.')

        print(f'Writing idblock to NAND LBA {self.idblock_lba}...')
        self.check_call([self.rockutil, 'WL', self.idblock_lba, idblock])

        print(f'Writing zephyr.itb to NAND LBA {self.itb_lba}...')
        self.check_call([self.rockutil, 'WL', self.itb_lba, itb])

        if not self.no_reboot:
            print('Rebooting device...')
            self.check_call([self.rockutil, 'RD'])
        else:
            print('Flashing complete.  Reboot the device manually.')
