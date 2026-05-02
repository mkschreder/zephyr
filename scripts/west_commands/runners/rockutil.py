# Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0

'''West runner for Rockchip devices using the rockutil flashing tool.

Supports the Rockchip MaskROM → Loader → flash workflow for SPI NAND targets
(e.g. Luckfox Pico Plus / RV1106).  The device must be held in MaskROM mode
(BOOT button + power cycle) before running ``west flash``.

Two distinct binary blobs are involved — they serve different purposes:

  ``loader``   (RKBOOT/LDR magic) — sent over USB via ``rockutil UL`` to
               perform the MaskROM handshake.  It bundles DDR-init (e471
               entries) and a USB plug binary (e472 entries) in Rockchip's
               RKBOOT container format.  For RV1106 this is ``download.bin``
               (or ``MiniLoaderAll.bin``) from the official Luckfox SDK.
               This is NOT written to NAND.

  ``idblock``  (RKNS magic) — the actual NAND boot image written to NAND
               LBA 0x200 by ``rockutil WL``.  The BootROM searches for this
               ``RKNS`` magic on the NAND after a normal power-on reset.

Flash sequence
--------------
1. ``rockutil LD`` — detect MaskROM or Loader device.
2. If MaskROM: ``rockutil UL <loader>`` — send DDR-init + usbplug via USB,
   wait for device to re-enumerate as Loader mode.
3. ``rockutil WL <idblock-lba> <idblock>`` — write RKNS idblock to NAND.
4. ``rockutil WL <itb-lba>   <zephyr.itb>`` — write FIT image to NAND.
5. ``rockutil RD`` — reboot (unless --no-reboot).
'''

import os
import subprocess
import sys
from pathlib import Path

from runners.core import RunnerCaps, ZephyrBinaryRunner

# Default NAND LBA offsets matching the Luckfox Pico Plus partition table.
_IDBLOCK_LBA_DEFAULT = '0x200'
_ITB_LBA_DEFAULT = '0x400'


class RockutilBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for the rockutil Rockchip flashing tool.'''

    def __init__(self, cfg, *, rockutil='rockutil',
                 loader, idblock, itb_file=None,
                 idblock_lba=_IDBLOCK_LBA_DEFAULT,
                 itb_lba=_ITB_LBA_DEFAULT,
                 no_reboot=False):
        super().__init__(cfg)
        self.rockutil = rockutil
        self.loader = loader
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
            '--loader', required=True,
            help='Path to the Rockchip USB loader in RKBOOT format '
                 '(BOOT/LDR magic; contains DDR-init e471 entries and '
                 'usbplug e472 entries).  Used only for the MaskROM→Loader '
                 'USB handshake via "rockutil UL".  For RV1106 this is '
                 'download.bin / MiniLoaderAll.bin from the Luckfox SDK.')
        parser.add_argument(
            '--idblock', required=True,
            help='Path to the NAND idblock (RKNS magic). '
                 'Written to NAND at --idblock-lba via "rockutil WL". '
                 'The BootROM loads this on normal power-on from NAND.')
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
            loader=args.loader,
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
        return ('MaskRom' in ld_output or 'Maskrom' in ld_output or
                'maskrom' in ld_output or 'MASKROM' in ld_output or
                '350A' in ld_output)

    def _is_loader(self, ld_output):
        return ('Loader' in ld_output or 'loader' in ld_output or
                'LOADER' in ld_output or
                '110D' in ld_output or '110B' in ld_output or
                '350B' in ld_output)

    # ------------------------------------------------------------------
    # Flash
    # ------------------------------------------------------------------

    def do_run(self, command, **kwargs):
        self.require(self.rockutil)

        loader  = str(Path(self.loader).resolve())
        idblock = str(Path(self.idblock).resolve())
        itb     = str(Path(self.itb_file).resolve())

        if not Path(loader).is_file():
            raise RuntimeError(
                f'Loader (RKBOOT) not found: {loader}\n'
                'Pass --loader=<path> or set it in board.cmake.\n'
                'The loader is the RKBOOT-format file (BOOT/LDR magic) used\n'
                'for the MaskROM USB handshake — e.g. download.bin or\n'
                'MiniLoaderAll.bin from the Luckfox SDK, NOT the NAND idblock.')

        if not Path(idblock).is_file():
            raise RuntimeError(
                f'NAND idblock not found: {idblock}\n'
                'Pass --idblock=<path> or set it in board.cmake.\n'
                'The idblock is the RKNS-format NAND image written to\n'
                f'NAND LBA {self.idblock_lba} via "rockutil WL".')

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
            print('MaskROM detected — uploading DDR-init + usbplug loader...')
            print(f'  loader: {loader}')
            self.check_call([self.rockutil, 'UL', loader])
            # After UL the device re-enumerates in Loader mode.
            ld_out = self._rockutil_ld()
            print(ld_out.strip())
            if not self._is_loader(ld_out):
                raise RuntimeError(
                    'Device did not switch to Loader mode after UL.\n'
                    'Check that --loader points to a valid RKBOOT file\n'
                    '(BOOT/LDR magic) for this SoC.')

        print(f'Writing idblock to NAND LBA {self.idblock_lba}...')
        self.check_call([self.rockutil, 'WL', self.idblock_lba, idblock])

        print(f'Writing zephyr.itb to NAND LBA {self.itb_lba}...')
        self.check_call([self.rockutil, 'WL', self.itb_lba, itb])

        if not self.no_reboot:
            print('Rebooting device...')
            self.check_call([self.rockutil, 'RD'])
        else:
            print('Flashing complete.  Reboot the device manually.')
