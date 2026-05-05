#!/usr/bin/env python3
# Copyright 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0

"""west secure-build: dual-image Secure + Non-Secure Zephyr build helper.

This west extension eliminates the manual two-pass build required when a
Zephyr application is split across a Secure image and a Non-Secure image
without TF-M.  It:

 1. Builds the Secure image (CONFIG_ARM_SECURE_FIRMWARE=y).
 2. Harvests the CMSE import library (libentryveneers.a) produced by the
    Secure linker (--cmse-implib / --out-implib).
 3. Passes the import library directory to the Non-Secure CMake configuration
    so that CONFIG_ARM_FIRMWARE_USES_SECURE_ENTRY_FUNCS links correctly.
 4. Builds the Non-Secure image (CONFIG_ARM_NONSECURE_FIRMWARE=y).
 5. Produces a merged build manifest (secure.hex, ns.hex, manifest.json).

Usage:
    west secure-build \\
        --secure-app  path/to/secure_app \\
        --ns-app      path/to/ns_app \\
        --board       mps2_an521/mps2/cpu0 \\
        [--build-dir  build/trustzone] \\
        [--cmake-args KEY=VALUE ...]

Example:
    west secure-build \\
        --secure-app samples/arch/arm/trustzone_secure \\
        --ns-app     samples/arch/arm/trustzone_ns \\
        --board      mps2_an521/mps2/cpu0

Reference: Zephyr TrustZone-M application note.
           ARM DDI 0553 §C1.4.5 (SG / entry veneers / libentryveneers).
"""

import argparse
import json
import logging
import os
import shutil
import subprocess
import sys
from pathlib import Path
from textwrap import dedent

from west.commands import WestCommand

logger = logging.getLogger(__name__)

SECURE_BUILD_SUBDIR = "secure"
NS_BUILD_SUBDIR = "ns"
VENEER_LIB_NAME = "libentryveneers.a"
MANIFEST_NAME = "manifest.json"


class SecureBuild(WestCommand):
    """west secure-build command implementation."""

    def __init__(self):
        super().__init__(
            "secure-build",
            "Build a paired Secure + Non-Secure Zephyr application",
            dedent("""\
                Builds a Secure Zephyr image and a Non-Secure Zephyr image in
                the correct order, passing the CMSE import library
                (libentryveneers.a) produced by the Secure link step to the
                Non-Secure CMake configuration.
            """),
        )

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description,
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )

        parser.add_argument(
            "--secure-app",
            required=True,
            metavar="PATH",
            help="Path to the Secure application source directory.",
        )
        parser.add_argument(
            "--ns-app",
            required=True,
            metavar="PATH",
            help="Path to the Non-Secure application source directory.",
        )
        parser.add_argument(
            "--board",
            required=True,
            metavar="BOARD[@REV][/QUALIFIERS]",
            help="Target board (same as west build -b).",
        )
        parser.add_argument(
            "--build-dir",
            default="build/trustzone",
            metavar="DIR",
            help="Top-level build directory (default: build/trustzone).",
        )
        parser.add_argument(
            "--cmake-args",
            nargs="*",
            default=[],
            metavar="KEY=VALUE",
            help="Extra CMake definitions passed to both builds.",
        )
        parser.add_argument(
            "--pristine",
            action="store_true",
            help="Clean build directories before building.",
        )
        parser.add_argument(
            "--veneer-lib-name",
            default=VENEER_LIB_NAME,
            metavar="LIBNAME",
            help=f"Name of the CMSE import library (default: {VENEER_LIB_NAME}).",
        )
        return parser

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _run_west_build(app, board, build_dir, extra_cmake_args, pristine=False):
        """Invoke ``west build`` for a single image."""
        cmd = [
            sys.executable, "-m", "west",
            "build",
            "-b", board,
            "-d", str(build_dir),
            str(app),
        ]
        if pristine:
            cmd.append("--pristine")
        if extra_cmake_args:
            cmd += ["--"] + [f"-D{a}" for a in extra_cmake_args]

        logger.info("Running: %s", " ".join(cmd))
        result = subprocess.run(cmd)
        if result.returncode != 0:
            raise RuntimeError(
                f"west build failed for {app} (exit {result.returncode})"
            )

    @staticmethod
    def _find_veneer_lib(build_dir, lib_name):
        """Locate the CMSE import library in the Secure build tree."""
        lib_path = build_dir / "zephyr" / lib_name
        if lib_path.exists():
            return lib_path
        # Fallback: search recursively (handles multi-domain builds)
        matches = list(build_dir.rglob(lib_name))
        if matches:
            return matches[0]
        raise FileNotFoundError(
            f"{lib_name} not found under {build_dir}. "
            "Ensure CONFIG_ARM_FIRMWARE_HAS_SECURE_ENTRY_FUNCS=y in the "
            "Secure image and ARM_TRUSTZONE_M=y."
        )

    @staticmethod
    def _write_manifest(build_dir, secure_hex, ns_hex, veneer_lib):
        manifest = {
            "format": "zephyr-trustzone-manifest-v1",
            "secure": {
                "hex": str(secure_hex),
            },
            "nonsecure": {
                "hex": str(ns_hex),
            },
            "cmse_import_lib": str(veneer_lib),
        }
        manifest_path = build_dir / MANIFEST_NAME
        manifest_path.write_text(json.dumps(manifest, indent=2))
        logger.info("Manifest written to %s", manifest_path)
        return manifest_path

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def do_run(self, args, unknown_args):
        top_build = Path(args.build_dir).resolve()
        secure_build = top_build / SECURE_BUILD_SUBDIR
        ns_build = top_build / NS_BUILD_SUBDIR

        secure_app = Path(args.secure_app).resolve()
        ns_app = Path(args.ns_app).resolve()

        if not secure_app.is_dir():
            self.die(f"Secure app path does not exist: {secure_app}")
        if not ns_app.is_dir():
            self.die(f"NS app path does not exist: {ns_app}")

        if args.pristine:
            for d in (secure_build, ns_build):
                if d.exists():
                    logger.info("Removing %s (pristine)", d)
                    shutil.rmtree(d)

        # ---- Step 1: Build the Secure image ----
        logger.info("=== Building Secure image ===")
        secure_cmake = list(args.cmake_args) + [
            "CONFIG_ARM_SECURE_FIRMWARE=y",
            "CONFIG_ARM_TRUSTZONE_M=y",
            "CONFIG_ARM_FIRMWARE_HAS_SECURE_ENTRY_FUNCS=y",
            f"CONFIG_ARM_ENTRY_VENEERS_LIB_NAME={args.veneer_lib_name}",
        ]
        self._run_west_build(
            secure_app, args.board, secure_build, secure_cmake, pristine=False
        )

        # ---- Step 2: Find the CMSE import library ----
        veneer_lib = self._find_veneer_lib(secure_build, args.veneer_lib_name)
        logger.info("Found CMSE import library: %s", veneer_lib)

        # ---- Step 3: Build the Non-Secure image ----
        logger.info("=== Building Non-Secure image ===")
        ns_cmake = list(args.cmake_args) + [
            "CONFIG_ARM_NONSECURE_FIRMWARE=y",
            "CONFIG_ARM_TRUSTZONE_M=y",
            "CONFIG_ARM_FIRMWARE_USES_SECURE_ENTRY_FUNCS=y",
            f"CONFIG_ARM_ENTRY_VENEERS_LIB_NAME={args.veneer_lib_name}",
            # Point CMake at the directory containing the veneer library so
            # that zephyr_link_libraries() in tz/CMakeLists.txt can find it.
            f"CMAKE_BINARY_DIR={veneer_lib.parent}",
        ]
        self._run_west_build(
            ns_app, args.board, ns_build, ns_cmake, pristine=False
        )

        # ---- Step 4: Locate output HEX files ----
        secure_hex = secure_build / "zephyr" / "zephyr.hex"
        ns_hex = ns_build / "zephyr" / "zephyr.hex"
        for h in (secure_hex, ns_hex):
            if not h.exists():
                logger.warning("HEX file not found: %s", h)

        # ---- Step 5: Write manifest ----
        manifest = self._write_manifest(top_build, secure_hex, ns_hex, veneer_lib)

        self.inf(
            "\n"
            "=== west secure-build complete ===\n"
            f"  Secure HEX : {secure_hex}\n"
            f"  NS HEX     : {ns_hex}\n"
            f"  Veneer lib : {veneer_lib}\n"
            f"  Manifest   : {manifest}\n"
        )
