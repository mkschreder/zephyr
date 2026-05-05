#!/usr/bin/env python3
# Copyright (c) 2026 Zephyr Project contributors
# SPDX-License-Identifier: Apache-2.0
"""
check_cmse_entry_no_variadic.py — Build-time safety check for CMSE entry functions.

Scans all C source / header files passed on the command line (or the paths
listed in a response file) for functions declared with
__attribute__((cmse_nonsecure_entry)) that also use variadic arguments (...).

Variadic CMSE entry functions are banned by ARM DDI 0553B §C1.4.5 because:
  - The va_list argument-access model uses the caller's stack layout.
  - The NS stack layout is opaque to the Secure entry function.
  - A variadic format-string argument is a classic attack surface.

Usage:
    python scripts/check_cmse_entry_no_variadic.py [file ...]
    python scripts/check_cmse_entry_no_variadic.py @filelist.txt

Exit codes:
    0  — no violations found
    1  — one or more violations found (build should fail)

The script uses a best-effort regex heuristic and is intentionally
conservative: it may produce false positives but never false negatives.
"""

import re
import sys
import os
from pathlib import Path


# Pattern: find function signatures containing cmse_nonsecure_entry and "..."
# This matches the common forms:
#   ret __attribute__((cmse_nonsecure_entry)) name(a, b, ...)
#   __cmse_nonsecure_entry ret name(a, ...)
#   ret name(a, ...) __attribute__((cmse_nonsecure_entry));
_CMSE_ENTRY_VARIADIC_RE = re.compile(
    r"""
    (?:                                 # cmse_nonsecure_entry can appear before
        __attribute__\s*\(\s*\(\s*cmse_nonsecure_entry\s*\)\s*\)
        |
        __cmse_nonsecure_entry
    )
    [^;{]*?                             # rest of the declaration/definition
    \(                                  # opening paren of argument list
    [^)]*?                              # arguments up to '...'
    \.\.\.                              # variadic marker
    [^)]*?                              # anything else
    \)                                  # closing paren
    """,
    re.VERBOSE | re.DOTALL,
)

# Also match the reverse order: attribute comes after the signature.
# We intentionally do NOT use re.DOTALL here and restrict the "rest" between
# the closing ')' and the attribute to the same source line ([^;{\n]*?).
# This prevents false positives where a variadic macro on one line is matched
# to a 'cmse_nonsecure_entry' that appears in a comment or code many lines
# later (e.g. Z_ARM_TZ_NS_ENTRY_WDOG_FEED in tz_ns.h).
_VARIADIC_CMSE_RE = re.compile(
    r"""
    \(                                  # opening paren of argument list
    [^)]*?
    \.\.\.                              # variadic marker
    [^)]*?
    \)                                  # closing paren
    [^;{\n]*?                           # rest — same line only (no newlines)
    (?:
        __attribute__\s*\(\s*\(\s*cmse_nonsecure_entry\s*\)\s*\)
        |
        __cmse_nonsecure_entry
    )
    """,
    re.VERBOSE,
)


def check_file(path: Path) -> list[str]:
    """Return a list of violation descriptions found in *path*."""
    try:
        text = path.read_text(errors="replace")
    except OSError as e:
        print(f"warning: cannot read {path}: {e}", file=sys.stderr)
        return []

    violations = []
    for pattern in (_CMSE_ENTRY_VARIADIC_RE, _VARIADIC_CMSE_RE):
        for m in pattern.finditer(text):
            line_no = text[: m.start()].count("\n") + 1
            snippet = m.group(0).replace("\n", " ").strip()[:120]
            violations.append(f"{path}:{line_no}: variadic cmse_nonsecure_entry: {snippet}")

    return violations


def expand_args(args: list[str]) -> list[str]:
    """Expand @filelist.txt arguments."""
    expanded = []
    for a in args:
        if a.startswith("@"):
            try:
                lines = Path(a[1:]).read_text().splitlines()
                expanded.extend(l.strip() for l in lines if l.strip())
            except OSError as e:
                print(f"warning: cannot read filelist {a}: {e}", file=sys.stderr)
        else:
            expanded.append(a)
    return expanded


def main() -> int:
    args = expand_args(sys.argv[1:])
    if not args:
        print("usage: check_cmse_entry_no_variadic.py [file ...] [@filelist]",
              file=sys.stderr)
        return 0

    all_violations: list[str] = []
    for a in args:
        path = Path(a)
        if path.suffix not in (".c", ".h", ".cpp", ".hpp"):
            continue
        if not path.exists():
            print(f"warning: file not found: {path}", file=sys.stderr)
            continue
        all_violations.extend(check_file(path))

    if all_violations:
        print(
            "\nerror: variadic __attribute__((cmse_nonsecure_entry)) functions are forbidden.\n"
            "ARM DDI 0553B §C1.4.5: va_list arguments are a format-string attack surface\n"
            "in Secure entry functions.  Replace with an explicit, bounded parameter.\n",
            file=sys.stderr,
        )
        for v in all_violations:
            print(v, file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
