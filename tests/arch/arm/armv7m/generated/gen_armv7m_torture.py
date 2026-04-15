#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
ARMv7-M torture test generator.

Generates two categories of C files:
  A) Compiler stress tests: complex expressions whose expected output is
     computed here on the host.  Tests exercise compiler code generation
     paths (multiply, shift, rotate, XOR, add/sub chains).

  B) Instruction-level inline-asm tests: systematic coverage of each ALU
     instruction with multiple operand classes.  Expected result and flags
     are computed on the host Python model, embedded as assertions.

Usage:
    python3 gen_armv7m_torture.py [--output-dir <dir>]

The generated files are committed to the repo so Twister finds them
without a build-time generation step.

Reference: DDI0403E (ARMv7-M ARM)
  ADD/SUB/ADC/SBC  A7.7.1-6, A7.7.119-120, A7.7.176  lines 5465-5916
  AND/ORR/EOR/BIC  A7.7.8-9, A7.7.35-36, A7.7.91-92, A7.7.15-16
  MOV/MOVT         A7.7.76-79  lines 10358-10630
  CLZ/RBIT         A7.7.24, A7.7.112
  MUL/SMULL/UMULL  A7.7.84, A7.7.149, A7.7.204
  SDIV/UDIV        A7.7.127, A7.7.195
  Flags            A2.3.2  lines 769-794
"""

import argparse
import os
import sys
import ctypes
import textwrap
from pathlib import Path

# ---------------------------------------------------------------------------
# ARMv7-M host model helpers
# ---------------------------------------------------------------------------

U32_MAX = 0xFFFF_FFFF
U64_MAX = 0xFFFF_FFFF_FFFF_FFFF

def u32(v):
    return int(v) & U32_MAX

def s32(v):
    v = u32(v)
    return v if v < 0x8000_0000 else v - 0x1_0000_0000

def add_with_carry(a, b, c_in):
    """ARM AddWithCarry: returns (result, C, V) as u32."""
    a = u32(a)
    b = u32(b)
    c_in = int(c_in) & 1
    r64 = a + b + c_in
    result = u32(r64)
    c_out = 1 if r64 > U32_MAX else 0
    sa, sb, sr = (a >> 31) & 1, (b >> 31) & 1, (result >> 31) & 1
    v_out = 1 if (sa == sb) and (sa != sr) else 0
    return result, c_out, v_out

def flags_nzcv(result, c, v):
    n = (result >> 31) & 1
    z = 1 if result == 0 else 0
    return (n << 3) | (z << 2) | (c << 1) | v

def lsl_c(x, n):
    x = u32(x)
    if n == 0:
        return x, 0
    if n >= 32:
        c = (x >> (32 - n)) & 1 if n == 32 else 0
        return 0, c
    c = (x >> (32 - n)) & 1
    return u32(x << n), c

def lsr_c(x, n):
    x = u32(x)
    if n == 0:
        return x, 0
    if n >= 32:
        c = (x >> (n - 1)) & 1 if n == 32 else 0
        return 0, c
    c = (x >> (n - 1)) & 1
    return x >> n, c

def asr_c(x, n):
    x = u32(x)
    if n == 0:
        return x, 0
    sx = s32(x)
    if n >= 32:
        c = (x >> 31) & 1
        return u32(-1 if sx < 0 else 0), c
    c = (x >> (n - 1)) & 1
    return u32(sx >> n), c

def ror_c(x, n):
    x = u32(x)
    n = n % 32
    if n == 0:
        return x, (x >> 31) & 1
    r = ((x >> n) | (x << (32 - n))) & U32_MAX
    c = (x >> (n - 1)) & 1
    return r, c

def rotate_right_32(x, n):
    n &= 7
    return ((x >> n) | (x << (32 - n))) & U32_MAX

# ---------------------------------------------------------------------------
# Operand classes
# ---------------------------------------------------------------------------

ALU_OPERANDS = [
    (0, 0),
    (0, 1),
    (1, 0),
    (1, 1),
    (0xFFFF_FFFF, 1),
    (0x7FFF_FFFF, 1),
    (0x8000_0000, 0x8000_0000),
    (0xDEAD_BEEF, 0xCAFE_BABE),
    (0xAAAA_AAAA, 0x5555_5555),
    (0x1234_5678, 0x9ABC_DEF0),
    (0xFFFF_FFFF, 0xFFFF_FFFF),
    (0x0000_0001, 0xFFFF_FFFE),
    (0x8000_0001, 0x7FFF_FFFF),
    (100, 7),
    (0x1000, 0x1000),
]

SHIFT_AMOUNTS = [0, 1, 4, 7, 15, 16, 24, 31]

# ---------------------------------------------------------------------------
# Category A: Compiler stress tests
# ---------------------------------------------------------------------------

def rotate_left_py(x, n):
    n &= 31
    return u32((x << n) | (x >> (32 - n)))

def gen_stress_case(idx, a, b, c):
    """Generate one compiler stress case and compute expected value."""
    x = u32((u32(a) * 33) ^ (u32(b) + 0x1234_5678))
    shift = c & 7
    x = rotate_left_py(x, shift)
    x = u32(x + u32(u32(a ^ c) - u32(b | 0x55AA_55AA)))
    x = u32(x ^ (u32(a * b) >> 8))
    x = u32(x + (c & 0xFFFF))
    expected = x

    code = f"""\
ZTEST(armv7m_generated, test_compiler_stress_{idx:04d})
{{
\tuint32_t a = 0x{a:08X}U, b = 0x{b:08X}U, c = 0x{c:08X}U;
\tuint32_t x;

\tx = (a * 33U) ^ (b + 0x12345678U);
\tuint32_t shift = c & 7U;
\tx = (x << shift) | (x >> (32U - shift));
\tx += ((a ^ c) - (b | 0x55AA55AAU));
\tx ^= ((a * b) >> 8U);
\tx += (c & 0xFFFFU);
\tzassert_equal(x, 0x{expected:08X}U,
\t\t"stress_{idx:04d}: got 0x%08x exp 0x{expected:08X}", x);
}}
"""
    return code

# ---------------------------------------------------------------------------
# Category B: Instruction-level inline-asm tests
# ---------------------------------------------------------------------------

def gen_add_case(idx, a, b):
    result, c, v = add_with_carry(a, b, 0)
    nzcv = flags_nzcv(result, c, v)
    return f"""\
ZTEST(armv7m_generated, test_adds_{idx:04d})
{{
\tuint32_t r, apsr;
\t__asm__ volatile(
\t\t"adds %[r], %[a], %[b]\\n\\t"
\t\t"mrs  %[f], apsr\\n\\t"
\t\t: [r] "=&r"(r), [f] "=r"(apsr)
\t\t: [a] "r"(0x{a:08X}U), [b] "r"(0x{b:08X}U)
\t\t: "cc");
\tzassert_equal(r, 0x{result:08X}U, "ADDS_{idx} result: got 0x%08x", r);
\tzassert_equal((apsr >> 28) & 0xF, 0x{nzcv:X}U,
\t\t"ADDS_{idx} flags: got 0x%x exp 0x{nzcv:X}", (apsr >> 28) & 0xF);
}}
"""

def gen_subs_case(idx, a, b):
    result, c, v = add_with_carry(a, ~u32(b), 1)
    nzcv = flags_nzcv(result, c, v)
    return f"""\
ZTEST(armv7m_generated, test_subs_{idx:04d})
{{
\tuint32_t r, apsr;
\t__asm__ volatile(
\t\t"subs %[r], %[a], %[b]\\n\\t"
\t\t"mrs  %[f], apsr\\n\\t"
\t\t: [r] "=&r"(r), [f] "=r"(apsr)
\t\t: [a] "r"(0x{a:08X}U), [b] "r"(0x{b:08X}U)
\t\t: "cc");
\tzassert_equal(r, 0x{result:08X}U, "SUBS_{idx} result: got 0x%08x", r);
\tzassert_equal((apsr >> 28) & 0xF, 0x{nzcv:X}U,
\t\t"SUBS_{idx} flags: got 0x%x exp 0x{nzcv:X}", (apsr >> 28) & 0xF);
}}
"""

def gen_ands_case(idx, a, b):
    result = u32(a & b)
    n = (result >> 31) & 1
    z = 1 if result == 0 else 0
    nz = (n << 1) | z
    return f"""\
ZTEST(armv7m_generated, test_ands_{idx:04d})
{{
\tuint32_t r, apsr;
\t__asm__ volatile(
\t\t"ands %[r], %[a], %[b]\\n\\t"
\t\t"mrs  %[f], apsr\\n\\t"
\t\t: [r] "=&r"(r), [f] "=r"(apsr)
\t\t: [a] "r"(0x{a:08X}U), [b] "r"(0x{b:08X}U)
\t\t: "cc");
\tzassert_equal(r, 0x{result:08X}U, "ANDS_{idx} result: got 0x%08x", r);
\tzassert_equal((apsr >> 30) & 3U, 0x{nz:X}U,
\t\t"ANDS_{idx} NZ: got 0x%x exp 0x{nz:X}", (apsr >> 30) & 3U);
}}
"""

def gen_orrs_case(idx, a, b):
    result = u32(a | b)
    n = (result >> 31) & 1
    z = 1 if result == 0 else 0
    nz = (n << 1) | z
    return f"""\
ZTEST(armv7m_generated, test_orrs_{idx:04d})
{{
\tuint32_t r, apsr;
\t__asm__ volatile(
\t\t"orrs %[r], %[a], %[b]\\n\\t"
\t\t"mrs  %[f], apsr\\n\\t"
\t\t: [r] "=&r"(r), [f] "=r"(apsr)
\t\t: [a] "r"(0x{a:08X}U), [b] "r"(0x{b:08X}U)
\t\t: "cc");
\tzassert_equal(r, 0x{result:08X}U, "ORRS_{idx} result: got 0x%08x", r);
\tzassert_equal((apsr >> 30) & 3U, 0x{nz:X}U,
\t\t"ORRS_{idx} NZ flags", (apsr >> 30) & 3U);
}}
"""

def gen_eors_case(idx, a, b):
    result = u32(a ^ b)
    n = (result >> 31) & 1
    z = 1 if result == 0 else 0
    nz = (n << 1) | z
    return f"""\
ZTEST(armv7m_generated, test_eors_{idx:04d})
{{
\tuint32_t r, apsr;
\t__asm__ volatile(
\t\t"eors %[r], %[a], %[b]\\n\\t"
\t\t"mrs  %[f], apsr\\n\\t"
\t\t: [r] "=&r"(r), [f] "=r"(apsr)
\t\t: [a] "r"(0x{a:08X}U), [b] "r"(0x{b:08X}U)
\t\t: "cc");
\tzassert_equal(r, 0x{result:08X}U, "EORS_{idx} result: got 0x%08x", r);
}}
"""

def gen_lsls_case(idx, a, sh):
    result, c = lsl_c(a, sh)
    n = (result >> 31) & 1
    z = 1 if result == 0 else 0
    nzcv = (n << 3) | (z << 2) | (c << 1)
    return f"""\
ZTEST(armv7m_generated, test_lsls_{idx:04d})
{{
\tuint32_t r, apsr;
\t__asm__ volatile(
\t\t"lsls %[r], %[a], %[s]\\n\\t"
\t\t"mrs  %[f], apsr\\n\\t"
\t\t: [r] "=&r"(r), [f] "=r"(apsr)
\t\t: [a] "r"(0x{a:08X}U), [s] "r"({sh}U)
\t\t: "cc");
\tzassert_equal(r, 0x{result:08X}U, "LSLS_{idx} result: got 0x%08x", r);
\tzassert_equal((apsr >> 28) & 0xE, 0x{nzcv & 0xE:X}U,
\t\t"LSLS_{idx} NZC: got 0x%x exp 0x{nzcv & 0xE:X}", (apsr >> 28) & 0xE);
}}
"""

def gen_lsrs_case(idx, a, sh):
    result, c = lsr_c(a, sh)
    n = (result >> 31) & 1
    z = 1 if result == 0 else 0
    nzcv = (n << 3) | (z << 2) | (c << 1)
    return f"""\
ZTEST(armv7m_generated, test_lsrs_{idx:04d})
{{
\tuint32_t r, apsr;
\t__asm__ volatile(
\t\t"lsrs %[r], %[a], %[s]\\n\\t"
\t\t"mrs  %[f], apsr\\n\\t"
\t\t: [r] "=&r"(r), [f] "=r"(apsr)
\t\t: [a] "r"(0x{a:08X}U), [s] "r"({sh}U)
\t\t: "cc");
\tzassert_equal(r, 0x{result:08X}U, "LSRS_{idx} result: got 0x%08x", r);
}}
"""

def gen_mul_case(idx, a, b):
    result = u32(a * b)
    return f"""\
ZTEST(armv7m_generated, test_mul_{idx:04d})
{{
\tuint32_t r;
\t__asm__ volatile("mul %[r], %[a], %[b]\\n\\t"
\t\t: [r] "=r"(r) : [a] "r"(0x{a:08X}U), [b] "r"(0x{b:08X}U) : );
\tzassert_equal(r, 0x{result:08X}U, "MUL_{idx} result: got 0x%08x", r);
}}
"""

def gen_udiv_case(idx, a, b):
    if b == 0:
        return ""
    result = u32(a) // u32(b)
    return f"""\
ZTEST(armv7m_generated, test_udiv_{idx:04d})
{{
\tuint32_t r;
\t__asm__ volatile("udiv %[r], %[a], %[b]\\n\\t"
\t\t: [r] "=r"(r) : [a] "r"(0x{a:08X}U), [b] "r"(0x{b:08X}U) : );
\tzassert_equal(r, 0x{result:08X}U, "UDIV_{idx}: got 0x%08x", r);
}}
"""

def gen_sdiv_case(idx, a, b):
    if b == 0:
        return ""
    sa = s32(a)
    sb = s32(b)
    # Truncate toward zero
    import math
    result = math.trunc(sa / sb)
    result = u32(result)
    return f"""\
ZTEST(armv7m_generated, test_sdiv_{idx:04d})
{{
\tuint32_t r;
\t__asm__ volatile("sdiv %[r], %[a], %[b]\\n\\t"
\t\t: [r] "=r"(r) : [a] "r"(0x{a:08X}U), [b] "r"(0x{b:08X}U) : );
\tzassert_equal(r, 0x{result:08X}U, "SDIV_{idx}: got 0x%08x", r);
}}
"""

def gen_clz_case(idx, a):
    a = u32(a)
    if a == 0:
        clz = 32
    else:
        clz = 0
        x = a
        while not (x & 0x8000_0000):
            clz += 1
            x <<= 1
    return f"""\
ZTEST(armv7m_generated, test_clz_{idx:04d})
{{
\tuint32_t r;
\t__asm__ volatile("clz %[r], %[v]\\n\\t"
\t\t: [r] "=r"(r) : [v] "r"(0x{a:08X}U) : );
\tzassert_equal(r, {clz}U, "CLZ_{idx}: got %d exp {clz}", r);
}}
"""

# ---------------------------------------------------------------------------
# Generate all files
# ---------------------------------------------------------------------------

STRESS_PARAMS = [
    (0x10203040, 0xDEADBEEF, 5),
    (0x00000000, 0x00000000, 0),
    (0xFFFFFFFF, 0xFFFFFFFF, 31),
    (0x12345678, 0x9ABCDEF0, 7),
    (0x80000000, 0x00000001, 1),
    (0x7FFFFFFF, 0x80000000, 15),
    (0xAAAAAAAA, 0x55555555, 3),
    (0x01020304, 0x05060708, 2),
    (0xCAFEBABE, 0xDEADC0DE, 11),
    (0x13579BDF, 0x2468ACE0, 17),
    (0x11111111, 0x22222222, 4),
    (0x33333333, 0x44444444, 6),
    (0x55555555, 0x66666666, 8),
    (0x77777777, 0x88888888, 9),
    (0x99999999, 0xAAAAAAAA, 10),
    (0xBBBBBBBB, 0xCCCCCCCC, 12),
    (0xDDDDDDDD, 0xEEEEEEEE, 13),
    (0xFEDCBA98, 0x76543210, 14),
    (0x0F0F0F0F, 0xF0F0F0F0, 16),
    (0x00FF00FF, 0xFF00FF00, 20),
]

DIV_PAIRS = [
    (100, 7),
    (0xFFFFFFFF, 1),
    (0xFFFFFFFF, 0xFFFFFFFF),
    (1000, 3),
    (0x80000000, 2),
    (0x7FFFFFFF, 0x7FFFFFFF),
    (50, 50),
    (0x10000, 0x100),
    (1, 1),
    (7, 3),
]

CLZ_VALUES = [0, 1, 0x80000000, 0xFFFFFFFF, 0x00000100, 0x00010000,
              0x0000FFFF, 0x7FFFFFFF, 0x40000000, 2]

HEADER = """\
/*
 * SPDX-License-Identifier: Apache-2.0
 * AUTO-GENERATED by gen_armv7m_torture.py -- DO NOT EDIT MANUALLY.
 * Run gen_armv7m_torture.py to regenerate.
 */

#include <zephyr/ztest.h>
#include <stdint.h>

"""

def write_file(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    print(f"  wrote {path}")

def generate(output_dir):
    out = Path(output_dir)

    # ---- Category A: compiler stress ----
    stress_body = HEADER
    for i, (a, b, c) in enumerate(STRESS_PARAMS):
        stress_body += gen_stress_case(i, a, b, c)
    write_file(out / "gen_compiler_stress.c", stress_body)

    # ---- Category B: instruction tests, split into multiple files ----
    add_body = HEADER
    for i, (a, b) in enumerate(ALU_OPERANDS):
        add_body += gen_add_case(i, a, b)
    write_file(out / "gen_adds.c", add_body)

    sub_body = HEADER
    for i, (a, b) in enumerate(ALU_OPERANDS):
        sub_body += gen_subs_case(i, a, b)
    write_file(out / "gen_subs.c", sub_body)

    and_body = HEADER
    for i, (a, b) in enumerate(ALU_OPERANDS):
        and_body += gen_ands_case(i, a, b)
    write_file(out / "gen_ands.c", and_body)

    orr_body = HEADER
    for i, (a, b) in enumerate(ALU_OPERANDS):
        orr_body += gen_orrs_case(i, a, b)
    write_file(out / "gen_orrs.c", orr_body)

    eor_body = HEADER
    for i, (a, b) in enumerate(ALU_OPERANDS):
        eor_body += gen_eors_case(i, a, b)
    write_file(out / "gen_eors.c", eor_body)

    lsl_body = HEADER
    idx = 0
    for a in [0, 1, 0x80000000, 0xFFFFFFFF, 0x12345678, 0xAAAAAAAA]:
        for sh in SHIFT_AMOUNTS:
            if sh > 0:  # skip shift-by-0 for inline-asm (C flag undefined)
                lsl_body += gen_lsls_case(idx, a, sh)
                idx += 1
    write_file(out / "gen_lsls.c", lsl_body)

    lsr_body = HEADER
    idx = 0
    for a in [0, 1, 0x80000000, 0xFFFFFFFF, 0x12345678]:
        for sh in [1, 4, 8, 16, 31]:
            lsr_body += gen_lsrs_case(idx, a, sh)
            idx += 1
    write_file(out / "gen_lsrs.c", lsr_body)

    mul_body = HEADER
    for i, (a, b) in enumerate(ALU_OPERANDS):
        mul_body += gen_mul_case(i, a, b)
    write_file(out / "gen_mul.c", mul_body)

    udiv_body = HEADER
    idx = 0
    for a, b in DIV_PAIRS:
        c = gen_udiv_case(idx, a, b)
        if c:
            udiv_body += c
            idx += 1
    write_file(out / "gen_udiv.c", udiv_body)

    sdiv_body = HEADER
    idx = 0
    for a, b in DIV_PAIRS:
        c = gen_sdiv_case(idx, a, b)
        if c:
            sdiv_body += c
            idx += 1
    write_file(out / "gen_sdiv.c", sdiv_body)

    clz_body = HEADER
    for i, a in enumerate(CLZ_VALUES):
        clz_body += gen_clz_case(i, a)
    write_file(out / "gen_clz.c", clz_body)

    # ---- main.c: suite declaration ----
    main_body = """\
/*
 * SPDX-License-Identifier: Apache-2.0
 * AUTO-GENERATED by gen_armv7m_torture.py -- DO NOT EDIT MANUALLY.
 */

#include <zephyr/ztest.h>

ZTEST_SUITE(armv7m_generated, NULL, NULL, NULL, NULL, NULL);
"""
    write_file(out / "main.c", main_body)

    print(f"Generated {len(list(out.glob('*.c')))} files in {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARMv7-M torture test generator")
    parser.add_argument("--output-dir", default=str(Path(__file__).parent / "src"),
                        help="Directory to write generated .c files")
    args = parser.parse_args()
    generate(args.output_dir)
