/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M assembly self-check torture suite.
 *
 * Each ZTEST calls an assembly function that returns 0 on pass,
 * nonzero error code on fail.
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

ZTEST_SUITE(armv7m_asm_selfcheck, NULL, NULL, NULL, NULL, NULL);

/* Declarations of assembly test functions */
extern int asm_adds_zero_carry(void);
extern int asm_adds_signed_overflow(void);
extern int asm_subs_borrow(void);
extern int asm_ands_zero(void);
extern int asm_eors_identity(void);
extern int asm_lsls_carry(void);
extern int asm_asrs_sign(void);
extern int asm_clz_zero(void);
extern int asm_rbit_bit0(void);
extern int asm_mul_basic(void);
extern int asm_rev_word(void);
extern int asm_sbfx_sign(void);
extern int asm_it_eq_taken(void);
extern int asm_it_ne_skip(void);

extern int asm_str_ldr_word(void);
extern int asm_pre_index_wb(void);
extern int asm_post_index_wb(void);
extern int asm_ldrb_zero_ext(void);
extern int asm_ldrsb_sign_ext(void);
extern int asm_stmia_ldmia(void);

/* ALU cases */
ZTEST(armv7m_asm_selfcheck, test_adds_zero_carry)
{
	zassert_equal(asm_adds_zero_carry(), 0, "ADDS zero+carry failed");
}
ZTEST(armv7m_asm_selfcheck, test_adds_signed_overflow)
{
	zassert_equal(asm_adds_signed_overflow(), 0, "ADDS signed overflow failed");
}
ZTEST(armv7m_asm_selfcheck, test_subs_borrow)
{
	zassert_equal(asm_subs_borrow(), 0, "SUBS borrow failed");
}
ZTEST(armv7m_asm_selfcheck, test_ands_zero)
{
	zassert_equal(asm_ands_zero(), 0, "ANDS zero result failed");
}
ZTEST(armv7m_asm_selfcheck, test_eors_identity)
{
	zassert_equal(asm_eors_identity(), 0, "EORS identity failed");
}
ZTEST(armv7m_asm_selfcheck, test_lsls_carry)
{
	zassert_equal(asm_lsls_carry(), 0, "LSLS carry failed");
}
ZTEST(armv7m_asm_selfcheck, test_asrs_sign)
{
	zassert_equal(asm_asrs_sign(), 0, "ASRS sign failed");
}
ZTEST(armv7m_asm_selfcheck, test_clz_zero)
{
	zassert_equal(asm_clz_zero(), 0, "CLZ zero failed");
}
ZTEST(armv7m_asm_selfcheck, test_rbit_bit0)
{
	zassert_equal(asm_rbit_bit0(), 0, "RBIT bit0 failed");
}
ZTEST(armv7m_asm_selfcheck, test_mul_basic)
{
	zassert_equal(asm_mul_basic(), 0, "MUL basic failed");
}
ZTEST(armv7m_asm_selfcheck, test_rev_word)
{
	zassert_equal(asm_rev_word(), 0, "REV word failed");
}
ZTEST(armv7m_asm_selfcheck, test_sbfx_sign)
{
	zassert_equal(asm_sbfx_sign(), 0, "SBFX sign extension failed");
}
ZTEST(armv7m_asm_selfcheck, test_it_eq_taken)
{
	zassert_equal(asm_it_eq_taken(), 0, "IT EQ taken failed");
}
ZTEST(armv7m_asm_selfcheck, test_it_ne_skip)
{
	zassert_equal(asm_it_ne_skip(), 0, "IT NE skip failed");
}

/* Load/store cases */
ZTEST(armv7m_asm_selfcheck, test_str_ldr_word)
{
	zassert_equal(asm_str_ldr_word(), 0, "STR/LDR word failed");
}
ZTEST(armv7m_asm_selfcheck, test_pre_index_wb)
{
	zassert_equal(asm_pre_index_wb(), 0, "Pre-index writeback failed");
}
ZTEST(armv7m_asm_selfcheck, test_post_index_wb)
{
	zassert_equal(asm_post_index_wb(), 0, "Post-index writeback failed");
}
ZTEST(armv7m_asm_selfcheck, test_ldrb_zero_ext)
{
	zassert_equal(asm_ldrb_zero_ext(), 0, "LDRB zero-extend failed");
}
ZTEST(armv7m_asm_selfcheck, test_ldrsb_sign_ext)
{
	zassert_equal(asm_ldrsb_sign_ext(), 0, "LDRSB sign-extend failed");
}
ZTEST(armv7m_asm_selfcheck, test_stmia_ldmia)
{
	zassert_equal(asm_stmia_ldmia(), 0, "STMIA/LDMIA failed");
}
