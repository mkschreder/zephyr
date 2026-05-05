/*
 * Copyright (c) 2026 Zephyr Project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * SAU register programming and TT instruction tests.
 * DDI 0553B §B3.4, §C2.338-340.
 */

#include <zephyr/ztest.h>
#include <tz_test.h>

/* --- TT with SAU disabled ---------------------------------------------- */

ZTEST(arm_trustzone, test_sau_disabled_allns0_is_secure)
{
	uint32_t saved = tz_sau_save();
	TZ_SAU_CTRL = 0U; /* disabled + ALLNS=0 → all Secure */

	uint32_t result = tz_tt(0x00000000U);

	tz_sau_restore(saved);

	zassert_not_equal(result & TZ_TT_S_BIT, 0U,
		"SAU disabled (ALLNS=0): S must be 1, result=0x%08x", result);
	zassert_equal(result & TZ_TT_SRVALID_BIT, 0U,
		"SAU disabled: SRVALID must be 0, result=0x%08x", result);
}

ZTEST(arm_trustzone, test_sau_disabled_allns1_is_nonsecure)
{
	uint32_t saved = tz_sau_save();
	TZ_SAU_CTRL = TZ_SAU_CTRL_ALLNS; /* disabled + ALLNS=1 → all NS */

	uint32_t result = tz_tt(0x20000000U);

	tz_sau_restore(saved);

	zassert_equal(result & TZ_TT_S_BIT, 0U,
		"SAU ALLNS=1: S must be 0, result=0x%08x", result);
	zassert_equal(result & TZ_TT_SRVALID_BIT, 0U,
		"SAU ALLNS=1: SRVALID must be 0, result=0x%08x", result);
}

/* --- TT with SAU enabled ------------------------------------------------ */

ZTEST(arm_trustzone, test_sau_ns_region_s_zero)
{
	uint32_t saved = tz_sau_save();
	TZ_SAU_CTRL = 0U;
	tz_sau_set_region(0, 0x20000000U, 0x2000FFFFU, false);
	TZ_SAU_CTRL = TZ_SAU_CTRL_ENABLE;

	uint32_t result = tz_tt(0x20000000U);

	tz_sau_restore(saved);

	zassert_equal(result & TZ_TT_S_BIT, 0U,
		"NS region: S must be 0, result=0x%08x", result);
	zassert_not_equal(result & TZ_TT_SRVALID_BIT, 0U,
		"NS region: SRVALID must be 1, result=0x%08x", result);
	zassert_equal(TZ_TT_SREGION(result), 0U,
		"NS region: SREGION must be 0, result=0x%08x", result);
}

ZTEST(arm_trustzone, test_sau_nsc_region_is_secure)
{
	uint32_t saved = tz_sau_save();
	TZ_SAU_CTRL = 0U;
	tz_sau_set_region(1, 0x10000000U, 0x1000FFFFU, true);
	TZ_SAU_CTRL = TZ_SAU_CTRL_ENABLE;

	uint32_t result = tz_tt(0x10000000U);

	tz_sau_restore(saved);

	/* NSC is architecturally Secure from Secure state perspective. */
	zassert_not_equal(result & TZ_TT_S_BIT, 0U,
		"NSC region: S must be 1 (NSC is Secure), result=0x%08x", result);
	zassert_not_equal(result & TZ_TT_SRVALID_BIT, 0U,
		"NSC region: SRVALID must be 1, result=0x%08x", result);
}

ZTEST(arm_trustzone, test_sau_unmatched_defaults_secure)
{
	uint32_t saved = tz_sau_save();
	TZ_SAU_CTRL = 0U;
	tz_sau_set_region(0, 0x20000000U, 0x2000FFFFU, false);
	TZ_SAU_CTRL = TZ_SAU_CTRL_ENABLE;

	uint32_t result = tz_tt(0x40000000U); /* not in any region */

	tz_sau_restore(saved);

	zassert_not_equal(result & TZ_TT_S_BIT, 0U,
		"Unmatched: S must be 1 (default Secure), result=0x%08x", result);
	zassert_equal(result & TZ_TT_SRVALID_BIT, 0U,
		"Unmatched: SRVALID must be 0, result=0x%08x", result);
}

ZTEST(arm_trustzone, test_sau_type_has_min_regions)
{
	uint32_t nregions = TZ_SAU_TYPE & 0xFFU;

	zassert_true(nregions >= 4U,
		"SAU_TYPE.SREGION must be >= 4, got %u", nregions);
}

/* --- SFSR accessibility ------------------------------------------------- */

ZTEST(arm_trustzone, test_sfsr_clears_to_zero)
{
	TZ_SAU_SFSR = 0xFFFFFFFFU; /* w1c clear all */
	uint32_t sfsr = TZ_SAU_SFSR;

	zassert_equal(sfsr, 0U,
		"SFSR must read 0 after clearing, got 0x%08x", sfsr);
}

/* --- AIRCR.BFHFNMINS ---------------------------------------------------- */

ZTEST(arm_trustzone, test_aircr_bfhfnmins_readwrite)
{
	uint32_t saved = TZ_AIRCR;

	TZ_AIRCR = TZ_AIRCR_VECTKEY | TZ_AIRCR_BFHFNMINS;
	uint32_t set_val = TZ_AIRCR;

	TZ_AIRCR = TZ_AIRCR_VECTKEY;
	uint32_t clr_val = TZ_AIRCR;

	/* Restore */
	TZ_AIRCR = TZ_AIRCR_VECTKEY | (saved & ~(uint32_t)0xFFFFU);

	zassert_not_equal(set_val & TZ_AIRCR_BFHFNMINS, 0U,
		"BFHFNMINS must read 1 after setting, got 0x%08x", set_val);
	zassert_equal(clr_val & TZ_AIRCR_BFHFNMINS, 0U,
		"BFHFNMINS must read 0 after clearing, got 0x%08x", clr_val);
}

/* --- Banked MSPLIM_S / MSPLIM_NS --------------------------------------- */

ZTEST(arm_trustzone, test_msplim_banked_registers)
{
	uint32_t orig_s, orig_ns, rb_s, rb_ns;

	/* GAS with -mcmse recognises MSPLIM and MSPLIM_NS as named system
	 * registers (DDI 0553B SYSm=0x0A / 0x1A). */
	__asm__ volatile("mrs %0, MSPLIM\n\t"    : "=r"(orig_s));
	__asm__ volatile("mrs %0, MSPLIM_NS\n\t" : "=r"(orig_ns));

	__asm__ volatile("msr MSPLIM,    %0\n\t" : : "r"(0x20001000U));
	__asm__ volatile("msr MSPLIM_NS, %0\n\t" : : "r"(0x20003000U));

	__asm__ volatile("mrs %0, MSPLIM\n\t"    : "=r"(rb_s));
	__asm__ volatile("mrs %0, MSPLIM_NS\n\t" : "=r"(rb_ns));

	__asm__ volatile("msr MSPLIM,    %0\n\t" : : "r"(orig_s));
	__asm__ volatile("msr MSPLIM_NS, %0\n\t" : : "r"(orig_ns));

	zassert_equal(rb_s & ~7U, 0x20001000U,
		"MSPLIM_S: wrote 0x%08x, read 0x%08x", 0x20001000U, rb_s);
	zassert_equal(rb_ns & ~7U, 0x20003000U,
		"MSPLIM_NS: wrote 0x%08x, read 0x%08x", 0x20003000U, rb_ns);
	zassert_not_equal(rb_s & ~7U, rb_ns & ~7U,
		"MSPLIM_S and MSPLIM_NS must be independent, both=0x%08x", rb_s);
}

#undef _MRS_R0_MSPLIM_S
#undef _MSR_MSPLIM_S_R0
#undef _MRS_R0_MSPLIM_NS
#undef _MSR_MSPLIM_NS_R0
