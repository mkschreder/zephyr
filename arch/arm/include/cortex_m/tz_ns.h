/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief TrustZone API for use in nonsecure firmware
 *
 * TrustZone API for Cortex-M CPUs implementing the Security Extension.
 * The following API can be used by the nonsecure firmware to interact with the
 * secure firmware.
 */

#ifndef ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_CORTEX_M_TZ_NS_H_
#define ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_CORTEX_M_TZ_NS_H_

#ifdef _ASMLANGUAGE

/* nothing */

#else

/**
 * @brief Macro for "sandwiching" a function call (@p name) between two other
 *	  calls
 *
 * This macro should be called via @ref __TZ_WRAP_FUNC.
 *
 * This macro creates the function body of an "outer" function which behaves
 * exactly like the wrapped function (@p name), except that the preface function
 * is called before, and the postface function afterwards.
 *
 * @param preface   The function to call first. Must have no parameters and no
 *                  return value.
 * @param name      The main function, i.e. the function to wrap. This function
 *                  will receive the arguments, and its return value will be
 *                  returned.
 * @param postface  The function to call last. Must have no parameters and no
 *                  return value.
 * @param store_lr  The assembly instruction for storing away the LR value
 *                  before the functions are called. This instruction must leave
 *                  r0-r3 unmodified.
 * @param load_lr   The assembly instruction for restoring the LR value after
 *                  the functions have been called. This instruction must leave
 *                  r0-r3 unmodified.
 */
#define __TZ_WRAP_FUNC_RAW(preface, name, postface, store_lr, load_lr)                             \
	__asm__ volatile(".global " #preface "; .type " #preface ", %function");                   \
	__asm__ volatile(".global " #name "; .type " #name ", %function");                         \
	__asm__ volatile(".global " #postface "; .type " #postface ", %function");                 \
	__asm__ volatile(store_lr "\n\t"                                                           \
				  "push {r0-r3}\n\t"                                               \
				  "bl " #preface "\n\t"                                            \
				  "pop {r0-r3}\n\t"                                                \
				  "bl " #name "\n\t"                                               \
				  "push {r0-r3}\n\t"                                               \
				  "bl " #postface "\n\t"                                           \
				  "pop {r0-r3}\n\t" load_lr "\n\t" ::);

/**
 * @brief Macro for "sandwiching" a function call (@p name) in two other calls
 *
 * @pre The wrapped function MUST not pass arguments or return values via
 *      the stack. I.e. the arguments and return values must each fit within 4
 *      words, after accounting for alignment.
 *      Since nothing is passed on the stack, the stack can safely be used to
 *      store LR.
 *
 * Usage example:
 *
 *	int foo(char *arg); // Implemented elsewhere.
 *	int __attribute__((naked)) foo_wrapped(char *arg)
 *	{
 *		__TZ_WRAP_FUNC(bar, foo, baz)
 *	}
 *
 * is equivalent to
 *
 *	int foo(char *arg); // Implemented elsewhere.
 *	int foo_wrapped(char *arg)
 *	{
 *		bar();
 *		int res = foo(arg);
 *		baz();
 *		return res;
 *	}
 *
 * @note __attribute__((naked)) is not mandatory, but without it, GCC gives a
 *       warning for functions with a return value. It also reduces flash use.
 *
 * See @ref __TZ_WRAP_FUNC_RAW for more information.
 */
#define __TZ_WRAP_FUNC(preface, name, postface)                                                    \
	__TZ_WRAP_FUNC_RAW(preface, name, postface, "push {r4, lr}", "pop {r4, pc}")

#ifdef CONFIG_ARM_FIRMWARE_USES_SECURE_ENTRY_FUNCS
/**
 * @brief Create a thread safe wrapper function for a non-secure entry function
 *
 * This locks the scheduler before calling the function by wrapping the NS entry
 * function in @ref k_sched_lock / @ref k_sched_unlock, using
 * @ref __TZ_WRAP_FUNC.
 *
 * In non-secure code:
 *
 *	int foo(char *arg); // Declaration of entry function.
 *	TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC(foo_safe, int, foo, char *arg)
 *
 * Usage in non-secure code:
 *
 *	int ret = foo_safe("my arg");
 *
 * If NS entry functions are called without such a wrapper, and a thread switch
 * happens while execution is in the secure binary, the app will possibly crash
 * upon returning to the non-secure binary.
 *
 * @param ret   The return type of the NS entry function.
 * @param name  The desired name of the safe function. This assumes there is a
 *              corresponding NS entry function called nsc_name.
 * @param ...   The rest of the signature of the function. This must be the same
 *              signature as the corresponding NS entry function.
 */
#define TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC(name, ret, nsc_name, ...)                              \
	ret __attribute__((naked)) name(__VA_ARGS__)                                               \
	{                                                                                          \
		__TZ_WRAP_FUNC(k_sched_lock, nsc_name, k_sched_unlock);                            \
	}

/*
 * TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC_MUTEX — per-entry-function mutex.
 *
 * A drop-in replacement for TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC that uses a
 * dedicated k_mutex per entry function instead of k_sched_lock / k_sched_unlock.
 *
 * Benefits over the scheduler-lock variant:
 *   - Only threads calling *this specific* entry function are serialized.
 *   - The Zephyr scheduler remains active for unrelated threads.
 *   - Priority inheritance through k_mutex prevents priority inversion.
 *
 * Enabled when CONFIG_ARM_TZ_ENTRY_MUTEX=y.  Falls back to the scheduler-
 * lock variant otherwise to keep the API surface consistent.
 *
 * Usage (Non-Secure firmware side):
 *
 *   int foo(char *arg);  // Declaration of Secure entry function.
 *   TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC_MUTEX(foo_safe, int, foo, char *arg)
 *   // Now call foo_safe("hello") from any NS thread.
 *
 * Limitation: both mutex lock and unlock are void; the entry function return
 * value is passed through unchanged.  Arguments must fit in R0-R3 (≤4 words).
 *
 * Reference: Zephyr k_mutex API; ARM DDI 0553B §C1.4 (NS call serialization).
 */
#if defined(CONFIG_ARM_TZ_ENTRY_MUTEX)

#include <zephyr/kernel.h>

/*
 * Helper: static-duration mutex for each wrapped entry function.
 * Defined at file scope so that the naked wrapper can reference it.
 */
#define _TZ_ENTRY_MUTEX_NAME(name)  _tz_entry_mutex_##name

#define TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC_MUTEX(name, ret, nsc_name, ...)   \
	static K_MUTEX_DEFINE(_TZ_ENTRY_MUTEX_NAME(name));                    \
	static void _tz_lock_##name(void)                                      \
	{                                                                      \
		(void)k_mutex_lock(&_TZ_ENTRY_MUTEX_NAME(name), K_FOREVER);    \
	}                                                                      \
	static void _tz_unlock_##name(void)                                    \
	{                                                                      \
		(void)k_mutex_unlock(&_TZ_ENTRY_MUTEX_NAME(name));             \
	}                                                                      \
	ret __attribute__((naked)) name(__VA_ARGS__)                           \
	{                                                                      \
		__TZ_WRAP_FUNC(_tz_lock_##name, nsc_name, _tz_unlock_##name);  \
	}

#else /* !CONFIG_ARM_TZ_ENTRY_MUTEX — fall back to scheduler lock */

#define TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC_MUTEX(name, ret, nsc_name, ...)   \
	TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC(name, ret, nsc_name, __VA_ARGS__)

#endif /* CONFIG_ARM_TZ_ENTRY_MUTEX */

/*
 * Z_ARM_TZ_NS_ENTRY_WDOG_FEED — Secure-call watchdog integration.
 *
 * Wraps a Secure entry function call and feeds a watchdog channel on each
 * successful round-trip.  If the Non-Secure world stalls, the watchdog fires.
 *
 * Enabled when CONFIG_ARM_TZ_SECURE_CALL_WATCHDOG=y.
 * The board must define a watchdog device alias "tz-wdog".
 *
 * Usage:
 *   int result = Z_ARM_TZ_NS_ENTRY_WDOG_FEED(my_entry_func, arg1, arg2);
 *
 * Reference: ARM DDI 0553B §C1.4; Zephyr WDT API.
 */
#if defined(CONFIG_ARM_TZ_SECURE_CALL_WATCHDOG)
#include <zephyr/drivers/watchdog.h>
#define Z_ARM_TZ_NS_ENTRY_WDOG_FEED(_fn, ...)                                 \
	({                                                                     \
		const struct device *__wdog =                                  \
			DEVICE_DT_GET(DT_ALIAS(tz_wdog));                      \
		__auto_type __ret = (_fn)(__VA_ARGS__);                        \
		if (device_is_ready(__wdog)) {                                 \
			(void)wdt_feed(__wdog, 0);                             \
		}                                                              \
		__ret;                                                         \
	})
#else
#define Z_ARM_TZ_NS_ENTRY_WDOG_FEED(_fn, ...)  ((_fn)(__VA_ARGS__))
#endif /* CONFIG_ARM_TZ_SECURE_CALL_WATCHDOG */

/*
 * Z_ARM_TZ_NS_ENTRY — GPR clearing wrapper for Secure entry functions.
 *
 * When a Secure entry function returns, unused argument / scratch registers
 * (R1–R3, R12) may still contain Secure data.  GCC's
 * __attribute__((cmse_nonsecure_entry)) clears them for functions with
 * declared return types, but struct-return or multi-word returns leave some
 * registers dirty.  This macro emits explicit `mov rN, #0` to scrub all
 * non-result registers before returning to Non-Secure state.
 *
 * @param fn       The Secure entry function to wrap (called internally).
 * @param retsize  Number of 32-bit words in the return value:
 *                 - 0: void   → clear R0–R3, R12
 *                 - 1: 1 word → clear R1–R3, R12
 *                 - 2: 2 word → clear R2–R3, R12
 *                 - 3: 3 word → clear R3, R12
 *                 - 4: 4 word → clear R12 only
 *
 * Reference: ARM DDI 0553B §C1.4.5 GPR clearing rules on Secure return.
 *            ARM AAPCS §6.1.2 (r0–r3 are result / scratch registers).
 *
 * Note: The Z_ARM_TZ_NS_ENTRY macro is a declaration helper, not a call
 * wrapper.  It generates a new naked function that clears registers then
 * branches to @p fn.  Use TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC for the
 * NS-side caller wrapper.
 */
#if defined(CONFIG_ARM_SECURE_FIRMWARE)

/* Clear R0–R3 and R12. */
#define _Z_TZ_CLEAR_REGS_0  "mov r0, #0\n\t mov r1, #0\n\t mov r2, #0\n\t mov r3, #0\n\t mov r12, #0\n\t"
/* Clear R1–R3 and R12. */
#define _Z_TZ_CLEAR_REGS_1  "mov r1, #0\n\t mov r2, #0\n\t mov r3, #0\n\t mov r12, #0\n\t"
/* Clear R2–R3 and R12. */
#define _Z_TZ_CLEAR_REGS_2  "mov r2, #0\n\t mov r3, #0\n\t mov r12, #0\n\t"
/* Clear R3 and R12. */
#define _Z_TZ_CLEAR_REGS_3  "mov r3, #0\n\t mov r12, #0\n\t"
/* Clear R12 only. */
#define _Z_TZ_CLEAR_REGS_4  "mov r12, #0\n\t"

/* Select the right set of clears based on retsize (0–4). */
#define _Z_TZ_CLEAR_SELECT(retsize) _Z_TZ_CLEAR_REGS_ ## retsize

/**
 * @brief Generate a GPR-scrubbing wrapper for a Secure entry function.
 *
 * Declares a new naked function @p wrapper_name that:
 *   1. Branches to @p fn (the real implementation).
 *   2. On return, clears all non-result GPRs before the BXNS return.
 *
 * In practice GCC handles most of the clearing via the cmse_nonsecure_entry
 * attribute; this macro adds belt-and-braces for struct-return cases.
 *
 * @param wrapper_name  Name of the generated wrapper function.
 * @param fn            The underlying __attribute__((cmse_nonsecure_entry))
 *                      function.
 * @param retsize       Return size in 32-bit words (0–4).
 */
#define Z_ARM_TZ_NS_ENTRY(wrapper_name, fn, retsize)                           \
	static void __attribute__((naked)) wrapper_name(void)                  \
	{                                                                       \
		__asm__ volatile(                                               \
			"push {lr}\n\t"                                         \
			"bl   " #fn "\n\t"                                      \
			_Z_TZ_CLEAR_SELECT(retsize)                             \
			"pop  {pc}\n\t"                                         \
			: : : "memory");                                        \
	}

#endif /* CONFIG_ARM_SECURE_FIRMWARE */

#endif /* CONFIG_ARM_FIRMWARE_USES_SECURE_ENTRY_FUNCS */

#endif /* _ASMLANGUAGE */
#endif /* ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_CORTEX_M_TZ_NS_H_ */
