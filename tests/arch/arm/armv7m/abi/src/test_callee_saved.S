/* SPDX-License-Identifier: Apache-2.0 */
/*
 * test_check_callee_saved_asm - Verify r4-r7 survive a C function call.
 *
 * void test_check_callee_saved_asm(uint32_t *r4_out, uint32_t *r5_out,
 *                                  uint32_t *r6_out, uint32_t *r7_out,
 *                                  void (*fn)(void));
 *
 * Loads known values into r4-r7, calls fn(), then stores r4-r7 to the
 * output pointers.  All done in assembly so GCC has no opportunity to
 * re-arrange register allocation.
 */

	.syntax unified
	.thumb
	.text

	.global test_check_callee_saved_asm
	.type   test_check_callee_saved_asm, %function
test_check_callee_saved_asm:
	/* r0 = r4_out, r1 = r5_out, r2 = r6_out, r3 = r7_out
	 * [sp+0] = fn (5th arg, pushed by caller) */
	push    {r4, r5, r6, r7, lr}

	/* Load known sentinel values into the callee-saved registers */
	movw    r4, #0xAA01
	movt    r4, #0x0000        /* r4 = 0x0000AA01 */
	movw    r5, #0xBB02
	movt    r5, #0x0000        /* r5 = 0x0000BB02 */
	movw    r6, #0xCC03
	movt    r6, #0x0000        /* r6 = 0x0000CC03 */
	movw    r7, #0xDD04
	movt    r7, #0x0000        /* r7 = 0x0000DD04 */

	/* Save output pointers (r0-r3 will be clobbered by call) */
	push    {r0, r1, r2, r3}

	/* Call fn() - must preserve r4-r7 per AAPCS */
	ldr     r0, [sp, #(5*4 + 4*4)]   /* fn is above saved {r4-r7,lr} + {r0-r3} */
	blx     r0

	/* Restore output pointers */
	pop     {r0, r1, r2, r3}

	/* Store post-call register values */
	str     r4, [r0]
	str     r5, [r1]
	str     r6, [r2]
	str     r7, [r3]

	pop     {r4, r5, r6, r7, pc}

	.size test_check_callee_saved_asm, .-test_check_callee_saved_asm
