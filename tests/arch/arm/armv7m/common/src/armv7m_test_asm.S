/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M torture test harness -- assembly helpers.
 *
 * armv7m_capture_sig(struct armv7m_sig *sig)
 *   Captures r0-r12, SP, LR, return address, and xPSR into *sig.
 *   The function is called with r0 pointing to the sig struct.
 *
 * struct armv7m_sig layout (must match armv7m_test.h):
 *   Offset  0: r[0]  .. r[12]  (13 x uint32_t = 52 bytes)
 *   Offset 52: sp    (uint32_t)
 *   Offset 56: lr    (uint32_t)
 *   Offset 60: pc    (uint32_t, holds return address at capture time)
 *   Offset 64: xpsr  (uint32_t)
 */

.syntax unified
.thumb

.global armv7m_capture_sig
.type   armv7m_capture_sig, %function
armv7m_capture_sig:
    /* r0 = pointer to struct armv7m_sig.
     * We must save r0 first (it is the struct pointer, but also r[0]
     * in the sig).  Use r1 as scratch; save r1 to the sig immediately. */
    push    {r1, r2}

    /* r[1] = original r1; store now (r0 still intact) */
    ldr     r1, [sp, #0]        /* original r1 from stack */
    str     r1, [r0, #4]        /* sig->r[1] */

    /* r[2] = original r2 */
    ldr     r2, [sp, #4]        /* original r2 from stack */
    str     r2, [r0, #8]        /* sig->r[2] */

    pop     {r1, r2}

    /* Now store r0 (the struct pointer itself WAS the caller's r0).
     * We record the caller's r0 as the first argument value -- callers
     * that need r0 saved should do so before calling this function.
     * We store the pointer itself so the sig is self-consistent. */
    str     r0,  [r0, #0]       /* sig->r[0]  = r0 (struct pointer) */
    str     r3,  [r0, #12]      /* sig->r[3]  */
    str     r4,  [r0, #16]      /* sig->r[4]  */
    str     r5,  [r0, #20]      /* sig->r[5]  */
    str     r6,  [r0, #24]      /* sig->r[6]  */
    str     r7,  [r0, #28]      /* sig->r[7]  */
    str     r8,  [r0, #32]      /* sig->r[8]  */
    str     r9,  [r0, #36]      /* sig->r[9]  */
    str     r10, [r0, #40]      /* sig->r[10] */
    str     r11, [r0, #44]      /* sig->r[11] */
    str     r12, [r0, #48]      /* sig->r[12] */

    /* SP: capture current SP (post call, so caller SP + 0 relative) */
    mov     r1, sp
    str     r1,  [r0, #52]      /* sig->sp */

    /* LR: holds the return address to our caller */
    str     lr,  [r0, #56]      /* sig->lr */

    /* PC: we store the LR value as the "PC at time of capture" */
    str     lr,  [r0, #60]      /* sig->pc = return address */

    /* xPSR via MRS */
    mrs     r1, xpsr
    str     r1,  [r0, #64]      /* sig->xpsr */

    bx      lr
.size armv7m_capture_sig, . - armv7m_capture_sig
