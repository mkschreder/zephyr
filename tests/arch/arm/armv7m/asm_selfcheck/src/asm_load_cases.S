/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M pure assembly self-checking load/store tests.
 *
 * Reference: DDI0403E
 *   LDR A7.7.43-45  lines 8145-8412
 *   STR A7.7.161-163
 *   Addressing modes A7.5 lines 5393-5431
 */

.syntax unified
.thumb

/* Scratch buffer in BSS */
.data
.align 4
asm_ls_buf:
    .space 32

/* -----------------------------------------------------------------------
 * asm_str_ldr_word: STR then LDR roundtrip
 * --------------------------------------------------------------------- */
.global asm_str_ldr_word
.type   asm_str_ldr_word, %function
asm_str_ldr_word:
    ldr  r1, =asm_ls_buf
    movw r2, #0xBEEF
    movt r2, #0xDEAD    /* r2 = 0xDEADBEEF */
    str  r2, [r1]
    ldr  r3, [r1]
    cmp  r2, r3
    beq  .Lok_wrd
    movs r0, #1
    bx   lr
.Lok_wrd:
    movs r0, #0
    bx   lr
.size asm_str_ldr_word, . - asm_str_ldr_word

/* -----------------------------------------------------------------------
 * asm_pre_index_wb: STR [base, #4]! verifies writeback
 * --------------------------------------------------------------------- */
.global asm_pre_index_wb
.type   asm_pre_index_wb, %function
asm_pre_index_wb:
    ldr  r1, =asm_ls_buf     /* base */
    movw r2, #0x1234
    movt r2, #0x5678
    str  r2, [r1, #4]!       /* pre-index: store at base+4, r1 += 4 */
    /* r1 should now equal asm_ls_buf + 4 */
    ldr  r3, =asm_ls_buf
    adds r3, r3, #4
    cmp  r1, r3
    bne  .Lf_pwb
    /* verify stored value */
    ldr  r3, [r1]
    cmp  r2, r3
    beq  .Lok_pwb
.Lf_pwb:
    movs r0, #2
    bx   lr
.Lok_pwb:
    movs r0, #0
    bx   lr
.size asm_pre_index_wb, . - asm_pre_index_wb

/* -----------------------------------------------------------------------
 * asm_post_index_wb: LDR [base], #4 verifies post-index writeback
 * --------------------------------------------------------------------- */
.global asm_post_index_wb
.type   asm_post_index_wb, %function
asm_post_index_wb:
    ldr  r1, =asm_ls_buf
    movw r2, #0xABCD
    movt r2, #0xEF01
    str  r2, [r1]            /* store at base */
    ldr  r3, [r1], #4        /* load from base, then r1 += 4 */
    /* r3 should equal stored value */
    cmp  r2, r3
    bne  .Lf_poi
    /* r1 should be asm_ls_buf + 4 */
    ldr  r2, =asm_ls_buf
    adds r2, r2, #4
    cmp  r1, r2
    beq  .Lok_poi
.Lf_poi:
    movs r0, #3
    bx   lr
.Lok_poi:
    movs r0, #0
    bx   lr
.size asm_post_index_wb, . - asm_post_index_wb

/* -----------------------------------------------------------------------
 * asm_ldrb_zero_ext: LDRB 0xFF => 0x000000FF (no sign extension)
 * --------------------------------------------------------------------- */
.global asm_ldrb_zero_ext
.type   asm_ldrb_zero_ext, %function
asm_ldrb_zero_ext:
    ldr  r1, =asm_ls_buf
    movs r2, #0xFF
    strb r2, [r1]
    ldrb r3, [r1]
    cmp  r3, #0xFF
    beq  .Lok_ldrb
    movs r0, #4
    bx   lr
.Lok_ldrb:
    movs r0, #0
    bx   lr
.size asm_ldrb_zero_ext, . - asm_ldrb_zero_ext

/* -----------------------------------------------------------------------
 * asm_ldrsb_sign_ext: LDRSB 0x80 => 0xFFFFFF80
 * --------------------------------------------------------------------- */
.global asm_ldrsb_sign_ext
.type   asm_ldrsb_sign_ext, %function
asm_ldrsb_sign_ext:
    ldr  r1, =asm_ls_buf
    movw r2, #0
    movt r2, #0x80
    lsr  r2, r2, #16         /* r2 = 0x80 */
    strb r2, [r1]
    ldrsb r3, [r1]
    mvn  r2, #0x7F           /* r2 = 0xFFFFFF80 */
    cmp  r3, r2
    beq  .Lok_ldrsb
    movs r0, #5
    bx   lr
.Lok_ldrsb:
    movs r0, #0
    bx   lr
.size asm_ldrsb_sign_ext, . - asm_ldrsb_sign_ext

/* -----------------------------------------------------------------------
 * asm_stmia_ldmia: Store 3 regs then load back and verify
 * --------------------------------------------------------------------- */
.global asm_stmia_ldmia
.type   asm_stmia_ldmia, %function
asm_stmia_ldmia:
    ldr  r0, =asm_ls_buf
    movs r1, #0x11
    movs r2, #0x22
    movs r3, #0x33
    stmia r0, {r1, r2, r3}
    /* reload into different regs */
    ldr  r0, =asm_ls_buf
    push {r4, r5, r6}
    ldmia r0, {r4, r5, r6}
    cmp  r4, #0x11
    bne  .Lf_stm
    cmp  r5, #0x22
    bne  .Lf_stm
    cmp  r6, #0x33
    bne  .Lf_stm
    pop  {r4, r5, r6}
    movs r0, #0
    bx   lr
.Lf_stm:
    pop  {r4, r5, r6}
    movs r0, #6
    bx   lr
.size asm_stmia_ldmia, . - asm_stmia_ldmia
