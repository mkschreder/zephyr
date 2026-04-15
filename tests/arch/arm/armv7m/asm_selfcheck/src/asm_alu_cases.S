/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M pure assembly self-checking ALU tests.
 *
 * Each function returns 0 on pass, nonzero on fail.
 * Uses only caller-saved registers (r0-r3, r12) to avoid frame setup.
 *
 * Reference: DDI0403E
 *   ADD A7.7.3-4 lines 5601-5814
 *   ADC A7.7.1-2 lines 5465-5600
 *   SUB A7.7.176
 *   SBC A7.7.119-120
 *   AND A7.7.8-9 lines 6111-6246
 *   ORR A7.7.91-92 lines 11205-11342
 *   EOR A7.7.35-36 lines 7592-7727
 *   CLZ A7.7.24 lines 7065-7114
 *   RBIT A7.7.112 lines 12485-12532
 *   MUL A7.7.84 lines 10831-10902
 */

.syntax unified
.thumb

/* -----------------------------------------------------------------------
 * asm_adds_zero_carry: ADDS 0xFFFFFFFF + 1 => Z=1, C=1, N=0, V=0
 * --------------------------------------------------------------------- */
.global asm_adds_zero_carry
.type   asm_adds_zero_carry, %function
asm_adds_zero_carry:
    movw r1, #0xFFFF
    movt r1, #0xFFFF    /* r1 = 0xFFFFFFFF */
    movs r2, #1
    adds r3, r1, r2     /* r3 = 0, C=1, Z=1 */
    bne  .Lf_adds_zc    /* Z must be set */
    bcc  .Lf_adds_zc    /* C must be set */
    bmi  .Lf_adds_zc    /* N must be clear */
    bvs  .Lf_adds_zc    /* V must be clear */
    cmp  r3, #0
    bne  .Lf_adds_zc
    movs r0, #0
    bx   lr
.Lf_adds_zc:
    movs r0, #1
    bx   lr
.size asm_adds_zero_carry, . - asm_adds_zero_carry

/* -----------------------------------------------------------------------
 * asm_adds_signed_overflow: ADDS 0x7FFFFFFF + 1 => N=1, V=1, C=0, Z=0
 * --------------------------------------------------------------------- */
.global asm_adds_signed_overflow
.type   asm_adds_signed_overflow, %function
asm_adds_signed_overflow:
    movw r1, #0xFFFF
    movt r1, #0x7FFF    /* r1 = 0x7FFFFFFF */
    movs r2, #1
    adds r3, r1, r2     /* r3 = 0x80000000, N=1, V=1 */
    bpl  .Lf_sov        /* N must be set */
    bvc  .Lf_sov        /* V must be set */
    bcs  .Lf_sov        /* C must be clear */
    beq  .Lf_sov        /* Z must be clear */
    movw r0, #0
    movt r0, #0x8000
    cmp  r3, r0
    bne  .Lf_sov
    movs r0, #0
    bx   lr
.Lf_sov:
    movs r0, #2
    bx   lr
.size asm_adds_signed_overflow, . - asm_adds_signed_overflow

/* -----------------------------------------------------------------------
 * asm_subs_borrow: SUBS 0 - 1 => N=1, C=0, V=0, Z=0
 * --------------------------------------------------------------------- */
.global asm_subs_borrow
.type   asm_subs_borrow, %function
asm_subs_borrow:
    movs r1, #0
    movs r2, #1
    subs r3, r1, r2     /* r3 = 0xFFFFFFFF */
    bcs  .Lf_borrow     /* C=0 (borrow set) */
    bpl  .Lf_borrow     /* N=1 */
    beq  .Lf_borrow     /* Z=0 */
    bvs  .Lf_borrow     /* V=0 */
    movw r0, #0xFFFF
    movt r0, #0xFFFF
    cmp  r3, r0
    bne  .Lf_borrow
    movs r0, #0
    bx   lr
.Lf_borrow:
    movs r0, #3
    bx   lr
.size asm_subs_borrow, . - asm_subs_borrow

/* -----------------------------------------------------------------------
 * asm_ands_zero: ANDS 0xAA, 0x55 => result=0, Z=1
 * --------------------------------------------------------------------- */
.global asm_ands_zero
.type   asm_ands_zero, %function
asm_ands_zero:
    movs r1, #0xAA
    movs r2, #0x55
    ands r3, r1, r2
    bne  .Lf_and0
    movs r0, #0
    bx   lr
.Lf_and0:
    movs r0, #4
    bx   lr
.size asm_ands_zero, . - asm_ands_zero

/* -----------------------------------------------------------------------
 * asm_eors_identity: EOR r, r => 0, Z=1
 * --------------------------------------------------------------------- */
.global asm_eors_identity
.type   asm_eors_identity, %function
asm_eors_identity:
    movw r1, #0x1234
    movt r1, #0x5678    /* r1 = 0x56781234 */
    eors r2, r1, r1
    bne  .Lf_eor_id
    movs r0, #0
    bx   lr
.Lf_eor_id:
    movs r0, #5
    bx   lr
.size asm_eors_identity, . - asm_eors_identity

/* -----------------------------------------------------------------------
 * asm_lsls_carry: LSL 0x80000000 by 1 => C=1, result=0, Z=1
 * --------------------------------------------------------------------- */
.global asm_lsls_carry
.type   asm_lsls_carry, %function
asm_lsls_carry:
    movw r1, #0
    movt r1, #0x8000    /* r1 = 0x80000000 */
    lsls r2, r1, #1     /* C=1, result=0 */
    bcc  .Lf_lsl_c      /* C must be set */
    bne  .Lf_lsl_c      /* Z must be set */
    movs r0, #0
    bx   lr
.Lf_lsl_c:
    movs r0, #6
    bx   lr
.size asm_lsls_carry, . - asm_lsls_carry

/* -----------------------------------------------------------------------
 * asm_asrs_sign: ASR -1 by 1 => result=-1, N=1, C=1
 * --------------------------------------------------------------------- */
.global asm_asrs_sign
.type   asm_asrs_sign, %function
asm_asrs_sign:
    mvn  r1, #0         /* r1 = 0xFFFFFFFF = -1 */
    asrs r2, r1, #1     /* r2 = 0xFFFFFFFF, C=1 (bit0 of 0xFFFFFFFF) */
    bcc  .Lf_asr_s      /* C must be set */
    bpl  .Lf_asr_s      /* N must be set */
    mvn  r0, #0
    cmp  r2, r0
    bne  .Lf_asr_s
    movs r0, #0
    bx   lr
.Lf_asr_s:
    movs r0, #7
    bx   lr
.size asm_asrs_sign, . - asm_asrs_sign

/* -----------------------------------------------------------------------
 * asm_clz_zero: CLZ 0 => 32
 * --------------------------------------------------------------------- */
.global asm_clz_zero
.type   asm_clz_zero, %function
asm_clz_zero:
    movs r1, #0
    clz  r2, r1
    cmp  r2, #32
    beq  .Lok_clz
    movs r0, #8
    bx   lr
.Lok_clz:
    movs r0, #0
    bx   lr
.size asm_clz_zero, . - asm_clz_zero

/* -----------------------------------------------------------------------
 * asm_rbit_bit0: RBIT 1 => 0x80000000
 * --------------------------------------------------------------------- */
.global asm_rbit_bit0
.type   asm_rbit_bit0, %function
asm_rbit_bit0:
    movs r1, #1
    rbit r2, r1
    movw r0, #0
    movt r0, #0x8000
    cmp  r2, r0
    beq  .Lok_rbit
    movs r0, #9
    bx   lr
.Lok_rbit:
    movs r0, #0
    bx   lr
.size asm_rbit_bit0, . - asm_rbit_bit0

/* -----------------------------------------------------------------------
 * asm_mul_basic: MUL 7 * 6 => 42
 * --------------------------------------------------------------------- */
.global asm_mul_basic
.type   asm_mul_basic, %function
asm_mul_basic:
    movs r1, #7
    movs r2, #6
    mul  r3, r1, r2
    cmp  r3, #42
    beq  .Lok_mul
    movs r0, #10
    bx   lr
.Lok_mul:
    movs r0, #0
    bx   lr
.size asm_mul_basic, . - asm_mul_basic

/* -----------------------------------------------------------------------
 * asm_rev_word: REV 0x12345678 => 0x78563412
 * --------------------------------------------------------------------- */
.global asm_rev_word
.type   asm_rev_word, %function
asm_rev_word:
    movw r1, #0x5678
    movt r1, #0x1234    /* r1 = 0x12345678 */
    rev  r2, r1
    movw r0, #0x3412
    movt r0, #0x7856    /* expected 0x78563412 */
    cmp  r2, r0
    beq  .Lok_rev
    movs r0, #11
    bx   lr
.Lok_rev:
    movs r0, #0
    bx   lr
.size asm_rev_word, . - asm_rev_word

/* -----------------------------------------------------------------------
 * asm_sbfx_sign: SBFX bits[4:1] of 0x10 => -8
 * --------------------------------------------------------------------- */
.global asm_sbfx_sign
.type   asm_sbfx_sign, %function
asm_sbfx_sign:
    movs r1, #0x10      /* bit 4 set */
    sbfx r2, r1, #1, #4 /* extract bits[4:1], width=4, sign extends from bit3 */
    mvn  r0, #7         /* r0 = -8 = 0xFFFFFFF8 */
    cmp  r2, r0
    beq  .Lok_sbfx
    movs r0, #12
    bx   lr
.Lok_sbfx:
    movs r0, #0
    bx   lr
.size asm_sbfx_sign, . - asm_sbfx_sign

/* -----------------------------------------------------------------------
 * asm_it_eq_taken: IT EQ; MOVEQ => verifies IT block execution
 * --------------------------------------------------------------------- */
.global asm_it_eq_taken
.type   asm_it_eq_taken, %function
asm_it_eq_taken:
    movs r1, #5
    cmp  r1, r1         /* Z=1 */
    movs r0, #0
    it   eq
    moveq r0, #1        /* should execute */
    cmp  r0, #1
    beq  .Lok_it
    movs r0, #13
    bx   lr
.Lok_it:
    movs r0, #0
    bx   lr
.size asm_it_eq_taken, . - asm_it_eq_taken

/* -----------------------------------------------------------------------
 * asm_it_ne_skip: IT NE; MOVNE skipped when Z=1
 * Key: use MOV (no flag update) to set sentinel BEFORE the CMP,
 * so the CMP's Z=1 is not clobbered before the IT.
 * --------------------------------------------------------------------- */
.global asm_it_ne_skip
.type   asm_it_ne_skip, %function
asm_it_ne_skip:
    mov  r0, #0xFF      /* set sentinel WITHOUT updating flags */
    movs r1, #5
    cmp  r1, r1         /* Z=1 -- no instruction after this touches flags */
    it   ne
    movne r0, #0        /* NE is false (Z=1), so this SHOULD be skipped */
    cmp  r0, #0xFF
    beq  .Lok_ne_skip
    movs r0, #14
    bx   lr
.Lok_ne_skip:
    movs r0, #0
    bx   lr
.size asm_it_ne_skip, . - asm_it_ne_skip
