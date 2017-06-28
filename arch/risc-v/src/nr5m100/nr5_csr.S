/************************************************************************************
 * arch/risc-v/src/nr5m100/nr5_csr.S
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/*
 * Provide C level access function to asm only CSR registers
 */

#define ENABLE_QREGS
#include "nr5_custom_ops.h"
#include "nr5_csr.h"

	.global up_getmisa
	.global up_getarchid
	.global up_getimpid
	.global up_getvendorid
	.global up_gethartid
	.global up_getepicprimask
	.global up_setepicprimask
	.global up_setepicstat
	.global up_setsystick
	.global up_setpri1bit
	.global up_clearpri1bit
	.global up_setpri2bit
	.global up_clearpri2bit
	.global up_setpri3bit
	.global up_clearpri3bit
	.global up_setirqmaskbit
	.global up_clearirqmaskbit
	.global up_disableints
	.global up_enableints
	.global up_getq0
	.global up_getq1
	.global up_getq2
	.global up_setq0
	.global up_setq1
	.global up_setq2
	.global up_lsbenc

	.section .text

/****************************************************************************
 * Returns the Read Only RV32IM Machine ISA (capabilities)
 ****************************************************************************/

up_getmisa:
    csrr  a0, CSR_MISA
    ret

/****************************************************************************
 * Returns the Read Only RV32IM Vendor ID
 ****************************************************************************/

up_getvendorid:
    csrr  a0, CSR_MVENDORID
    ret

/****************************************************************************
 * Returns the Read Only RV32IM ARCH ID
 ****************************************************************************/

up_getarchid:
    csrr  a0, CSR_MARCHID
    ret

/****************************************************************************
 * Returns the Read Only RV32IM IMP ID
 ****************************************************************************/

up_getimpid:
    csrr  a0, CSR_MIMPID
    ret

/****************************************************************************
 * Returns the Read Only RV32IM HART ID
 ****************************************************************************/

up_gethartid:
    csrr  a0, CSR_MHARTID
    ret

/****************************************************************************
 * Returns the NR5M100 specific Embedded Priority Interrupt Controller (EPIC)
 * IRQ MASK register
 ****************************************************************************/

up_getepicmask:
    csrr  a0, NR5_EPIC_IRQ_MASK
    ret

/****************************************************************************
 * Returns the NR5M100 specific EPIC IRQ Priority register
 ****************************************************************************/

up_getepicpri:
    csrr  a0, NR5_EPIC_PRIMASK
    ret

/****************************************************************************
 * Sets the NR5M100 specific EPIC IRQ Priority register
 ****************************************************************************/

up_setepicpri:
    csrrw  a0, NR5_EPIC_PRIMASK, a0
    ret

/****************************************************************************
 * Sets the NR5M100 specific SYSTICK control register
 ****************************************************************************/

up_setsystick:
    csrw  NR5_MSYSTICK_REG, a0
    ret

/****************************************************************************
 * Sets bits in the NR5M100 specific PRI1 mask control register
 ****************************************************************************/

up_setpri1bit:
    csrrs a0, NR5_EPIC_PRI1, a0
    ret

/****************************************************************************
 * Clears bits in the NR5M100 specific PRI1 mask control register
 ****************************************************************************/

up_clearpri1bit:
    csrrc a0, NR5_EPIC_PRI1, a0
    ret

/****************************************************************************
 * Sets bits in the NR5M100 specific PRI2 mask control register
 ****************************************************************************/

up_setpri2bit:
    csrrs a0, NR5_EPIC_PRI2, a0
    ret

/****************************************************************************
 * Clears bits in the NR5M100 specific PRI2 mask control register
 ****************************************************************************/

up_clearpri2bit:
    csrrc a0, NR5_EPIC_PRI2, a0
    ret

/****************************************************************************
 * Sets bits in the NR5M100 specific PRI3 mask control register
 ****************************************************************************/

up_setpri3bit:
    csrrs a0, NR5_EPIC_PRI3, a0
    ret

/****************************************************************************
 * Clears bits in the NR5M100 specific PRI3 mask control register
 ****************************************************************************/

up_clearpri3bit:
    csrrc a0, NR5_EPIC_PRI3, a0
    ret

/****************************************************************************
 * Sets bits in the NR5M100 specific IRQ mask control register
 ****************************************************************************/

up_setirqmaskbit:
    csrrs a0, NR5_EPIC_IRQ_MASK, a0
    ret

/****************************************************************************
 * Clears bits in the NR5M100 specific IRQ mask control register
 ****************************************************************************/

up_clearirqmaskbit:
    csrrc a0, NR5_EPIC_IRQ_MASK, a0
    ret

/****************************************************************************
 * Disables global interrupts in NR5M100 specific IRQ PRI control register
 ****************************************************************************/

up_disableints:
    csrrc a0, NR5_EPIC_PRIMASK, 1
    ret

/****************************************************************************
 * Enables global interrupts in NR5M100 specific IRQ PRI control register
 ****************************************************************************/

up_enableints:
    csrrs a0, NR5_EPIC_PRIMASK, 1
    ret

/****************************************************************************
 * Reads NR5M100 specific Q0 register (used for interrupt processing)
 ****************************************************************************/

up_getq0:
    getq a0, q0
    ret

/****************************************************************************
 * Reads NR5M100 specific Q1 register (used for interrupt processing)
 ****************************************************************************/

up_getq1:
    getq a0, q1
    ret

/****************************************************************************
 * Reads NR5M100 specific Q2 register (used for interrupt processing)
 ****************************************************************************/

up_getq2:
    getq a0, q2
    ret

/****************************************************************************
 * Sets NR5M100 specific Q0 register (used for interrupt processing)
 ****************************************************************************/

up_setq0:
    setq q0, a0
    ret

/****************************************************************************
 * Sets NR5M100 specific Q1 register (used for interrupt processing)
 ****************************************************************************/

up_setq1:
    setq q1, a0
    ret

/****************************************************************************
 * Sets NR5M100 specific Q2 register (used for interrupt processing)
 ****************************************************************************/

up_setq2:
    setq q2, a0
    ret

/****************************************************************************
 * Calls the  NR5M100 specific LSBENC opcode.  This opcode will find the
 * first least significant non-zero bit in a0 and return it's ordinal value.
 ****************************************************************************/

up_lsbenc:
   lsbenc a0, a0
   ret

/****************************************************************************
 * Modeline to set vim formatting options for ASM file.  For this to work,
 * you must enable moeline processing in your ~/.vimrc file with:
 *
 *    ~/.vimrc:
 *      set modeline
 *
 * vim: noet:ts=4:sw=4
 ****************************************************************************/

