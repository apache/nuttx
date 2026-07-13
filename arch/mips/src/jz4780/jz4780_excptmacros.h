/*****************************************************************************
 * arch/mips/src/jz4780/jz4780_excptmacros.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 *****************************************************************************/

#ifndef __ARCH_MIPS_SRC_JZ4780_EXCPTMACROS_H
#define __ARCH_MIPS_SRC_JZ4780_EXCPTMACROS_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/jz4780/cp0.h>

#ifdef __ASSEMBLY__

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * Public Symbols
 *****************************************************************************/

/*****************************************************************************
 * Assembly Language Macros
 *****************************************************************************/

/*****************************************************************************
 * Name: EXCPT_PROLOGUE
 *
 * Description:
 *   Provides the "prologue" logic that should appear at the beginning of
 *   every exception handler.
 *
 * On Entry:
 *   sp - Points to the top of the stack
 *   tmp - Is a register the can be modified for scratch usage (after it has
 *   been saved) k0 and k1 - Since we are in an exception handler, these are
 *   available for use
 *
 * At completion:
 *   Register state is saved on the stack; All registers are available for
 *   usage except sp and k1:
 *
 *   - sp points the beginning of the register save area
 *   - k1 holds the value of the STATUS register
 *
 *   The following registers are modified: k0, k1, sp, a0
 *
 *****************************************************************************/

    .macro      EXCPT_PROLOGUE, tmp
    .set        noat

    mfc0        k0, MIPS32_CP0_EPC
    mfc0        k1, MIPS32_CP0_STATUS

    addiu       sp, sp, -XCPTCONTEXT_SIZE

    /* Save the EPC and STATUS in the register context array */

    sw          k0, REG_EPC(sp)
    sw          k1, REG_STATUS(sp)

    /* Save floating point registers */

    mfhi        k0
    sw          k0, REG_MFHI(sp)
    mflo        k0
    sw          k0, REG_MFLO(sp)

    /* Save general purpose registers */

    /* $1: at_reg, assembler temporary */

    sw          $1, REG_AT(sp)

    /* $2-$3 = v0-v1: Return value registers */

    sw          v0, REG_V0(sp)
    sw          v1, REG_V1(sp)

    /* $4-$7 = a0-a3: Argument registers */

    sw          a0, REG_A0(sp)
    sw          a1, REG_A1(sp)
    sw          a2, REG_A2(sp)
    sw          a3, REG_A3(sp)

    /* $8-$15 = t0-t7: Volatile registers */

    sw          t0, REG_T0(sp)
    sw          t1, REG_T1(sp)
    sw          t2, REG_T2(sp)
    sw          t3, REG_T3(sp)
    sw          t4, REG_T4(sp)
    sw          t5, REG_T5(sp)
    sw          t6, REG_T6(sp)
    sw          t7, REG_T7(sp)

    /* $16-$23 = s0-s7: Static registers */

    sw          s0, REG_S0(sp)
    sw          s1, REG_S1(sp)
    sw          s2, REG_S2(sp)
    sw          s3, REG_S3(sp)
    sw          s4, REG_S4(sp)
    sw          s5, REG_S5(sp)
    sw          s6, REG_S6(sp)
    sw          s7, REG_S7(sp)

    /* $24-25 = t8-t9: More Volatile registers */

    sw          t8, REG_T8(sp)
    sw          t9, REG_T9(sp)

#ifdef MIPS32_SAVE_GP
    sw          gp, REG_GP(sp)
#endif

    /* $30 = either s8 or fp:  Depends if a frame pointer is used or not */

    sw          s8, REG_S8(sp)

    /* $31 = ra: Return address */

    sw          ra, REG_RA(sp)

    addiu       \tmp, sp, XCPTCONTEXT_SIZE
    sw          \tmp, REG_SP(sp)
    .endm

/*****************************************************************************
 * Name: EXCPT_EPILOGUE
 *
 * Description:
 *   Provides the "epilogue" logic that should appear at the end of every
 *   exception handler.
 *
 * On input:
 *   regs - points to the register save structure.  NOTE:  This *may not* be
 *          an address lying in a stack!  It might be an address in a TCB!
 *   Interrupts are disabled (via 'di')
 *
 * On completion:
 *   All registers restored
 *     eret is executed to return from the exception
 *
 *****************************************************************************/

    .macro      EXCPT_EPILOGUE, regs
    .set        noat

    /* Use k1 as the pointer to the register save array.  */

    move        k1, \regs

    /* Restore the floating point register state */

    lw          k0, REG_MFLO(k1)
    mtlo        k0
    lw          k0, REG_MFHI(k1)
    mthi        k0

    /* Restore general purpose registers */

    /* $1: at_reg, assembler temporary */

    lw          $1, REG_AT(k1)

    /* $2-$3 = v0-v1: Return value registers */

    lw          v0, REG_V0(k1)
    lw          v1, REG_V1(k1)

    /* $4-$7 = a0-a3: Argument registers */

    lw          a0, REG_A0(k1)
    lw          a1, REG_A1(k1)
    lw          a2, REG_A2(k1)
    lw          a3, REG_A3(k1)

    /* $8-$15 = t0-t7: Volatile registers */

    lw          t0, REG_T0(k1)
    lw          t1, REG_T1(k1)
    lw          t2, REG_T2(k1)
    lw          t3, REG_T3(k1)
    lw          t4, REG_T4(k1)
    lw          t5, REG_T5(k1)
    lw          t6, REG_T6(k1)
    lw          t7, REG_T7(k1)

    /* $16-$23 = s0-s7: Static registers */

    lw          s0, REG_S0(k1)
    lw          s1, REG_S1(k1)
    lw          s2, REG_S2(k1)
    lw          s3, REG_S3(k1)
    lw          s4, REG_S4(k1)
    lw          s5, REG_S5(k1)
    lw          s6, REG_S6(k1)
    lw          s7, REG_S7(k1)

    /* $24-25 = t8-t9: More Volatile registers */

    lw          t8, REG_T8(k1)
    lw          t9, REG_T9(k1)

#ifdef MIPS32_SAVE_GP
    lw          gp, REG_GP(k1)
#endif

    /* $29 = sp: Stack pointer */

    lw          sp, REG_SP(k1)

    /* $30 = either s8 or fp:  Depends if a frame pointer is used or not */

    lw          s8, REG_S8(k1)

    /* $31 = ra: Return address */

    lw          ra, REG_RA(k1)

    /* Finally, restore CP status and the EPC */

    lw          k0, REG_STATUS(k1)
    lw          k1, REG_EPC(k1)
    mtc0        k0, MIPS32_CP0_STATUS
    ehb
    mtc0        k1, MIPS32_CP0_EPC
    eret
    nop
    .endm

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_JZ4780_EXCPTMACROS_H */
