/**************************************************************************
 * arch/or1k/src/common/up_saveusercontext.S
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <arch/irq.h>
#include "up_internal.h"

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_saveusercontext
 **************************************************************************/

  .text
  .globl  up_saveusercontext
  .type   up_saveusercontext, function
up_saveusercontext:

  /* r3 holds the address of the xcptcontext structure */

  l.sw    4*REG_R1(r3), r1;
  l.sw    4*REG_R2(r3), r2;
  l.sw    4*REG_R3(r3), r3;
  l.sw    4*REG_R4(r3), r4;
  l.sw    4*REG_R5(r3), r5;
  l.sw    4*REG_R6(r3), r6;
  l.sw    4*REG_R7(r3), r7;
  l.sw    4*REG_R8(r3), r8;
  l.sw    4*REG_R9(r3), r9;
  l.sw    4*REG_R10(r3), r10;
  l.sw    4*REG_R12(r3), r12;
  l.sw    4*REG_R13(r3), r13;
  l.sw    4*REG_R14(r3), r14;
  l.sw    4*REG_R15(r3), r15;
  l.sw    4*REG_R16(r3), r16;
  l.sw    4*REG_R17(r3), r17;
  l.sw    4*REG_R18(r3), r18;
  l.sw    4*REG_R19(r3), r19;
  l.sw    4*REG_R20(r3), r20;
  l.sw    4*REG_R21(r3), r21;
  l.sw    4*REG_R22(r3), r22;
  l.sw    4*REG_R23(r3), r23;
  l.sw    4*REG_R24(r3), r24;
  l.sw    4*REG_R25(r3), r25;
  l.sw    4*REG_R26(r3), r26;
  l.sw    4*REG_R27(r3), r27;
  l.sw    4*REG_R28(r3), r28;
  l.sw    4*REG_R29(r3), r29;
  l.sw    4*REG_R30(r3), r30;
  l.sw    4*REG_R31(r3), r31;

  /* Store the return address as the saved PC */
  l.sw    4*REG_PC(r3), r9;

  /* Store the SR */
  l.mfspr r30,r0,SPR_SYS_SR;
  l.sw    4*REG_SR(r3), r30;

  /* Store a non-zero return address in the saved context */
  l.ori   r11,r0,0x1
  l.sw    4*REG_R11(r3), r11;

  /* Return 0 */
  l.ori   r11,r0,0x0
  l.jr    r9
  l.nop

  .size	up_saveusercontext, . - up_saveusercontext
