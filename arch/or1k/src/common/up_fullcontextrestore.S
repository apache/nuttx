/**************************************************************************
 * arch/or1k/src/common/up_fullcontextrestore.S
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

#include <nuttx/irq.h>
#include "up_internal.h"

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_fullcontextrestore
 **************************************************************************/

  .globl  up_fullcontextrestore
  .type   up_fullcontextrestore, function
up_fullcontextrestore:

/*
  l.lwz   r30, 4*REG_PC(r3);
  l.mtspr r0,r30,SPR_SYS_EPCR0;

  l.mfspr r31,r0,SPR_SYS_SR;
  l.andi  r31,r31,(SPR_SR_IEE|SPR_SR_TEE);

  l.lwz   r30, 4*REG_SR(r3);
  l.or    r30,r30,r31;
  l.mtspr r0,r30,SPR_SYS_ESR0;
*/

  l.lwz   r1, 4*REG_R1(r3);
  l.lwz   r2, 4*REG_R2(r3);
  l.lwz   r4, 4*REG_R4(r3);
  l.lwz   r5, 4*REG_R5(r3);
  l.lwz   r6, 4*REG_R6(r3);
  l.lwz   r7, 4*REG_R7(r3);
  l.lwz   r8, 4*REG_R8(r3);
  //l.lwz   r9, 4*REG_R9(r3);
  l.lwz   r10, 4*REG_R10(r3);
  l.lwz   r11, 4*REG_R11(r3);
  l.lwz   r12, 4*REG_R12(r3);
  l.lwz   r13, 4*REG_R13(r3);
  l.lwz   r14, 4*REG_R14(r3);
  l.lwz   r15, 4*REG_R15(r3);
  l.lwz   r16, 4*REG_R16(r3);
  l.lwz   r17, 4*REG_R17(r3);
  l.lwz   r18, 4*REG_R18(r3);
  l.lwz   r19, 4*REG_R19(r3);
  l.lwz   r20, 4*REG_R20(r3);
  l.lwz   r21, 4*REG_R21(r3);
  l.lwz   r22, 4*REG_R22(r3);
  l.lwz   r23, 4*REG_R23(r3);
  l.lwz   r24, 4*REG_R24(r3);
  l.lwz   r25, 4*REG_R25(r3);
  l.lwz   r26, 4*REG_R26(r3);
  l.lwz   r27, 4*REG_R27(r3);
  l.lwz   r28, 4*REG_R28(r3);
  l.lwz   r29, 4*REG_R29(r3);
  l.lwz   r30, 4*REG_R30(r3);
  l.lwz   r31, 4*REG_R31(r3);


  /* Get the return value */

  l.lwz   r11, 4*REG_R11(r3);
  l.lwz   r9, 4*REG_PC(r3);

  l.lwz   r3, 4*REG_R3(r3);

  l.jr    r9;
  l.nop;

  //l.rfe;

  .size up_fullcontextrestore, . - up_fullcontextrestore
