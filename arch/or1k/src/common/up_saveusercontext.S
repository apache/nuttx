/**************************************************************************
 * arch/or1k/src/common/up_saveusercontext.S
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
