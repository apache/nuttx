/**************************************************************************
 * arch/or1k/src/common/up_fullcontextrestore.S
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
