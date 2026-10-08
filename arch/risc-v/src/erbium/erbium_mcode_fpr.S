/****************************************************************************
 * arch/risc-v/src/erbium/erbium_mcode_fpr.S
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
 ****************************************************************************/

/* The trap path does not save the F registers and an FMV instruction
 * encodes its register number as an immediate, so the M-code emulator
 * reaches the live registers through tables of 8-byte FMV + RET pairs.
 * These are the only FP instructions on the emulation path. The table is
 * entered through t1, because a jump through t0 would be a return hint.
 *
 * fmv.x.w sign-extends bit 31, which is how the LP64 ABI returns a
 * uint32_t. On ET-Minion fmv.w.x clears bits 32-255 of the register, as
 * the scalar instruction being emulated would.
 */

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .globl  erbium_mcode_getfpr
  .globl  erbium_mcode_setfpr

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .text
  .option push
  .option norvc

/****************************************************************************
 * Name: erbium_mcode_getfpr
 *
 * Description:
 *   Read the low 32 bits of a live F register.
 *
 * C Function Prototype:
 *   uint32_t erbium_mcode_getfpr(unsigned int reg);
 *
 * Input Parameters:
 *   reg - F register number, 0 to 31.
 *
 * Returned Value:
 *   The register's single-precision value.
 *
 ****************************************************************************/

  .balign 8
  .type   erbium_mcode_getfpr, @function

erbium_mcode_getfpr:
  andi    a0, a0, 31
  slli    a0, a0, 3
  lla     t1, 1f
  add     t1, t1, a0
  jr      t1
  .balign 8
1:
  .irp    n, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
  fmv.x.w a0, f\n
  ret
  .endr
  .irp    n, 16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
  fmv.x.w a0, f\n
  ret
  .endr
  .size   erbium_mcode_getfpr, . - erbium_mcode_getfpr

/****************************************************************************
 * Name: erbium_mcode_setfpr
 *
 * Description:
 *   Write a single-precision value to a live F register.
 *
 * C Function Prototype:
 *   void erbium_mcode_setfpr(unsigned int reg, uint32_t val);
 *
 * Input Parameters:
 *   reg - F register number, 0 to 31.
 *   val - The single-precision value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

  .balign 8
  .type   erbium_mcode_setfpr, @function

erbium_mcode_setfpr:
  andi    a0, a0, 31
  slli    a0, a0, 3
  lla     t1, 1f
  add     t1, t1, a0
  jr      t1
  .balign 8
1:
  .irp    n, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
  fmv.w.x f\n, a1
  ret
  .endr
  .irp    n, 16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
  fmv.w.x f\n, a1
  ret
  .endr
  .size   erbium_mcode_setfpr, . - erbium_mcode_setfpr

  .option pop
