/****************************************************************************
 * libs/libc/machine/xtensa/arch_strnlen.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "xtensa_asm.h"

#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_abi.h>

#include "libc.h"

#ifdef LIBC_BUILD_STRNLEN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(strnlen)
  .type ARCH_LIBCFUN(strnlen), @function
ARCH_LIBCFUN(strnlen):
  ENTRY(16)
  /* a2 = s, a3 = maxlen */

  /* Return 0 if maxlen == 0 */

  beqz  a3, .Lret_zero

  /* Save start pointer */

  mov a10, a2           /* a10 = start */

  /* Byte-by-byte until s is word-aligned */

  bbci.l  a2, 0, .Lcheck2
  l8ui  a8, a2, 0
  beqz  a8, .Lret_len
  addi  a2, a2, 1
  addi  a3, a3, -1
  beqz  a3, .Lret_maxlen_a2
.Lcheck2:
  bbci.l  a2, 1, .Lword_setup
  l8ui  a8, a2, 0
  beqz  a8, .Lret_len
  addi  a2, a2, 1
  addi  a3, a3, -1
  beqz  a3, .Lret_maxlen_a2
  l8ui  a8, a2, 0
  beqz  a8, .Lret_len
  addi  a2, a2, 1
  addi  a3, a3, -1
  beqz  a3, .Lret_maxlen_a2

  /* s is word-aligned. Set up MASKs. */

.Lword_setup:
  movi  a4, MASK0
  movi  a5, MASK1
  movi  a6, MASK2
  movi  a7, MASK3

  /* Word-at-a-time loop using count (a3) */

.Lword_loop:
  bltui a3, 4, .Ltail   /* less than 4 bytes remaining */
  l32i  a8, a2, 0
  bnone a8, a4, .Lz0
  bnone a8, a5, .Lz1
  bnone a8, a6, .Lz2
  bnone a8, a7, .Lz3
  addi  a2, a2, 4
  addi  a3, a3, -4
  j .Lword_loop

  /* Found null byte in word */

.Lz0:
  sub a2, a2, a10
  RET(16)

.Lz1:
  addi  a2, a2, 1
  sub a2, a2, a10
  RET(16)

.Lz2:
  addi  a2, a2, 2
  sub a2, a2, a10
  RET(16)

.Lz3:
  addi  a2, a2, 3
  sub a2, a2, a10
  RET(16)

  /* Tail: byte-by-byte for remaining bytes */

.Ltail:
  beqz  a3, .Lret_maxlen_a2
.Ltail_loop:
  l8ui  a8, a2, 0
  beqz  a8, .Lret_len
  addi  a2, a2, 1
  addi  a3, a3, -1
  bnez  a3, .Ltail_loop

.Lret_maxlen_a2:
  /* Return s - start = total bytes scanned = original maxlen */
  sub a2, a2, a10
  RET(16)

.Lret_zero:
  movi  a2, 0
  RET(16)

.Lret_len:
  sub a2, a2, a10
  RET(16)

  .end schedule

  .size ARCH_LIBCFUN(strnlen), . - ARCH_LIBCFUN(strnlen)

#endif
