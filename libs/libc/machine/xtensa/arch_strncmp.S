/****************************************************************************
 * libs/libc/machine/xtensa/arch_strncmp.S
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

#ifdef LIBC_BUILD_STRNCMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(strncmp)
  .type ARCH_LIBCFUN(strncmp), @function
ARCH_LIBCFUN(strncmp):
  ENTRY(16)
  /* a2 = s1, a3 = s2, a4 = n */

  /* Return 0 if n == 0 */

  beqz  a4, .Lret_zero

  /* Byte compare until s1 is word-aligned, also handle short n */

.Lbyte_head:
  bbci.l  a2, 0, .Lcheck2
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  addi  a3, a3, 1
  addi  a4, a4, -1
  bne a8, a9, .Ldiff_byte
  beqz  a8, .Lret_zero  /* null terminator */
  beqz  a4, .Lret_zero
.Lcheck2:
  bbci.l  a2, 1, .Lcheck_word
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  addi  a3, a3, 1
  addi  a4, a4, -1
  bne a8, a9, .Ldiff_byte
  beqz  a8, .Lret_zero
  beqz  a4, .Lret_zero
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  addi  a3, a3, 1
  addi  a4, a4, -1
  bne a8, a9, .Ldiff_byte
  beqz  a8, .Lret_zero
  beqz  a4, .Lret_zero

  /* s1 is word-aligned. Check if s2 is also aligned. */

.Lcheck_word:
  bbsi.l  a3, 0, .Ltail_loop
  bbsi.l  a3, 1, .Ltail_loop

  /* Both aligned. Word-at-a-time loop with null and diff detection. */

  movi  a5, MASK0
  movi  a6, MASK1
  movi  a7, MASK2

.Laligned_loop:
  bltui a4, 4, .Ltail_loop
  l32i  a8, a2, 0
  l32i  a9, a3, 0
  addi  a2, a2, 4
  addi  a3, a3, 4
  addi  a4, a4, -4
  bne a8, a9, .Lfind_diff
  /* Check for null in w1 */
  bnone a8, a5, .Lret_zero  /* byte0 is null */
  bnone a8, a6, .Lret_zero  /* byte1 is null */
  bnone a8, a7, .Lret_zero  /* byte2 is null */
  movi  a10, MASK3
  bnone a8, a10, .Lret_zero /* byte3 is null */
  j .Laligned_loop

  /* Words differ. Find first diff, but null before diff → equal. */

.Lfind_diff:
  xor a10, a8, a9

  /* byte0 */
  bnone a8, a5, .Lret_zero    /* null at byte0 → equal */
  bany  a10, a5, .Ldiff0      /* diff at byte0 */

  /* byte1 */
  bnone a8, a6, .Lret_zero
  bany  a10, a6, .Ldiff1

  /* byte2 */
  bnone a8, a7, .Lret_zero
  bany  a10, a7, .Ldiff2

  /* byte3: null check */
  movi  a11, MASK3
  bnone a8, a11, .Lret_zero

  /* byte3 differs */
#if XCHAL_HAVE_BE
  extui a8, a8, 0, 8
  extui a9, a9, 0, 8
#else
  extui a8, a8, 24, 8
  extui a9, a9, 24, 8
#endif
  sub a2, a8, a9
  RET(16)

.Ldiff0:
#if XCHAL_HAVE_BE
  extui a8, a8, 24, 8
  extui a9, a9, 24, 8
#else
  extui a8, a8, 0, 8
  extui a9, a9, 0, 8
#endif
  sub a2, a8, a9
  RET(16)

.Ldiff1:
#if XCHAL_HAVE_BE
  extui a8, a8, 16, 8
  extui a9, a9, 16, 8
#else
  extui a8, a8, 8, 8
  extui a9, a9, 8, 8
#endif
  sub a2, a8, a9
  RET(16)

.Ldiff2:
#if XCHAL_HAVE_BE
  extui a8, a8, 8, 8
  extui a9, a9, 8, 8
#else
  extui a8, a8, 16, 8
  extui a9, a9, 16, 8
#endif
  sub a2, a8, a9
  RET(16)

  /* Tail: byte-by-byte for unaligned s2 or remaining bytes */

.Ltail_loop:
  beqz  a4, .Lret_zero
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  addi  a3, a3, 1
  bne a8, a9, .Ldiff_byte
  beqz  a8, .Lret_zero
  addi  a4, a4, -1
  bnez  a4, .Ltail_loop

.Lret_zero:
  movi  a2, 0
  RET(16)

.Ldiff_byte:
  sub a2, a8, a9
  RET(16)

  .end schedule

  .size ARCH_LIBCFUN(strncmp), . - ARCH_LIBCFUN(strncmp)

#endif
