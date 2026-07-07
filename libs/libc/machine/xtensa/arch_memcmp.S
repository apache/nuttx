/****************************************************************************
 * libs/libc/machine/xtensa/arch_memcmp.S
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

#ifdef LIBC_BUILD_MEMCMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(memcmp)
  .type ARCH_LIBCFUN(memcmp), @function
ARCH_LIBCFUN(memcmp):
  ENTRY(16)
  /* a2 = s1, a3 = s2, a4 = n */

  /* Return 0 if n == 0 */

  beqz  a4, .Lret_zero

  /* Byte compare until s1 is word-aligned */

  bbci.l  a2, 0, .Lalign2
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  addi  a3, a3, 1
  addi  a4, a4, -1
  bne a8, a9, .Ldiff_byte
  beqz  a4, .Lret_zero
.Lalign2:
  bbci.l  a2, 1, .Laligned
  beqz  a4, .Lret_zero
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  addi  a3, a3, 1
  addi  a4, a4, -1
  bne a8, a9, .Ldiff_byte
  beqz  a4, .Lret_zero
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  addi  a3, a3, 1
  addi  a4, a4, -1
  bne a8, a9, .Ldiff_byte
  beqz  a4, .Lret_zero

  /* s1 is word-aligned. Check if s2 is also word-aligned. */

.Laligned:
  movi  a5, MASK0
  movi  a6, MASK1
  movi  a7, MASK2

  /* Check s2 alignment */

  bbsi.l  a3, 0, .Lunaligned
  bbsi.l  a3, 1, .Lunaligned

  /* Both s1 and s2 are word-aligned. Word-at-a-time compare. */

.Laligned_loop:
  bltui a4, 4, .Ltail
#if XCHAL_HAVE_LOOPS
  srli  a8, a4, 2       /* a8 = n / 4 = loop count */
  loopnez a8, .Laligned_loop_end
#endif
.Laligned_loop_body:
  l32i  a8, a2, 0
  l32i  a9, a3, 0
  addi  a2, a2, 4
  addi  a3, a3, 4
  addi  a4, a4, -4
  bne a8, a9, .Lfind_diff
#if XCHAL_HAVE_LOOPS
.Laligned_loop_end:
  j .Ltail
#else
  bgeui a4, 4, .Laligned_loop_body
  j .Ltail
#endif

  /* s1 aligned, s2 unaligned. Use funnel shift for s2. */

.Lunaligned:
  bltui a4, 4, .Ltail

  /* Set up funnel shift */

  ssa8  a3
  movi  a10, ~3
  and a3, a3, a10   /* align s2 pointer down */
  l32i  a11, a3, 0  /* load first word of s2 */

.Lunaligned_loop:
  bltui a4, 4, .Ltail_unaligned
#if XCHAL_HAVE_LOOPS
  srli  a8, a4, 2
  loopnez a8, .Lunaligned_loop_end
#endif
.Lunaligned_loop_body:
  l32i  a9, a3, 4   /* load next aligned word from s2 */
  src_b a10, a11, a9 /* funnel shift to get unaligned word */
  l32i  a8, a2, 0   /* load aligned word from s1 */
  addi  a2, a2, 4
  addi  a3, a3, 4
  addi  a4, a4, -4
  mov a11, a9        /* save for next iteration */
  bne a8, a10, .Lfind_diff_ua
#if XCHAL_HAVE_LOOPS
.Lunaligned_loop_end:
#else
  bgeui a4, 4, .Lunaligned_loop_body
#endif

.Ltail_unaligned:
  /* Restore s2 byte pointer from aligned pointer + SAR offset */

  rsr a9, SAR
  srli  a9, a9, 3   /* offset = SAR / 8 */
  add a3, a3, a9    /* a3 = real unaligned s2 position */

.Ltail:
  /* Byte-by-byte compare for remaining n bytes */

  beqz  a4, .Lret_zero
.Ltail_loop:
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  addi  a3, a3, 1
  bne a8, a9, .Ldiff_byte
  addi  a4, a4, -1
  bnez  a4, .Ltail_loop

.Lret_zero:
  movi  a2, 0
  RET(16)

  /* Found difference in word compare (both aligned) */

.Lfind_diff:
  /* a8 = w1, a9 = w2, they differ. Find first different byte. */

  xor a10, a8, a9
  bany  a10, a5, .Ldiff0  /* byte0 differs */
  bany  a10, a6, .Ldiff1  /* byte1 differs */
  bany  a10, a7, .Ldiff2  /* byte2 differs */
  /* byte3 must differ */
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

  /* Found difference in unaligned path */

.Lfind_diff_ua:
  /* a8 = w1 (from s1), a10 = w2 (funnel shifted from s2) */

  mov a9, a10
  xor a10, a8, a9
  bany  a10, a5, .Ldiff0
  bany  a10, a6, .Ldiff1
  bany  a10, a7, .Ldiff2
#if XCHAL_HAVE_BE
  extui a8, a8, 0, 8
  extui a9, a9, 0, 8
#else
  extui a8, a8, 24, 8
  extui a9, a9, 24, 8
#endif
  sub a2, a8, a9
  RET(16)

  /* Byte difference return */

.Ldiff_byte:
  sub a2, a8, a9
  RET(16)

  .end schedule

  .size ARCH_LIBCFUN(memcmp), . - ARCH_LIBCFUN(memcmp)

#endif
