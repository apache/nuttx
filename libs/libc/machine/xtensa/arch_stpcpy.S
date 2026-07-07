/****************************************************************************
 * libs/libc/machine/xtensa/arch_stpcpy.S
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

#ifdef LIBC_BUILD_STPCPY

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(stpcpy)
  .type ARCH_LIBCFUN(stpcpy), @function
ARCH_LIBCFUN(stpcpy):
  ENTRY(16)
  /* a2 = dst, a3 = src */
  /* Returns pointer to the null terminator written in dst */

  mov a10, a2           /* a10 = dst working pointer */
  movi  a4, MASK0
  movi  a5, MASK1
  movi  a6, MASK2
  movi  a7, MASK3

  /* Align src to word boundary with byte copies */

  bbsi.l  a3, 0, .Lsrc1mod2
  bbsi.l  a3, 1, .Lsrc2mod4
  j .Lsrcaligned

.Lsrc1mod2:
  l8ui  a8, a3, 0
  addi  a3, a3, 1
  s8i a8, a10, 0
  beqz  a8, .Lret_null_pos
  addi  a10, a10, 1
  bbci.l  a3, 1, .Lsrcaligned

.Lsrc2mod4:
  l8ui  a8, a3, 0
  s8i a8, a10, 0
  beqz  a8, .Lret_null_pos
  l8ui  a8, a3, 1
  addi  a3, a3, 2
  s8i a8, a10, 1
  addi  a10, a10, 2
  beqz  a8, .Lret_null_pos_minus1

.Lsrcaligned:
  /* src is word-aligned. Check if dst is also word-aligned. */

  movi  a9, 3
  bnone a10, a9, .Lboth_aligned

  /* src aligned, dst NOT aligned. Word-read src, byte-write dst. */

.Lsrc_aligned_only:
  l32i  a8, a3, 0
  addi  a3, a3, 4

  /* Check and store byte0 */

#if XCHAL_HAVE_BE
  extui a9, a8, 24, 8
#else
  extui a9, a8, 0, 8
#endif
  s8i a9, a10, 0
  beqz  a9, .Lret_null_pos

  /* byte1 */

#if XCHAL_HAVE_BE
  extui a9, a8, 16, 8
#else
  extui a9, a8, 8, 8
#endif
  s8i a9, a10, 1
  beqz  a9, .Lret_null_pos_plus1

  /* byte2 */

#if XCHAL_HAVE_BE
  extui a9, a8, 8, 8
#else
  extui a9, a8, 16, 8
#endif
  s8i a9, a10, 2
  beqz  a9, .Lret_null_pos_plus2

  /* byte3 */

#if XCHAL_HAVE_BE
  extui a9, a8, 0, 8
#else
  extui a9, a8, 24, 8
#endif
  s8i a9, a10, 3
  addi  a10, a10, 4
  beqz  a9, .Lret_null_pos_minus1
  j .Lsrc_aligned_only

  /* Both src and dst are word-aligned. Fast path. */

.Lboth_aligned:
#if XCHAL_HAVE_LOOPS
  _movi.n a8, 0
  loop  a8, .Lloop_end
#endif
.Lloop:
  l32i  a8, a3, 0
  addi  a3, a3, 4

  bnone a8, a4, .Lfound0
  bnone a8, a5, .Lfound1
  bnone a8, a6, .Lfound2
  bnone a8, a7, .Lfound3

  s32i  a8, a10, 0
  addi  a10, a10, 4
#if XCHAL_HAVE_LOOPS
.Lloop_end:
  j .Lloop
#else
  j .Lloop
#endif

  /* Found null at byte0 of word */

.Lfound0:
  s8i a8, a10, 0
  mov a2, a10
  RET(16)

  /* Found null at byte1 */

.Lfound1:
#if XCHAL_HAVE_BE
  extui a9, a8, 24, 8
  s8i a9, a10, 0
  s8i a8, a10, 1
#else
  s8i a8, a10, 0
  extui a9, a8, 8, 8
  s8i a9, a10, 1
#endif
  addi  a2, a10, 1
  RET(16)

  /* Found null at byte2 */

.Lfound2:
#if XCHAL_HAVE_BE
  extui a9, a8, 24, 8
  s8i a9, a10, 0
  extui a9, a8, 16, 8
  s8i a9, a10, 1
  extui a9, a8, 8, 8
  s8i a9, a10, 2
#else
  s8i a8, a10, 0
  extui a9, a8, 8, 8
  s8i a9, a10, 1
  extui a9, a8, 16, 8
  s8i a9, a10, 2
#endif
  addi  a2, a10, 2
  RET(16)

  /* Found null at byte3 - all 4 bytes valid, store whole word */

.Lfound3:
  s32i  a8, a10, 0
  addi  a2, a10, 3
  RET(16)

  /* Return: a10 points to the null byte written */

.Lret_null_pos:
  mov a2, a10
  RET(16)

.Lret_null_pos_plus1:
  addi  a2, a10, 1
  RET(16)

.Lret_null_pos_plus2:
  addi  a2, a10, 2
  RET(16)

.Lret_null_pos_minus1:
  addi  a2, a10, -1
  RET(16)

  .end schedule

  .size ARCH_LIBCFUN(stpcpy), . - ARCH_LIBCFUN(stpcpy)

#endif
