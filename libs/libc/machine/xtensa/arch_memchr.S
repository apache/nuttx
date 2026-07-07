/****************************************************************************
 * libs/libc/machine/xtensa/arch_memchr.S
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

#ifdef LIBC_BUILD_MEMCHR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(memchr)
  .type ARCH_LIBCFUN(memchr), @function
ARCH_LIBCFUN(memchr):
  ENTRY(16)
  /* a2 = s, a3 = c, a4 = n */

  /* Return NULL if n == 0 */

  beqz  a4, .Lret_null

  /* Broadcast c to 4 bytes: a5 = pattern */

  extui a3, a3, 0, 8   /* c &= 0xff */
  slli  a5, a3, 8
  or  a5, a3, a5        /* 0x00cc -> 0xcccc */
  slli  a8, a5, 16
  or  a5, a5, a8        /* 0xcccc -> 0xcccccccc */

  /* Byte-by-byte until s is word-aligned */

  bbci.l  a2, 0, .Lcheck_align2
  l8ui  a8, a2, 0
  beq a8, a3, .Lret_found
  addi  a2, a2, 1
  addi  a4, a4, -1
  beqz  a4, .Lret_null
.Lcheck_align2:
  bbci.l  a2, 1, .Lword_setup
  beqz  a4, .Lret_null
  l8ui  a8, a2, 0
  beq a8, a3, .Lret_found
  addi  a2, a2, 1
  addi  a4, a4, -1
  beqz  a4, .Lret_null
  l8ui  a8, a2, 0
  beq a8, a3, .Lret_found
  addi  a2, a2, 1
  addi  a4, a4, -1
  beqz  a4, .Lret_null

  /* s is now word-aligned. Set up MASKs for word-at-a-time loop. */

.Lword_setup:
  movi  a6, MASK0
  movi  a7, MASK1
  movi  a8, MASK2
  movi  a9, MASK3

  /* Word-at-a-time main loop */

.Lword_loop:
  bltui a4, 4, .Ltail
#if XCHAL_HAVE_LOOPS
  srli  a10, a4, 2      /* a10 = n / 4 = loop count */
  loopnez a10, .Lword_loop_end
#endif
.Lword_loop_body:
  l32i  a10, a2, 0
  xor a11, a10, a5      /* XOR with pattern: matching bytes become 0 */
  bnone a11, a6, .Lfound0
  bnone a11, a7, .Lfound1
  bnone a11, a8, .Lfound2
  bnone a11, a9, .Lfound3
  addi  a2, a2, 4
  addi  a4, a4, -4
#if XCHAL_HAVE_LOOPS
.Lword_loop_end:
#else
  bgeui a4, 4, .Lword_loop_body
#endif

  /* Tail: byte-by-byte for remaining */

.Ltail:
  beqz  a4, .Lret_null
.Ltail_loop:
  l8ui  a10, a2, 0
  beq a10, a3, .Lret_found
  addi  a2, a2, 1
  addi  a4, a4, -1
  bnez  a4, .Ltail_loop

.Lret_null:
  movi  a2, 0
  RET(16)

.Lret_found:
  /* a2 already points to the found byte */
  RET(16)

.Lfound0:
  /* byte0 of word matches, s+0 */
  RET(16)

.Lfound1:
  /* byte1 of word matches, s+1 */
  addi  a2, a2, 1
  RET(16)

.Lfound2:
  /* byte2 of word matches, s+2 */
  addi  a2, a2, 2
  RET(16)

.Lfound3:
  /* byte3 of word matches, s+3 */
  addi  a2, a2, 3
  RET(16)

  .end schedule

  .size ARCH_LIBCFUN(memchr), . - ARCH_LIBCFUN(memchr)

#endif
