/****************************************************************************
 * libs/libc/machine/xtensa/arch_strchr.S
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

#ifdef USE_AS_STRCHRNUL
#  ifdef LIBC_BUILD_STRCHRNUL
#    define STRCHR_FUNC strchrnul
#  else
#    undef USE_AS_STRCHRNUL
#  endif
#endif

#ifndef USE_AS_STRCHRNUL
#  ifdef LIBC_BUILD_STRCHR
#    define STRCHR_FUNC strchr
#  else
     /* Neither guard is set, skip compilation */
#    define SKIP_COMPILATION
#  endif
#endif

#ifndef SKIP_COMPILATION

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(STRCHR_FUNC)
  .type ARCH_LIBCFUN(STRCHR_FUNC), @function
ARCH_LIBCFUN(STRCHR_FUNC):
  ENTRY(16)
  /* a2 = s, a3 = c */

  /* Broadcast c to 4 bytes: a5 = pattern */

  extui a5, a3, 0, 8   /* c &= 0xff */
  slli  a8, a5, 8
  or  a5, a5, a8        /* 0xcc -> 0xcccc */
  slli  a8, a5, 16
  or  a5, a5, a8        /* 0xcccc -> 0xcccccccc */

  /* Also keep single byte c in a3 for byte comparisons */

  extui a3, a3, 0, 8

  /* Byte-by-byte until s is word-aligned */

  bbci.l  a2, 0, .Lcheck_align2
  l8ui  a8, a2, 0
  beq a8, a3, .Lret_found
  beqz  a8, .Lret_null_or_pos
  addi  a2, a2, 1
.Lcheck_align2:
  bbci.l  a2, 1, .Lword_setup
  l8ui  a8, a2, 0
  beq a8, a3, .Lret_found
  beqz  a8, .Lret_null_or_pos
  addi  a2, a2, 1
  l8ui  a8, a2, 0
  beq a8, a3, .Lret_found
  beqz  a8, .Lret_null_or_pos
  addi  a2, a2, 1

  /* s is now word-aligned. Set up MASKs. */

.Lword_setup:
  movi  a6, MASK0
  movi  a7, MASK1
  movi  a8, MASK2
  movi  a9, MASK3

  /* Word-at-a-time main loop.
     For each word, check both for null and for matching char.
     The byte order determines which comes first. */

#if XCHAL_HAVE_LOOPS
  _movi.n a10, 0
  loop  a10, .Lloop_end   /* infinite loop */
#endif
.Lloop:
  l32i  a10, a2, 0
  xor a11, a10, a5      /* xored: matching bytes become 0 */

  /* Check byte0 */

  bnone a10, a6, .Lnull0  /* byte0 of word is null */
  bnone a11, a6, .Lfound0 /* byte0 matches c */

  /* Check byte1 */

  bnone a10, a7, .Lnull1
  bnone a11, a7, .Lfound1

  /* Check byte2 */

  bnone a10, a8, .Lnull2
  bnone a11, a8, .Lfound2

  /* Check byte3 */

  bnone a10, a9, .Lnull3
  bnone a11, a9, .Lfound3

  addi  a2, a2, 4
#if XCHAL_HAVE_LOOPS
.Lloop_end:
  j .Lloop
#else
  j .Lloop
#endif

  /* Found matching character */

.Lfound0:
  RET(16)

.Lfound1:
  addi  a2, a2, 1
  RET(16)

.Lfound2:
  addi  a2, a2, 2
  RET(16)

.Lfound3:
  addi  a2, a2, 3
  RET(16)

  /* Found null terminator before c */

.Lnull0:
  /* Check if c == '\0' (i.e., null IS the char we're looking for) */
  bnone a11, a6, .Lfound0
#ifdef USE_AS_STRCHRNUL
  /* strchrnul: return pointer to the null byte */
  RET(16)
#else
  movi  a2, 0
  RET(16)
#endif

.Lnull1:
  bnone a11, a7, .Lfound1
#ifdef USE_AS_STRCHRNUL
  addi  a2, a2, 1
  RET(16)
#else
  movi  a2, 0
  RET(16)
#endif

.Lnull2:
  bnone a11, a8, .Lfound2
#ifdef USE_AS_STRCHRNUL
  addi  a2, a2, 2
  RET(16)
#else
  movi  a2, 0
  RET(16)
#endif

.Lnull3:
  bnone a11, a9, .Lfound3
#ifdef USE_AS_STRCHRNUL
  addi  a2, a2, 3
  RET(16)
#else
  movi  a2, 0
  RET(16)
#endif

  /* Return found (byte loop) */

.Lret_found:
  RET(16)

  /* Return null or position (byte loop hit null) */

.Lret_null_or_pos:
#ifdef USE_AS_STRCHRNUL
  /* strchrnul returns pointer to null */
  RET(16)
#else
  movi  a2, 0
  RET(16)
#endif

  .end schedule

  .size ARCH_LIBCFUN(STRCHR_FUNC), . - ARCH_LIBCFUN(STRCHR_FUNC)

#endif /* !SKIP_COMPILATION */
