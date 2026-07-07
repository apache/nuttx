/****************************************************************************
 * libs/libc/machine/xtensa/arch_strrchr.S
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

#ifdef LIBC_BUILD_STRRCHR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(strrchr)
  .type ARCH_LIBCFUN(strrchr), @function
ARCH_LIBCFUN(strrchr):
  ENTRY(16)
  /* a2 = s, a3 = c */

  extui a3, a3, 0, 8   /* c &= 0xff */
  movi  a10, 0         /* a10 = last_found = NULL */

  /* Broadcast c to 4 bytes: a5 = pattern */

  slli  a5, a3, 8
  or  a5, a3, a5
  slli  a8, a5, 16
  or  a5, a5, a8        /* a5 = pattern = c * 0x01010101 */

  /* Byte-by-byte until s is word-aligned */

  bbci.l  a2, 0, .Lcheck2
  l8ui  a8, a2, 0
  beq a8, a3, .Lupdate0
.Lafter0:
  beqz  a8, .Lret_last
  addi  a2, a2, 1
.Lcheck2:
  bbci.l  a2, 1, .Lword_setup
  l8ui  a8, a2, 0
  beq a8, a3, .Lupdate1
.Lafter1:
  beqz  a8, .Lret_last
  addi  a2, a2, 1
  l8ui  a8, a2, 0
  beq a8, a3, .Lupdate2
.Lafter2:
  beqz  a8, .Lret_last
  addi  a2, a2, 1

  /* s is word-aligned. Set up MASKs. */

.Lword_setup:
  movi  a6, MASK0
  movi  a7, MASK1
  movi  a8, MASK2
  movi  a9, MASK3

  /* Word-at-a-time loop: scan forward, update last_found on match,
     stop at null. Order: check match first, then check null. */

.Lword_loop:
  l32i  a11, a2, 0
  xor a4, a11, a5       /* xored: matching bytes become 0 */

  /* byte0: check match, then null */

  bnone a4, a6, .Lmatch0
.Lm0_done:
  bnone a11, a6, .Lret_last

  /* byte1 */

  bnone a4, a7, .Lmatch1
.Lm1_done:
  bnone a11, a7, .Lret_last

  /* byte2 */

  bnone a4, a8, .Lmatch2
.Lm2_done:
  bnone a11, a8, .Lret_last

  /* byte3 */

  bnone a4, a9, .Lmatch3
.Lm3_done:
  bnone a11, a9, .Lret_last

  addi  a2, a2, 4
  j .Lword_loop

  /* Match handlers: update last_found and continue checking */

.Lmatch0:
  mov a10, a2
  j .Lm0_done

.Lmatch1:
  addi  a10, a2, 1
  j .Lm1_done

.Lmatch2:
  addi  a10, a2, 2
  j .Lm2_done

.Lmatch3:
  addi  a10, a2, 3
  j .Lm3_done

  /* Byte loop match handlers */

.Lupdate0:
  mov a10, a2
  j .Lafter0

.Lupdate1:
  mov a10, a2
  j .Lafter1

.Lupdate2:
  mov a10, a2
  j .Lafter2

  /* Return last found position (or NULL) */

.Lret_last:
  mov a2, a10
  RET(16)

  .end schedule

  .size ARCH_LIBCFUN(strrchr), . - ARCH_LIBCFUN(strrchr)

#endif
