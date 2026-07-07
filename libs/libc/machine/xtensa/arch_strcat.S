/****************************************************************************
 * libs/libc/machine/xtensa/arch_strcat.S
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

#ifdef LIBC_BUILD_STRCAT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(strcat)
  .type ARCH_LIBCFUN(strcat), @function
ARCH_LIBCFUN(strcat):
  ENTRY(16)
  /* a2 = dst, a3 = src */

  /* Save dst for return value */

  mov a10, a2

  /* --- Phase 1: Find end of dst (inline strlen) --- */

  /* Byte-by-byte until dst is word-aligned */

  bbci.l  a2, 0, .Lstrlen_check2
  l8ui  a8, a2, 0
  beqz  a8, .Lstrlen_done
  addi  a2, a2, 1
.Lstrlen_check2:
  bbci.l  a2, 1, .Lstrlen_word_setup
  l8ui  a8, a2, 0
  beqz  a8, .Lstrlen_done
  addi  a2, a2, 1
  l8ui  a8, a2, 0
  beqz  a8, .Lstrlen_done
  addi  a2, a2, 1

.Lstrlen_word_setup:
  movi  a4, MASK0
  movi  a5, MASK1
  movi  a6, MASK2
  movi  a7, MASK3

.Lstrlen_loop:
  l32i  a8, a2, 0
  bnone a8, a4, .Lstrlen_done
  bnone a8, a5, .Lstrlen_z1
  bnone a8, a6, .Lstrlen_z2
  bnone a8, a7, .Lstrlen_z3
  addi  a2, a2, 4
  j .Lstrlen_loop

.Lstrlen_z1:
  addi  a2, a2, 1
  j .Lstrlen_done
.Lstrlen_z2:
  addi  a2, a2, 2
  j .Lstrlen_done
.Lstrlen_z3:
  addi  a2, a2, 3

.Lstrlen_done:
  /* a2 now points to the null terminator of dst */

  /* --- Phase 2: Copy src to end of dst --- */
  /* Word-read src, byte-write dst (dst may not be aligned) */

  mov a11, a2           /* a11 = dst write pointer */

  /* Align src first */

  bbsi.l  a3, 0, .Lcpy_src1mod2
  bbsi.l  a3, 1, .Lcpy_src2mod4
  j .Lcpy_srcaligned

.Lcpy_src1mod2:
  l8ui  a8, a3, 0
  addi  a3, a3, 1
  s8i a8, a11, 0
  beqz  a8, .Lret
  addi  a11, a11, 1
  bbci.l  a3, 1, .Lcpy_srcaligned

.Lcpy_src2mod4:
  l8ui  a8, a3, 0
  s8i a8, a11, 0
  beqz  a8, .Lret
  l8ui  a8, a3, 1
  addi  a3, a3, 2
  s8i a8, a11, 1
  addi  a11, a11, 2
  beqz  a8, .Lret

.Lcpy_srcaligned:
  /* src is word-aligned. Word-read, byte-write. */

.Lcpy_word_loop:
  l32i  a8, a3, 0
  addi  a3, a3, 4

#if XCHAL_HAVE_BE
  extui a9, a8, 24, 8
#else
  extui a9, a8, 0, 8
#endif
  s8i a9, a11, 0
  beqz  a9, .Lret

#if XCHAL_HAVE_BE
  extui a9, a8, 16, 8
#else
  extui a9, a8, 8, 8
#endif
  s8i a9, a11, 1
  beqz  a9, .Lret

#if XCHAL_HAVE_BE
  extui a9, a8, 8, 8
#else
  extui a9, a8, 16, 8
#endif
  s8i a9, a11, 2
  beqz  a9, .Lret

#if XCHAL_HAVE_BE
  extui a9, a8, 0, 8
#else
  extui a9, a8, 24, 8
#endif
  s8i a9, a11, 3
  addi  a11, a11, 4
  beqz  a9, .Lret
  j .Lcpy_word_loop

.Lret:
  mov a2, a10   /* return original dst */
  RET(16)

  .end schedule

  .size ARCH_LIBCFUN(strcat), . - ARCH_LIBCFUN(strcat)

#endif
