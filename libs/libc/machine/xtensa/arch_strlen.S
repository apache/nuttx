/****************************************************************************
 * libs/libc/machine/xtensa/arch_strlen.S
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global strlen
  .type strlen, @function
strlen:
  ENTRY(16)
  /* a2 = s */

  addi  a3, a2, -4  # because we overincrement at the end
  movi  a4, MASK0
  movi  a5, MASK1
  movi  a6, MASK2
  movi  a7, MASK3
  bbsi.l  a2, 0, .L1mod2
  bbsi.l  a2, 1, .L2mod4
  j .Laligned

.L1mod2: # address is odd
  l8ui  a8, a3, 4 # get byte 0
  addi  a3, a3, 1 # advance string pointer
  beqz  a8, .Lz3  # if byte 0 is zero
  bbci.l  a3, 1, .Laligned # if string pointer is now word-aligned

.L2mod4: # address is 2 mod 4
  addi  a3, a3, 2 # advance ptr for aligned access
  l32i  a8, a3, 0 # get word with first two bytes of string
  bnone a8, a6, .Lz2  # if byte 2 (of word, not string) is zero
  bany  a8, a7, .Laligned # if byte 3 (of word, not string) is nonzero

  /* Byte 3 is zero.  */
  addi  a3, a3, 3 # point to zero byte
  sub a2, a3, a2  # subtract to get length
  RET(16)

/* String is word-aligned.  */

  .align  4
#if XCHAL_HAVE_LOOPS
#if XCHAL_HAVE_DENSITY
  /* (2 mod 4) alignment for loop instruction */
#else
  /* (1 mod 4) alignment for loop instruction */
  .byte 0
  .byte 0
#endif
#endif
.Laligned:
#if XCHAL_HAVE_LOOPS
#if XCHAL_HAVE_DENSITY
  _movi.n a8, 0   # set up for the maximum loop count
#else
  _movi a8, 0   # set up for the maximum loop count
#endif
  loop  a8, .Lz3  # loop forever (almost anyway)
#endif
1:  l32i  a8, a3, 4 # get next word of string
  addi  a3, a3, 4 # advance string pointer
  bnone a8, a4, .Lz0  # if byte 0 is zero
  bnone a8, a5, .Lz1  # if byte 1 is zero
  bnone a8, a6, .Lz2  # if byte 2 is zero
#if XCHAL_HAVE_LOOPS
  bnone a8, a7, .Lz3  # if byte 3 is zero
#else
  bany  a8, a7, 1b  # repeat if byte 3 is non-zero
#endif

.Lz3: /* Byte 3 is zero.  */
  addi  a3, a3, 3 # point to zero byte
  /* Fall through....  */

.Lz0: /* Byte 0 is zero.  */
  sub a2, a3, a2  # subtract to get length
  RET(16)

.Lz1: /* Byte 1 is zero.  */
  addi  a3, a3, 1 # point to zero byte
  sub a2, a3, a2  # subtract to get length
  RET(16)

.Lz2: /* Byte 2 is zero.  */
  addi  a3, a3, 2 # point to zero byte
  sub a2, a3, a2  # subtract to get length
  RET(16)

  .end schedule

  .size strlen, . - strlen
