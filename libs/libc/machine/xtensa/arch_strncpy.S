/****************************************************************************
 * libs/libc/machine/xtensa/arch_strncpy.S
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
__strncpy_aux:

.Lsrc1mod2: # src address is odd
  l8ui  a8, a3, 0 # get byte 0
  addi  a3, a3, 1 # advance src pointer
  s8i a8, a10, 0  # store byte 0
  addi  a4, a4, -1  # decrement n
  beqz    a4, .Lret       # if n is zero
  addi  a10, a10, 1 # advance dst pointer
  beqz  a8, .Lfill  # if byte 0 is zero
  bbci.l  a3, 1, .Lsrcaligned # if src is now word-aligned

.Lsrc2mod4: # src address is 2 mod 4
  l8ui  a8, a3, 0 # get byte 0
  addi  a4, a4, -1  # decrement n
  s8i a8, a10, 0  # store byte 0
  beqz    a4, .Lret       # if n is zero
  addi  a10, a10, 1 # advance dst pointer
  beqz  a8, .Lfill  # if byte 0 is zero
  l8ui  a8, a3, 1 # get byte 0
  addi  a3, a3, 2 # advance src pointer
  s8i a8, a10, 0  # store byte 0
  addi  a4, a4, -1  # decrement n
  beqz    a4, .Lret       # if n is zero
  addi  a10, a10, 1 # advance dst pointer
  bnez  a8, .Lsrcaligned
  j .Lfill

.Lret:
  RET(16)

  .align  4
  .global strncpy
  .type strncpy, @function
strncpy:
  ENTRY(16)
  /* a2 = dst, a3 = src */

  mov a10, a2   # leave dst in return value register
  beqz    a4, .Lret       # if n is zero

  movi  a11, MASK0
  movi  a5, MASK1
  movi  a6, MASK2
  movi  a7, MASK3
  bbsi.l  a3, 0, .Lsrc1mod2
  bbsi.l  a3, 1, .Lsrc2mod4
.Lsrcaligned:

  /* Check if the destination is aligned.  */
  movi  a8, 3
  bnone a10, a8, .Laligned

  j .Ldstunaligned

/* Fill the dst with zeros -- n is at least 1.  */

.Lfill:
  movi  a9, 0
  bbsi.l  a10, 0, .Lfill1mod2
  bbsi.l  a10, 1, .Lfill2mod4
.Lfillaligned:
  blti  a4, 4, .Lfillcleanup

  /* Loop filling complete words with zero.  */
#if XCHAL_HAVE_LOOPS

  srai  a8, a4, 2
  loop  a8, 1f
  s32i  a9, a10, 0
  addi  a10, a10, 4

1:  slli  a8, a8, 2
  sub a4, a4, a8

#else /* !XCHAL_HAVE_LOOPS */

1:  s32i  a9, a10, 0
  addi  a10, a10, 4
  addi  a4, a4, -4
  bgei    a4, 4, 1b

#endif /* !XCHAL_HAVE_LOOPS */

  beqz  a4, 2f

.Lfillcleanup:
  /* Fill leftover (1 to 3) bytes with zero.  */
  s8i a9, a10, 0  # store byte 0
  addi  a4, a4, -1  # decrement n
  addi  a10, a10, 1
  bnez    a4, .Lfillcleanup

2:  RET(16)

.Lfill1mod2: # dst address is odd
  s8i a9, a10, 0  # store byte 0
  addi  a4, a4, -1  # decrement n
  beqz    a4, 2b    # if n is zero
  addi    a10, a10, 1 # advance dst pointer
  bbci.l  a10, 1, .Lfillaligned # if dst is now word-aligned

.Lfill2mod4: # dst address is 2 mod 4
  s8i a9, a10, 0  # store byte 0
  addi  a4, a4, -1  # decrement n
  beqz    a4, 2b    # if n is zero
  s8i a9, a10, 1  # store byte 1
  addi  a4, a4, -1  # decrement n
  beqz    a4, 2b    # if n is zero
  addi    a10, a10, 2 # advance dst pointer
  j .Lfillaligned

/* dst is word-aligned; src is word-aligned; n is at least 1.  */

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
  loop  a8, 1f    # loop forever (almost anyway)
  blti  a4, 5, .Ldstunaligned # n is near limit; do one at a time
  l32i  a8, a3, 0 # get word from src
  addi  a3, a3, 4 # advance src pointer
  bnone a8, a11, .Lz0 # if byte 0 is zero
  bnone a8, a5, .Lz1  # if byte 1 is zero
  bnone a8, a6, .Lz2  # if byte 2 is zero
  s32i  a8, a10, 0  # store word to dst
  addi  a4, a4, -4  # decrement n
  addi  a10, a10, 4 # advance dst pointer
  bnone a8, a7, .Lfill  # if byte 3 is zero
1:

#else /* !XCHAL_HAVE_LOOPS */

1:  blti  a4, 5, .Ldstunaligned # n is near limit; do one at a time
  l32i  a8, a3, 0 # get word from src
  addi  a3, a3, 4 # advance src pointer
  bnone a8, a11, .Lz0 # if byte 0 is zero
  bnone a8, a5, .Lz1  # if byte 1 is zero
  bnone a8, a6, .Lz2  # if byte 2 is zero
  s32i  a8, a10, 0  # store word to dst
  addi  a4, a4, -4  # decrement n
  addi  a10, a10, 4 # advance dst pointer
  bany  a8, a7, 1b  # no zeroes
#endif /* !XCHAL_HAVE_LOOPS */

  j .Lfill

.Lz0: /* Byte 0 is zero.  */
#if XCHAL_HAVE_BE
  movi  a8, 0
#endif
  s8i a8, a10, 0
  addi  a4, a4, -1  # decrement n
  addi  a10, a10, 1 # advance dst pointer
  j .Lfill

.Lz1: /* Byte 1 is zero.  */
#if XCHAL_HAVE_BE
        extui   a8, a8, 16, 16
#endif
  s16i  a8, a10, 0
  addi  a4, a4, -2  # decrement n
  addi  a10, a10, 2 # advance dst pointer
  j .Lfill

.Lz2: /* Byte 2 is zero.  */
#if XCHAL_HAVE_BE
  extui   a8, a8, 16, 16
#endif
  s16i  a8, a10, 0
  movi  a8, 0
  s8i a8, a10, 2
  addi  a4, a4, -3  # decrement n
  addi  a10, a10, 3 # advance dst pointer
  j .Lfill

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
.Ldstunaligned:

#if XCHAL_HAVE_LOOPS
#if XCHAL_HAVE_DENSITY
  _movi.n a8, 0   # set up for the maximum loop count
#else
  _movi a8, 0   # set up for the maximum loop count
#endif
  loop  a8, 2f    # loop forever (almost anyway)
#endif
1:  l8ui  a8, a3, 0
  addi  a3, a3, 1
  s8i a8, a10, 0
  addi  a4, a4, -1
  beqz  a4, 3f
  addi  a10, a10, 1
#if XCHAL_HAVE_LOOPS
  beqz  a8, 2f
#else
  bnez  a8, 1b
#endif
2:  j .Lfill

3:  RET(16)
.end schedule

  .size strncpy, . - strncpy
