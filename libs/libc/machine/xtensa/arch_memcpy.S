/****************************************************************************
 * libs/libc/machine/xtensa/arch_memcpy.S
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
 * Pre-processor Macros
 ****************************************************************************/

/* set to 1 when running on ISS (simulator) with the
   lint or ferret client, or 0 to save a few cycles */

#define SIM_CHECKS_ALIGNMENT  0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .literal_position

  .local  .Ldst1mod2
  .local  .Ldst2mod4
  .local  .Lbytecopy

  .align  4
  .global memcpy
  .type memcpy, @function
memcpy:
  ENTRY(16)
  /* a2 = dst, a3 = src, a4 = len */

  mov a5, a2    # copy dst so that a2 is return value
  bbsi.l  a2, 0, .Ldst1mod2
  bbsi.l  a2, 1, .Ldst2mod4
.Ldstaligned:

  /* Get number of loop iterations with 16B per iteration.  */
  srli  a7, a4, 4

  /* Check if source is aligned.  */
  slli  a8, a3, 30
  bnez  a8, .Lsrcunaligned

  /* Destination and source are word-aligned, use word copy.  */
#if XCHAL_HAVE_LOOPS
  loopnez a7, 2f
#else
  beqz  a7, 2f
  slli  a8, a7, 4
  add a8, a8, a3  # a8 = end of last 16B source chunk
#endif
1:  l32i  a6, a3, 0
  l32i  a7, a3, 4
  s32i  a6, a5, 0
  l32i  a6, a3, 8

  s32i  a7, a5, 4
  l32i  a7, a3, 12
  s32i  a6, a5, 8
  addi  a3, a3, 16
  s32i  a7, a5, 12
  addi  a5, a5, 16
#if !XCHAL_HAVE_LOOPS
  bltu  a3, a8, 1b
#endif

  /* Copy any leftover pieces smaller than 16B.  */
2:  bbci.l  a4, 3, 3f

  /* Copy 8 bytes.  */
  l32i  a6, a3, 0
  l32i  a7, a3, 4
  addi  a3, a3, 8
  s32i  a6, a5, 0
  s32i  a7, a5, 4
  addi  a5, a5, 8

3:  bbsi.l  a4, 2, 4f
  bbsi.l  a4, 1, 5f
  bbsi.l  a4, 0, 6f
  RET(16)

  # .align 4
  /* Copy 4 bytes.  */
4:  l32i  a6, a3, 0
  addi  a3, a3, 4
  s32i  a6, a5, 0
  addi  a5, a5, 4
  bbsi.l  a4, 1, 5f
  bbsi.l  a4, 0, 6f
  RET(16)

  /* Copy 2 bytes.  */
5:  l16ui a6, a3, 0
  addi  a3, a3, 2
  s16i  a6, a5, 0
  addi  a5, a5, 2
  bbsi.l  a4, 0, 6f
  RET(16)

  /* Copy 1 byte.  */
6:  l8ui  a6, a3, 0
  s8i a6, a5, 0

.Ldone:
  RET(16)


/* Destination is aligned; source is unaligned.  */

  # .align 4
.Lsrcunaligned:
  /* Avoid loading anything for zero-length copies.  */
  beqz  a4, .Ldone

  /* Copy 16 bytes per iteration for word-aligned dst and
     unaligned src.  */
  ssa8  a3    # set shift amount from byte offset
#if XCHAL_UNALIGNED_LOAD_EXCEPTION || SIM_CHECKS_ALIGNMENT
  srli    a11, a8, 30     # save unalignment offset for below
  sub a3, a3, a11 # align a3
#endif
  l32i  a6, a3, 0 # load first word
#if XCHAL_HAVE_LOOPS
  loopnez a7, 2f
#else
  beqz  a7, 2f
  slli  a10, a7, 4
  add a10, a10, a3  # a10 = end of last 16B source chunk
#endif
1:  l32i  a7, a3, 4
  l32i  a8, a3, 8
  src_b a6, a6, a7
  s32i  a6, a5, 0
  l32i  a9, a3, 12
  src_b a7, a7, a8
  s32i  a7, a5, 4
  l32i  a6, a3, 16
  src_b a8, a8, a9
  s32i  a8, a5, 8
  addi  a3, a3, 16
  src_b a9, a9, a6
  s32i  a9, a5, 12
  addi  a5, a5, 16
#if !XCHAL_HAVE_LOOPS
  bltu  a3, a10, 1b
#endif

2:  bbci.l  a4, 3, 3f

  /* Copy 8 bytes.  */
  l32i  a7, a3, 4
  l32i  a8, a3, 8
  src_b a6, a6, a7
  s32i  a6, a5, 0
  addi  a3, a3, 8
  src_b a7, a7, a8
  s32i  a7, a5, 4
  addi  a5, a5, 8
  mov a6, a8

3:  bbci.l  a4, 2, 4f

  /* Copy 4 bytes.  */
  l32i  a7, a3, 4
  addi  a3, a3, 4
  src_b a6, a6, a7
  s32i  a6, a5, 0
  addi  a5, a5, 4
  mov a6, a7
4:
#if XCHAL_UNALIGNED_LOAD_EXCEPTION || SIM_CHECKS_ALIGNMENT
  add a3, a3, a11 # readjust a3 with correct misalignment
#endif
  bbsi.l  a4, 1, 5f
  bbsi.l  a4, 0, 6f
  RET(16)

  /* Copy 2 bytes.  */
5:  l8ui  a6, a3, 0
  l8ui  a7, a3, 1
  addi  a3, a3, 2
  s8i a6, a5, 0
  s8i a7, a5, 1
  addi  a5, a5, 2
  bbsi.l  a4, 0, 6f
  RET(16)

  /* Copy 1 byte.  */
6:  l8ui  a6, a3, 0
  s8i a6, a5, 0
  RET(16)


  # .align XCHAL_INST_FETCH_WIDTH
__memcpy_aux:

  /* Skip bytes to get proper alignment for three-byte loop */
# .skip XCHAL_INST_FETCH_WIDTH - 3

.Lbytecopy:
#if XCHAL_HAVE_LOOPS
  loopnez a4, 2f
#else
  beqz  a4, 2f
  add a7, a3, a4  # a7 = end address for source
#endif
1:  l8ui  a6, a3, 0
  addi  a3, a3, 1
  s8i a6, a5, 0
  addi  a5, a5, 1
#if !XCHAL_HAVE_LOOPS
  bltu  a3, a7, 1b
#endif
2:  RET(16)


/* Destination is unaligned.  */

  # .align 4
.Ldst1mod2: # dst is only byte aligned

  /* Do short copies byte-by-byte.  */
  bltui a4, 7, .Lbytecopy

  /* Copy 1 byte.  */
  l8ui  a6, a3, 0
  addi  a3, a3, 1
  addi  a4, a4, -1
  s8i a6, a5, 0
  addi  a5, a5, 1

  /* Return to main algorithm if dst is now aligned.  */
  bbci.l  a5, 1, .Ldstaligned

.Ldst2mod4: # dst has 16-bit alignment

  /* Do short copies byte-by-byte.  */
  bltui a4, 6, .Lbytecopy

  /* Copy 2 bytes.  */
  l8ui  a6, a3, 0
  l8ui  a7, a3, 1
  addi  a3, a3, 2
  addi  a4, a4, -2
  s8i a6, a5, 0
  s8i a7, a5, 1
  addi  a5, a5, 2

  /* dst is now aligned; return to main algorithm.  */
  j .Ldstaligned

  .end schedule

  .size memcpy, . - memcpy
