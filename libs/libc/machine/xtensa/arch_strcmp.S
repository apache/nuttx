/****************************************************************************
 * libs/libc/machine/xtensa/arch_strcmp.S
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

#define MASK4 0x40404040

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin  schedule
  .align  4
  .literal_position

  .global strcmp
  .type strcmp,@function
  .align  4

strcmp:

#if XCHAL_HAVE_LOOPS && XCHAL_HAVE_DENSITY && !XCHAL_HAVE_BE && XCHAL_HAVE_FLIX3
/*  Fast version for FLIX3 Little Endian */

  ENTRY(16)
  /* a2 = s1, a3 = s2 */

  l8ui  a8, a2, 0 # byte 0 from s1
  l8ui  a9, a3, 0 # byte 0 from s2
  movi  a10, 3    # mask
  movi  a5, 0xfffffffc
  or  a11, a2, a3
  movi  a4, MASK0 # mask for byte 0
  movi  a7, MASK4
  addi    a3, a3, -8
  addi    a2, a2, -8
  and a5, a5, a2
  bne.w18 a8, a9, .Lretdiff
  l32i  a8, a5, 8 # get word from aligned variant of s1

  bany.w18  a11, a10, .Lnot_aligned

/* s1 is word-aligned; s2 is word-aligned.

   If the zero-overhead loop option is available, use an (almost)
   infinite zero-overhead loop with conditional exits so we only pay
   for taken branches when exiting the loop.  */

/* New algorithm, relying on the fact that all normal ASCII is between
   32 and 127.

   Rather than check all bytes for zero:
   Take one word (4 bytes).  Call it w1.
   Shift w1 left by one into w1'.
   Or w1 and w1'.  For all normal ASCII bit 6 will be 1; for zero it won't.
   Check that all 4 bit 6's (one for each byte) are one:
   If they are, we are definitely not done.
   If they are not, we are probably done, but need to check for zero.  */

.Laligned:
  /* Loop forever */
1:
  loop  a0, .Laligned_done

  /* First unrolled loop body.  */
  l32i  a9, a3, 8 # get word from s2
  addi  a3, a3, 8 # advance s2 pointer
  slli  a5, a8, 1
  or  a10, a8, a5
  {l32i a11, a2, 12 # get word from s1+4
  bne.w18 a8, a9, .Lwne2}
  l32i  a9, a3, 4 # get word from s2+4
  bnall.w18 a10, a7, .Lprobeq

  /* Second unrolled loop body.  */
  slli  a5, a11, 1
  or  a10, a11, a5
  addi  a2, a2, 8 # advance s1 pointer
        mov a8, a11
  bne.w18 a11, a9, .Lwne2
  l32i  a8, a2, 8 # get word from s1
  bnall.w18 a10, a7, .Lprobeq2

.Laligned_done:
  l32i  a8, a2, 8 # get word from s1
  j       1b

.Lnot_aligned:
  xor a11, a2, a3 # compare low two bits of s1 and s2
  bany  a11, a10, .Lunaligned # if they have different alignment

  /* s1/s2 are not word-aligned.  */
  movi  a5, 0xfffffffc
  addi  a2, a2, 1 # advance s1
  beqz  a9, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  and     a6, a2, a5
  l32i  a8, a6, 8 # get word from s1
  bnone a2, a10, .Laligned # if s1/s2 now aligned
  l8ui  a8, a2, 8 # byte 1 from s1
  l8ui  a9, a3, 8 # byte 1 from s2
  addi  a2, a2, 1 # advance s1
  bne a8, a9, .Lretdiff # if different, return difference
  beqz  a8, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  and     a6, a2, a5
  l32i  a8, a6, 8 # get word from s1
  bnone a2, a10, .Laligned # if s1/s2 now aligned
  l8ui  a8, a2, 8 # byte 2 from s1
  l8ui  a9, a3, 8 # byte 2 from s2
  addi  a2, a2, 1 # advance s1
  bne a8, a9, .Lretdiff # if different, return difference
  beqz  a8, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  l32i  a8, a2, 8 # get word from s1
  j .Laligned

/* s1 and s2 have different alignment.

   If the zero-overhead loop option is available, use an (almost)
   infinite zero-overhead loop with conditional exits so we only pay
   for taken branches when exiting the loop.

   Note: It is important for this unaligned case to come before the
   code for aligned strings, because otherwise some of the branches
   above cannot reach and have to be transformed to branches around
   jumps.  The unaligned code is smaller and the branches can reach
   over it.  */

.Lunaligned:
  movi.n  a8, 0   # set up for the maximum loop count
  loop  a8, .Lretdiff # loop forever (almost anyway)
  l8ui  a8, a2, 8
  l8ui  a9, a3, 8
  addi  a2, a2, 1
  bne a8, a9, .Lretdiff
  addi  a3, a3, 1
  beqz  a8, .Lretdiff
.Lretdiff:
  sub a2, a8, a9
  RET(16)

.Lprobeq2:
  /* Adjust pointers to account for the loop unrolling.  */
        mov a8, a11
  addi  a2, a2, -4
  addi  a3, a3, 4

  /* align (0 mod 4) */
.Lprobeq:
  /* Words are probably equal, but check for sure.
     If not, loop over the rest of string using normal algorithm.  */

  bnone a8, a4, .Leq  # if byte 0 is zero
  movi  a5, MASK1 # mask for byte 1
  movi  a6, MASK2 # mask for byte 2
  bnone a8, a5, .Leq  # if byte 1 is zero
  movi  a7, MASK3 # mask for byte 3
  bnone a8, a6, .Leq  # if byte 2 is zero
  bnone a8, a7, .Leq  # if byte 3 is zero
  /* align (1 mod 4) */
  addi.n  a2, a2, 12  # advance s1 pointer
  addi.n  a3, a3, 4 # advance s2 pointer
  /* align (1 mod 4) or (2 mod 4) */
1:
  loop  a0, .Lend # loop forever (a4 is bigger than max iters)

  l32i  a8, a2, 0 # get word from s1
  l32i  a9, a3, 0 # get word from s2
  addi  a2, a2, 4 # advance s1 pointer
  bne a8, a9, .Lwne
  bnone a8, a4, .Leq  # if byte 0 is zero
  bnone a8, a5, .Leq  # if byte 1 is zero
  bnone a8, a6, .Leq  # if byte 2 is zero
  bnone a8, a7, .Leq  # if byte 3 is zero
  addi  a3, a3, 4 # advance s2 pointer
.Lend:
  j 1b

  /* Words are equal; some byte is zero.  */
.Leq: movi  a2, 0   # return equal
  RET(16)

.Lwne2: /* Words are not equal.  On big-endian processors, if none of the
     bytes are zero, the return value can be determined by a simple
     comparison.  */
.Lwne:  /* Words are not equal.  */
  xor a2, a8, a9  # get word with nonzero in byte that differs
  extui a10, a8, 0, 8
  extui a11, a9, 0, 8
  movi  a5, MASK1 # mask for byte 1
  bany.w18  a2, a4, .Ldiff0 # if byte 0 differs

  bnone.w18 a8, a4, .Leq  # if byte 0 is zero
  movi  a6, MASK2 # mask for byte 2
  bany.w18  a2, a5, .Ldiff1 # if byte 1 differs
  extui a10, a8, 24, 8
  bnone.w18 a8, a5, .Leq  # if byte 1 is zero
  extui a11, a9, 24, 8
  bany.w18  a2, a6, .Ldiff2 # if byte 2 differs
  sub a2, a10, a11
  bnone.w18 a8, a6, .Leq  # if byte 2 is zero
  /* Little-endian is a little more difficult because can't subtract
     whole words.  */
.Ldiff3:
  /* Bytes 0-2 are equal; byte 3 is different.
     For little-endian need to have a sign bit for the difference.  */
  RET(16)
.Ldiff0:
  /* Byte 0 is different.  */
  sub a2, a10, a11
  RET(16)

.Ldiff1:
  /* Byte 0 is equal; byte 1 is different.  */
  extui a10, a8, 8, 8
  extui a11, a9, 8, 8
  sub a2, a10, a11
  RET(16)

.Ldiff2:
  /* Bytes 0-1 are equal; byte 2 is different.  */
  extui a10, a8, 16, 8
  extui a11, a9, 16, 8
  sub a2, a10, a11
  RET(16)

#else
#if XCHAL_HAVE_LOOPS && XCHAL_HAVE_DENSITY && !XCHAL_HAVE_BE && XCHAL_HAVE_PDX4
/*  Fast version for FLIX3 Little Endian */

  ENTRY(16)
  /* a2 = s1, a3 = s2 */

  l8ui  a8, a2, 0 # byte 0 from s1
  l8ui  a9, a3, 0 # byte 0 from s2
  movi  a10, 3    # mask
  movi  a5, 0xfffffffc
  or  a11, a2, a3
  movi  a4, MASK0 # mask for byte 0
  movi  a7, MASK4
  addi    a3, a3, -8
  addi    a2, a2, -8
  and a5, a5, a2
  bne.w15 a8, a9, .Lretdiff
  l32i  a8, a5, 8 # get word from aligned variant of s1

  bany.w15  a11, a10, .Lnot_aligned

/* s1 is word-aligned; s2 is word-aligned.

   If the zero-overhead loop option is available, use an (almost)
   infinite zero-overhead loop with conditional exits so we only pay
   for taken branches when exiting the loop.  */

/* New algorithm, relying on the fact that all normal ASCII is between
   32 and 127.

   Rather than check all bytes for zero:
   Take one word (4 bytes).  Call it w1.
   Shift w1 left by one into w1'.
   Or w1 and w1'.  For all normal ASCII bit 6 will be 1; for zero it won't.
   Check that all 4 bit 6's (one for each byte) are one:
   If they are, we are definitely not done.
   If they are not, we are probably done, but need to check for zero.  */

.Laligned:
  /* Loop forever */
1:
  loop  a0, .Laligned_done

  /* First unrolled loop body.  */
  l32i  a9, a3, 8 # get word from s2
  addi  a3, a3, 8 # advance s2 pointer
  slli  a5, a8, 1
  or  a10, a8, a5
  {
  bne.w15 a8, a9, .Lwne2
  l32i  a11, a2, 12 # get word from s1+4
  nop
  nop
  }
  l32i  a9, a3, 4 # get word from s2+4
  bnall.w15 a10, a7, .Lprobeq

  /* Second unrolled loop body.  */
  slli  a5, a11, 1
  or  a10, a11, a5
  addi  a2, a2, 8 # advance s1 pointer
        mov a8, a11
  bne.w15 a11, a9, .Lwne2
  l32i  a8, a2, 8 # get word from s1
  bnall.w15 a10, a7, .Lprobeq2

.Laligned_done:
  l32i  a8, a2, 8 # get word from s1
  j       1b

.Lnot_aligned:
  xor a11, a2, a3 # compare low two bits of s1 and s2
  bany  a11, a10, .Lunaligned # if they have different alignment

  /* s1/s2 are not word-aligned.  */
  movi  a5, 0xfffffffc
  addi  a2, a2, 1 # advance s1
  beqz  a9, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  and     a6, a2, a5
  l32i  a8, a6, 8 # get word from s1
  bnone a2, a10, .Laligned # if s1/s2 now aligned
  l8ui  a8, a2, 8 # byte 1 from s1
  l8ui  a9, a3, 8 # byte 1 from s2
  addi  a2, a2, 1 # advance s1
  bne a8, a9, .Lretdiff # if different, return difference
  beqz  a8, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  and     a6, a2, a5
  l32i  a8, a6, 8 # get word from s1
  bnone a2, a10, .Laligned # if s1/s2 now aligned
  l8ui  a8, a2, 8 # byte 2 from s1
  l8ui  a9, a3, 8 # byte 2 from s2
  addi  a2, a2, 1 # advance s1
  bne a8, a9, .Lretdiff # if different, return difference
  beqz  a8, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  l32i  a8, a2, 8 # get word from s1
  j .Laligned

/* s1 and s2 have different alignment.

   If the zero-overhead loop option is available, use an (almost)
   infinite zero-overhead loop with conditional exits so we only pay
   for taken branches when exiting the loop.

   Note: It is important for this unaligned case to come before the
   code for aligned strings, because otherwise some of the branches
   above cannot reach and have to be transformed to branches around
   jumps.  The unaligned code is smaller and the branches can reach
   over it.  */

.Lunaligned:
  movi.n  a8, 0   # set up for the maximum loop count
  loop  a8, .Lretdiff # loop forever (almost anyway)
  l8ui  a8, a2, 8
  l8ui  a9, a3, 8
  addi  a2, a2, 1
  bne a8, a9, .Lretdiff
  addi  a3, a3, 1
  beqz  a8, .Lretdiff
.Lretdiff:
  sub a2, a8, a9
  RET(16)

.Lprobeq2:
  /* Adjust pointers to account for the loop unrolling.  */
        mov a8, a11
  addi  a2, a2, -4
  addi  a3, a3, 4

  /* align (0 mod 4) */
.Lprobeq:
  /* Words are probably equal, but check for sure.
     If not, loop over the rest of string using normal algorithm.  */

  bnone a8, a4, .Leq  # if byte 0 is zero
  movi  a5, MASK1 # mask for byte 1
  movi  a6, MASK2 # mask for byte 2
  bnone a8, a5, .Leq  # if byte 1 is zero
  movi  a7, MASK3 # mask for byte 3
  bnone a8, a6, .Leq  # if byte 2 is zero
  bnone a8, a7, .Leq  # if byte 3 is zero
  /* align (1 mod 4) */
  addi.n  a2, a2, 12  # advance s1 pointer
  addi.n  a3, a3, 4 # advance s2 pointer
  /* align (1 mod 4) or (2 mod 4) */
1:
  loop  a0, .Lend # loop forever (a4 is bigger than max iters)

  l32i  a8, a2, 0 # get word from s1
  l32i  a9, a3, 0 # get word from s2
  addi  a2, a2, 4 # advance s1 pointer
  bne a8, a9, .Lwne
  bnone a8, a4, .Leq  # if byte 0 is zero
  bnone a8, a5, .Leq  # if byte 1 is zero
  bnone a8, a6, .Leq  # if byte 2 is zero
  bnone a8, a7, .Leq  # if byte 3 is zero
  addi  a3, a3, 4 # advance s2 pointer
.Lend:
  j 1b

  /* Words are equal; some byte is zero.  */
.Leq: movi  a2, 0   # return equal
  RET(16)

.Lwne2: /* Words are not equal.  On big-endian processors, if none of the
     bytes are zero, the return value can be determined by a simple
     comparison.  */
.Lwne:  /* Words are not equal.  */
  xor a2, a8, a9  # get word with nonzero in byte that differs
  extui a10, a8, 0, 8
  extui a11, a9, 0, 8
  movi  a5, MASK1 # mask for byte 1
  bany.w15  a2, a4, .Ldiff0 # if byte 0 differs

  bnone.w15 a8, a4, .Leq  # if byte 0 is zero
  movi  a6, MASK2 # mask for byte 2
  bany.w15  a2, a5, .Ldiff1 # if byte 1 differs
  extui a10, a8, 24, 8
  bnone.w15 a8, a5, .Leq  # if byte 1 is zero
  extui a11, a9, 24, 8
  bany.w15  a2, a6, .Ldiff2 # if byte 2 differs
  sub a2, a10, a11
  bnone.w15 a8, a6, .Leq  # if byte 2 is zero
  /* Little-endian is a little more difficult because can't subtract
     whole words.  */
.Ldiff3:
  /* Bytes 0-2 are equal; byte 3 is different.
     For little-endian need to have a sign bit for the difference.  */
  RET(16)
.Ldiff0:
  /* Byte 0 is different.  */
  sub a2, a10, a11
  RET(16)

.Ldiff1:
  /* Byte 0 is equal; byte 1 is different.  */
  extui a10, a8, 8, 8
  extui a11, a9, 8, 8
  sub a2, a10, a11
  RET(16)

.Ldiff2:
  /* Bytes 0-1 are equal; byte 2 is different.  */
  extui a10, a8, 16, 8
  extui a11, a9, 16, 8
  sub a2, a10, a11
  RET(16)

#else /* Not FLIX3 */
  ENTRY(16)
  /* a2 = s1, a3 = s2 */

  l8ui  a8, a2, 0 # byte 0 from s1
  l8ui  a9, a3, 0 # byte 0 from s2
  movi  a10, 3    # mask
  bne a8, a9, .Lretdiff

  or  a11, a2, a3
  bnone a11, a10, .Laligned

  xor a11, a2, a3 # compare low two bits of s1 and s2
  bany  a11, a10, .Lunaligned # if they have different alignment

  /* s1/s2 are not word-aligned.  */
  addi  a2, a2, 1 # advance s1
  beqz  a8, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  bnone a2, a10, .Laligned # if s1/s2 now aligned
  l8ui  a8, a2, 0 # byte 1 from s1
  l8ui  a9, a3, 0 # byte 1 from s2
  addi  a2, a2, 1 # advance s1
  bne a8, a9, .Lretdiff # if different, return difference
  beqz  a8, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  bnone a2, a10, .Laligned # if s1/s2 now aligned
  l8ui  a8, a2, 0 # byte 2 from s1
  l8ui  a9, a3, 0 # byte 2 from s2
  addi  a2, a2, 1 # advance s1
  bne a8, a9, .Lretdiff # if different, return difference
  beqz  a8, .Leq  # bytes equal, if zero, strings are equal
  addi  a3, a3, 1 # advance s2
  j .Laligned

/* s1 and s2 have different alignment.

   If the zero-overhead loop option is available, use an (almost)
   infinite zero-overhead loop with conditional exits so we only pay
   for taken branches when exiting the loop.

   Note: It is important for this unaligned case to come before the
   code for aligned strings, because otherwise some of the branches
   above cannot reach and have to be transformed to branches around
   jumps.  The unaligned code is smaller and the branches can reach
   over it.  */

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
.Lunaligned:
#if XCHAL_HAVE_LOOPS
#if XCHAL_HAVE_DENSITY
  _movi.n a8, 0   # set up for the maximum loop count
#else
  _movi a8, 0   # set up for the maximum loop count
#endif
  loop  a8, .Lretdiff # loop forever (almost anyway)
#endif
.Lnextbyte:
  l8ui  a8, a2, 0
  l8ui  a9, a3, 0
  addi  a2, a2, 1
  bne a8, a9, .Lretdiff
  addi  a3, a3, 1
#if XCHAL_HAVE_LOOPS
  beqz  a8, .Lretdiff
#else
  bnez  a8, .Lnextbyte
#endif
.Lretdiff:
  sub a2, a8, a9
  RET(16)

/* s1 is word-aligned; s2 is word-aligned.

   If the zero-overhead loop option is available, use an (almost)
   infinite zero-overhead loop with conditional exits so we only pay
   for taken branches when exiting the loop.  */

/* New algorithm, relying on the fact that all normal ASCII is between
   32 and 127.

   Rather than check all bytes for zero:
   Take one word (4 bytes).  Call it w1.
   Shift w1 left by one into w1'.
   Or w1 and w1'.  For all normal ASCII bit 6 will be 1; for zero it won't.
   Check that all 4 bit 6's (one for each byte) are one:
   If they are, we are definitely not done.
   If they are not, we are probably done, but need to check for zero.  */

  .align  4
#if XCHAL_HAVE_LOOPS
#if XCHAL_HAVE_CONST16
  /* (2 mod 4) alignment for loop instruction */
  .byte 0
#endif
.Laligned:
  movi  a4, MASK0 # mask for byte 0
  movi  a7, MASK4

  /* Loop forever */
1:
  loop  a0, .Laligned_done

  /* First unrolled loop body.  */
  l32i  a8, a2, 0 # get word from s1
  l32i  a9, a3, 0 # get word from s2
  slli  a5, a8, 1
  bne a8, a9, .Lwne2
  or  a9, a8, a5
  bnall a9, a7, .Lprobeq

  /* Second unrolled loop body.  */
  l32i  a8, a2, 4 # get word from s1+4
  l32i  a9, a3, 4 # get word from s2+4
  slli  a5, a8, 1
  bne a8, a9, .Lwne2
  or  a9, a8, a5
  bnall a9, a7, .Lprobeq2

  addi  a2, a2, 8 # advance s1 pointer
  addi  a3, a3, 8 # advance s2 pointer
.Laligned_done:
  j       1b

.Lprobeq2:
  /* Adjust pointers to account for the loop unrolling.  */
  addi  a2, a2, 4
  addi  a3, a3, 4

#else /* !XCHAL_HAVE_LOOPS */

.Laligned:
  movi  a4, MASK0 # mask for byte 0
  movi  a7, MASK4
  j .Lfirstword
.Lnextword:
  addi  a2, a2, 4 # advance s1 pointer
  addi  a3, a3, 4 # advance s2 pointer
.Lfirstword:
  l32i  a8, a2, 0 # get word from s1
  l32i  a9, a3, 0 # get word from s2
  slli  a5, a8, 1
  bne a8, a9, .Lwne2
  or  a9, a8, a5
  ball  a9, a7, .Lnextword
#endif /* !XCHAL_HAVE_LOOPS */

  /* align (0 mod 4) */
.Lprobeq:
  /* Words are probably equal, but check for sure.
     If not, loop over the rest of string using normal algorithm.  */

  bnone a8, a4, .Leq  # if byte 0 is zero
  movi  a5, MASK1 # mask for byte 1
  movi  a6, MASK2 # mask for byte 2
  bnone a8, a5, .Leq  # if byte 1 is zero
  movi  a7, MASK3 # mask for byte 3
  bnone a8, a6, .Leq  # if byte 2 is zero
  bnone a8, a7, .Leq  # if byte 3 is zero
  /* align (1 mod 4) */
#if XCHAL_HAVE_DENSITY
  addi.n  a2, a2, 4 # advance s1 pointer
  addi.n  a3, a3, 4 # advance s2 pointer
  /* align (1 mod 4) or (2 mod 4) */
#else
  addi  a2, a2, 4 # advance s1 pointer
  addi  a3, a3, 4 # advance s2 pointer
  or  a1, a1, a1  # nop
#if XCHAL_HAVE_CONST16
  or  a1, a1, a1  # nop
#endif
  /* align (2 mod 4) */
#endif /* XCHAL_HAVE_DENSITY */
#if XCHAL_HAVE_LOOPS
1:
  loop  a0, .Leq  # loop forever (a4 is bigger than max iters)
  l32i  a8, a2, 0 # get word from s1
  l32i  a9, a3, 0 # get word from s2
  addi  a2, a2, 4 # advance s1 pointer
  bne a8, a9, .Lwne
  bnone a8, a4, .Leq  # if byte 0 is zero
  bnone a8, a5, .Leq  # if byte 1 is zero
  bnone a8, a6, .Leq  # if byte 2 is zero
  bnone a8, a7, .Leq  # if byte 3 is zero
  addi  a3, a3, 4 # advance s2 pointer
  j 1b
#else /* !XCHAL_HAVE_LOOPS */

  j .Lfirstword2
.Lnextword2:
  addi  a3, a3, 4 # advance s2 pointer
.Lfirstword2:
  l32i  a8, a2, 0 # get word from s1
  l32i  a9, a3, 0 # get word from s2
  addi  a2, a2, 4 # advance s1 pointer
  bne a8, a9, .Lwne
  bnone a8, a4, .Leq  # if byte 0 is zero
  bnone a8, a5, .Leq  # if byte 1 is zero
  bnone a8, a6, .Leq  # if byte 2 is zero
  bany  a8, a7, .Lnextword2 # if byte 3 is zero
#endif /* !XCHAL_HAVE_LOOPS */

  /* Words are equal; some byte is zero.  */
.Leq: movi  a2, 0   # return equal
  RET(16)

.Lwne2: /* Words are not equal.  On big-endian processors, if none of the
     bytes are zero, the return value can be determined by a simple
     comparison.  */
#if XCHAL_HAVE_BE
  or  a10, a8, a5
  bnall a10, a7, .Lsomezero
  bgeu  a8, a9, .Lposreturn
  movi  a2, -1
  RET(16)
.Lposreturn:
  movi  a2, 1
  RET(16)
.Lsomezero: # There is probably some zero byte.
#endif /* XCHAL_HAVE_BE */
.Lwne:  /* Words are not equal.  */
  xor a2, a8, a9  # get word with nonzero in byte that differs
  bany  a2, a4, .Ldiff0 # if byte 0 differs
  movi  a5, MASK1 # mask for byte 1
  bnone a8, a4, .Leq  # if byte 0 is zero
  bany  a2, a5, .Ldiff1 # if byte 1 differs
  movi  a6, MASK2 # mask for byte 2
  bnone a8, a5, .Leq  # if byte 1 is zero
  bany  a2, a6, .Ldiff2 # if byte 2 differs
  bnone a8, a6, .Leq  # if byte 2 is zero
#if XCHAL_HAVE_BE
.Ldiff3:
.Ldiff2:
.Ldiff1:
  /* Byte 0 is equal (at least) and there is a difference before a zero
     byte.  Just subtract words to get the return value.
     The high order equal bytes cancel, leaving room for the sign.  */
  sub a2, a8, a9
  RET(16)

.Ldiff0:
  /* Need to make room for the sign, so can't subtract whole words.  */
  extui a10, a8, 24, 8
  extui a11, a9, 24, 8
  sub a2, a10, a11
  RET(16)

#else /* !XCHAL_HAVE_BE */
  /* Little-endian is a little more difficult because can't subtract
     whole words.  */
.Ldiff3:
  /* Bytes 0-2 are equal; byte 3 is different.
     For little-endian need to have a sign bit for the difference.  */
  extui a10, a8, 24, 8
  extui a11, a9, 24, 8
  sub a2, a10, a11
  RET(16)

.Ldiff0:
  /* Byte 0 is different.  */
  extui a10, a8, 0, 8
  extui a11, a9, 0, 8
  sub a2, a10, a11
  RET(16)

.Ldiff1:
  /* Byte 0 is equal; byte 1 is different.  */
  extui a10, a8, 8, 8
  extui a11, a9, 8, 8
  sub a2, a10, a11
  RET(16)

.Ldiff2:
  /* Bytes 0-1 are equal; byte 2 is different.  */
  extui a10, a8, 16, 8
  extui a11, a9, 16, 8
  sub a2, a10, a11
  RET(16)

#endif /* !XCHAL_HAVE_BE */
#endif /* FLIX3 */
#endif /* FLIX3 */

  .end  schedule
  .size strcmp, . - strcmp

