/****************************************************************************
 * libs/libc/misc/lib_umul64.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/lib/math32.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umul64
 *
 * Description:
 *   Multiply two 64-bit values, factor1 and factor2, and return the
 *   truncated 64-bit product.
 *
 * Input Parameters:
 *   factor1 and factor2 - The values to be multiplied
 *   product - The location to return the product of the two values.
 *
 ****************************************************************************/

void umul64(FAR const struct uint64_s *factor1,
            FAR const struct uint64_s *factor2,
            FAR struct uint64_s *product)
{
  struct uint64_s part1;
  struct uint64_s part2;

  /* factor1 = factor1->ms << 32 + factor1->ls
   * factor2 = factor2->ms << 32 + factor2->ls
   *
   * Full 128-bit product:
   *   factor1 * factor2 = (factor1->ms * factor2->ms << 64) +
   *                       factor1->ls * (factor2->ms << 32) +
   *                       factor2->ls * (factor1->ms << 32) +
   *                       factor1->ls * factor2->ls
   *
   * Truncated, 64-bit product:
   *   factor1 * factor2 = (factor1->ls * factor2->ms +
   *                        factor2->ls * factor1->ms) << 32) +
   *                       factor1->ls * factor2->ls
   *
   *   part1             = (factor1->ls * factor2->ms +
   *                       factor2->ls * factor1->ms) << 32)
   *   part2             = factor1->ls * factor2->ls
   *   factor1 * factor2 = part1 + part2
   */

  /* Get part1 = factor1->ls * factor2->ms + factor2->ls * factor1->ms,
   * shifting left by 32-bits (truncating to 64-bits)
   */

  part1.ms = factor1->ls * factor2->ms +
             factor2->ls * factor2->ms;
  part1.ls = 0;

  /* Get the full 64-bit part2 = factor1->ls * factor2->ls */

  umul32(factor1->ls, factor2->ls, &part2);

  /* The product is then the sum */

  uadd64(&part1, &part2, product);
}
