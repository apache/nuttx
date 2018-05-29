/****************************************************************************
 * libs/libc/misc/lib_umul32.c
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
 * Name: umul32
 *
 * Description:
 *   Multiply two 32-bit values, factor1 and factor2, and return the
 *   full 64-bit product.
 *
 * Input Parameters:
 *   factor1 and factor2 - The values to be multiplied
 *   product - The location to return the product of the two values.
 *
 ****************************************************************************/

void umul32(uint32_t factor1, uint32_t factor2, FAR struct uint64_s *product)
{
  struct uint64_s part2;
  uint32_t ms1;
  uint32_t ls1;
  uint32_t ms2;
  uint32_t ls2;
  uint32_t tmp;

  /* factor1 = ms1 << 16 + ls1
   * factor2 = ms2 << 16 + ls2
   */

  ms1         = factor1 >> 16;
  ls1         = factor1 & 0x0000ffff;
  ms2         = factor2 >> 16;
  ls2         = factor2 & 0x0000ffff;

  /* factor1 * factor2 = (ms1 * ms2 << 32) +
   *                     ls1 * (ms2 << 16) + ls2 * (ms1 << 16) +
   *                     ls1 * ls2
   * part1             = (ms1 * ms2 << 32) + ls1 * ls2
   * part2             = ls1 * (ms2 << 16) + ls2 * (ms1 << 16)
   * factor1 * factor2 = part1 + part2
   */

  product->ms = ms1 * ms2;
  product->ls = ls1 * ls2;

  tmp         = ls1 * ms2 + ls2 * ms1;
  part2.ms    = tmp >> 16;
  part2.ls    = tmp << 16;

  uadd64(product, &part2, product);
}
