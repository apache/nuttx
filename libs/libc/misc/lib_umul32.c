/****************************************************************************
 * libs/libc/misc/lib_umul32.c
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
