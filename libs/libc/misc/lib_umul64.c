/****************************************************************************
 * libs/libc/misc/lib_umul64.c
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
