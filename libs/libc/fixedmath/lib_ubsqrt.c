/****************************************************************************
 * libs/libc/fixedmath/lib_ubsqrt.c
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

#include <nuttx/config.h>
#include <fixedmath.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_LONG

/****************************************************************************
 * Name: ub32sqrtub16
 *
 * Description:
 *   ub32sqrtub16 calculates square root for 'a'
 *
 ****************************************************************************/

ub16_t ub32sqrtub16(ub32_t a)
{
  uint64_t n = a;
  uint64_t xk = n;

  /* Direct conversion of ub32_t to uint64_t is same operation as multiplying
   * 'a' by 2^32, therefore n = a * 2^32.
   */

  if (xk == UINT64_MAX)
    {
      /* Avoid 'xk + n / xk' overflow on first iteration. */

      xk = (uint64_t)1 << 63;
    }

  while (xk)
    {
      uint64_t xk1 = (xk + n / xk) >> 1;

      if (xk1 >= xk)
        {
          break;
        }

      xk = xk1;
    }

  /* 'xk' now holds 'sqrt(n)' => 'sqrt(a * 2^32)' => 'sqrt(a) * 2^16', thus
   * 'xk' holds square root of 'a' in ub16_t format.
   */

  return (ub16_t)xk;
}

#endif

/****************************************************************************
 * Name: ub16sqrtub8
 *
 * Description:
 *   ub16sqrtub8 calculates square root for 'a'
 *
 ****************************************************************************/

ub8_t ub16sqrtub8(ub16_t a)
{
  uint32_t n = a;
  uint32_t xk = n;

  /* Direct conversion of ub16_t to uint32_t is same operation as multiplying
   * 'a' by 2^16, therefore n = a * 2^16.
   */

  if (xk == UINT32_MAX)
    {
      /* Avoid 'xk + n / xk' overflow on first iteration. */

      xk = (uint32_t)1 << 31;
    }

  while (xk)
    {
      uint32_t xk1 = (xk + n / xk) >> 1;

      if (xk1 >= xk)
        {
          break;
        }

      xk = xk1;
    }

  /* 'xk' now holds 'sqrt(n)' => 'sqrt(a * 2^16)' => 'sqrt(a) * 2^8', thus
   * 'xk' holds square root of 'a' in ub8_t format.
   */

  return (ub8_t)xk;
}
