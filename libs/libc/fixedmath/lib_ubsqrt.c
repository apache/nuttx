/****************************************************************************
 * libs/libc/fixedmath/lib_ubsqrt.c
 *
 *   Copyright (C) 2014,2017 Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

  do
    {
      uint64_t xk1 = (xk + n / xk) >> 1;

      if (xk1 >= xk)
        {
          break;
        }

      xk = xk1;
    }
  while (1);

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

  do
    {
      uint32_t xk1 = (xk + n / xk) >> 1;

      if (xk1 >= xk)
        {
          break;
        }

      xk = xk1;
    }
  while (1);

  /* 'xk' now holds 'sqrt(n)' => 'sqrt(a * 2^16)' => 'sqrt(a) * 2^8', thus
   * 'xk' holds square root of 'a' in ub8_t format.
   */

  return (ub8_t)xk;
}
