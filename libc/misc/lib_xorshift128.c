/****************************************************************************
 * libc/misc/lib_xorshift.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: David S. Alessio
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

/* This random number generator is simple, fast and portable.
 *      Ref:  https://en.wikipedia.org/wiki/Xorshift
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct xorshift128_state_s
{
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t w;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct xorshift128_state_s g_prng;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xorshift128_seed
 *
 * Description:
 *   Seed the XorShift128 PRNG
 *
 * Input Parameters:
 *   w, x, y, z:  Values for XorShift128 generation state.
 *
 * Returned Value:
 *   No
 *
 ****************************************************************************/

void xorshift128_seed(uint32_t w, uint32_t x, uint32_t y, uint32_t z)
{
  /* Seed the PRNG */

  g_prng.w = w;
  g_prng.x = x;
  g_prng.y = y;
  g_prng.z = z;
}

/****************************************************************************
 * Name: xorshift128
 *
 * Description:
 *   Generate one 32-bit pseudo-random number
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The generated pseudo-random number
 *
 ****************************************************************************/

uint32_t xorshift128(void)
{
  uint32_t t;

  t        = g_prng.x;
  t       ^= t << 11;
  t       ^= t >> 8;

  g_prng.x = g_prng.y;
  g_prng.y = g_prng.z;
  g_prng.z = g_prng.w;

  g_prng.w ^= g_prng.w >> 19;
  g_prng.w ^= t;

  return g_prng.w;
}
