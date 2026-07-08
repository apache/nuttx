/****************************************************************************
 * libs/libc/stdlib/lib_srand.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdlib.h>
#include <stdint.h>

#include <nuttx/lib/lib.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_randstate = 1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xorshift32
 *
 * Description:
 *   Marsaglia's xorshift32 PRNG.  Period 2^32 - 1 (about 4.29 billion).
 *   Much better statistical properties than a simple LCG with a small
 *   modulus, while remaining fast (three XOR/shift operations).
 *
 ****************************************************************************/

static inline uint32_t xorshift32(FAR uint32_t *state)
{
  uint32_t x = *state;

  /* State must never be zero, otherwise the generator is stuck.
   * The seed path (srand / rand_r) already guards against this, but we
   * check here as well for safety.
   */

  if (x == 0)
    {
      x = 1;
    }

  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;

  *state = x;
  return x;
}

static unsigned long nrand_r(unsigned long limit,
                             FAR uint32_t *state)
{
  uint32_t randval = xorshift32(state);

  /* Map the 32-bit random value into the range [0, limit).
   * The modulo bias is negligible for small limits relative to 2^32.
   */

  return (unsigned long)(randval % (uint32_t)limit);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: srand
 *
 * Description:
 *   Seed the random number generator.
 *
 ****************************************************************************/

void srand(unsigned int seed)
{
  /* Avoid a zero state which would make xorshift32 degenerate. */

  g_randstate = (seed == 0) ? 1 : (uint32_t)seed;
}

/****************************************************************************
 * Name: nrand
 *
 * Description:
 *   Return a random, unsigned long value in the range of 0 to (limit - 1)
 *
 ****************************************************************************/

unsigned long nrand(unsigned long limit)
{
  return nrand_r(limit, &g_randstate);
}

/****************************************************************************
 * Name: rand_r
 *
 * Description:
 *   The function rand() is not reentrant, since it uses hidden state that
 *   is modified on each call. This might just be the seed value to be used
 *   by the next call, or it might be something more elaborate. In order to
 *   get reproducible behavior in a threaded application, this state must be
 *   made explicit; this can be done using the reentrant function rand_r().
 *
 *   Return a random, int value in the range of 0 to INT_MAX.
 *
 ****************************************************************************/

int rand_r(FAR unsigned int *seedp)
{
  uint32_t state = (uint32_t)*seedp;
  unsigned long rand;

  if (state == 0)
    {
      state = 1;
    }

  rand = nrand_r(INT_MAX, &state);
  *seedp = (unsigned int)state;
  return (int)rand;
}
