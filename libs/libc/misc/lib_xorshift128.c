/****************************************************************************
 * libs/libc/misc/lib_xorshift128.c
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

/* This random number generator is simple, fast and portable.
 *      Ref:  https://en.wikipedia.org/wiki/Xorshift
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>

#include <nuttx/lib/xorshift128.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xorshift128
 *
 * Description:
 *   Generate one 32-bit pseudo-random number.
 *
 *   NOTE: Because the PRNG state is passed as a parameter, this function is
 *   fully re-entrant and may be called from an interrupt handler.
 *
 *   The downside to this is that users of the PRNG might not get as much
 *   entropy as if it were a common state structure.
 *
 * Input Parameters:
 *   state - The current XorShift128 state.
 *
 * Returned Value:
 *   The generated pseudo-random number
 *
 ****************************************************************************/

uint32_t xorshift128(FAR struct xorshift128_state_s *state)
{
  uint32_t t;

  DEBUGASSERT(state != NULL);

  t        = state->x;
  t       ^= t << 11;
  t       ^= t >> 8;

  state->x = state->y;
  state->y = state->z;
  state->z = state->w;

  state->w ^= state->w >> 19;
  state->w ^= t;

  return state->w;
}
