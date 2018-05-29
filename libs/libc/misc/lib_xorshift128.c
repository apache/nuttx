/****************************************************************************
 * libs/libc/misc/lib_xorshift.c
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
