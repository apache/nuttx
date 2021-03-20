/****************************************************************************
 * include/nuttx/lib/xorshift128.h
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

#ifndef __INCLUDE_NUTTX_LIB_XORSHIFT128_H
#define __INCLUDE_NUTTX_LIB_XORSHIFT128_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default XorShift128 state initializer */

#define XORSHIFT128_INITIALIZER { 97, 101, 97 << 17, 101 << 25 }

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Provides the state of the XorShift128 PRNG */

struct xorshift128_state_s
{
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t w;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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

uint32_t xorshift128(FAR struct xorshift128_state_s *state);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LIB_XORSHIFT128_H */
