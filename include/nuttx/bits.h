/****************************************************************************
 * include/nuttx/bits.h
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

#ifndef __INCLUDE_NUTTX_BITS_H
#define __INCLUDE_NUTTX_BITS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <inttypes.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef BITS_PER_BYTE
# define BITS_PER_BYTE 8
#endif

#if UINTPTR_MAX > UINT32_MAX
# define BITS_PER_LONG 64
#else
# define BITS_PER_LONG 32
#endif

#ifndef BITS_PER_LONG_LONG
# define BITS_PER_LONG_LONG 64
#endif

#define BIT_MASK(nr)       (UINT32_C(1) << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)       ((nr) / BITS_PER_LONG)
#define BIT_ULL_MASK(nr)   (UINT64_C(1) << ((nr) % BITS_PER_LONG_LONG))
#define BIT_ULL_WORD(nr)   ((nr) / BITS_PER_LONG_LONG)

/* Create a contiguous bitmask starting at bit position @l and ending at
 * position @h. For example
 * GENMASK_ULL(39, 21) gives us the 64bit vector 0x000000ffffe00000.
 */

#define __GENMASK(h, l) \
        (((~UINT32_C(0)) - (UINT32_C(1) << (l)) + 1) & \
         (~UINT32_C(0) >> (BITS_PER_LONG - 1 - (h))))
#define GENMASK(h, l) \
        (BUILD_BUG_ON_ZERO((l) > (h)) + __GENMASK(h, l))

#define __GENMASK_ULL(h, l) \
        (((~UINT64_C(0)) - (UINT64_C(1) << (l)) + 1) & \
         (~UINT64_C(0) >> (BITS_PER_LONG_LONG - 1 - (h))))
#define GENMASK_ULL(h, l) \
        (BUILD_BUG_ON_ZERO((l) > (h)) + __GENMASK_ULL(h, l))

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_BITS_H */
