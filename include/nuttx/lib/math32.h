/****************************************************************************
 * include/nuttx/lib/math32.h
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

#ifndef __INCLUDE_NUTTX_LIB_MATH32_H
#define __INCLUDE_NUTTX_LIB_MATH32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <inttypes.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Returns one plus the index of the most significant 1-bit of n,
 * or if n is zero, returns zero.
 */

#if UINTPTR_MAX > UINT32_MAX
#  define FLS(n) ((n) & UINT64_C(0xffffffff00000000) ? 32 + \
                  FLS32((size_t)(n) >> 32) : FLS32(n))
#else
#  define FLS(n) FLS32(n)
#endif

#define FLS32(n) ((n) & 0xffff0000 ? 16 + FLS16((n) >> 16) : FLS16(n))
#define FLS16(n) ((n) & 0xff00     ?  8 + FLS8 ((n) >>  8) : FLS8 (n))
#define FLS8(n)  ((n) & 0xf0       ?  4 + FLS4 ((n) >>  4) : FLS4 (n))
#define FLS4(n)  ((n) & 0xc        ?  2 + FLS2 ((n) >>  2) : FLS2 (n))
#define FLS2(n)  ((n) & 0x2        ?  1 + FLS1 ((n) >>  1) : FLS1 (n))
#define FLS1(n)  ((n) & 0x1        ?  1 : 0)

/* Returns round up and round down value of log2(n). Note: it can be used at
 * compile time.
 */

#define LOG2_CEIL(n)  ((n) & (n - 1) ? FLS(n) : FLS(n) - 1)
#define LOG2_FLOOR(n) (FLS(n) - 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These types are useful on platforms that do not support 64-bit types. */

struct int64_s
{
#ifdef CONFIG_ENDIAN_BIG
  int32_t  ms;
  uint32_t ls;
#else
  uint32_t ls;
  int32_t  ms;
#endif
};

struct uint64_s
{
#ifdef CONFIG_ENDIAN_BIG
  uint32_t ms;
  uint32_t ls;
#else
  uint32_t ls;
  uint32_t ms;
#endif
};

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

/****************************************************************************
 * Name: uneg64
 *
 * Description:
 *   Negate a 64-bit unsigned value.
 *
 * Input Parameters:
 *   value - The value to be negated.
 *
 ****************************************************************************/

/* void uneg64(FAR const uint64_s *value); */

#define uneg64(value) \
  do \
    { \
      value->ms = ~value->ms; \
      value->ls = -value->ls; \
      if (value->ls == 0) \
        { \
          value->ms++; \
        } \
    } \
  while (0)

/****************************************************************************
 * Name: uadd32x64
 *
 * Description:
 *   Add a 32-bit value to a 64-bit values and return the truncated 64-bit
 *   sum.
 *
 * Input Parameters:
 *   term1 and term2 - The values to be added
 *   sum - The location to return the product of the two values.  sum may
 *     be one of term1 or term2
 *
 ****************************************************************************/

void uadd32x64(uint32_t term1, FAR const struct uint64_s *term2,
               FAR struct uint64_s *sum);

/****************************************************************************
 * Name: uadd64
 *
 * Description:
 *   Add two 64-bit values and return a 64-bit sum.
 *
 * Input Parameters:
 *   term1 and term2 - The values to be added
 *   sum - The location to return the product of the two values.  sum may
 *     be one of term1 or term2
 *
 ****************************************************************************/

void uadd64(FAR const struct uint64_s *term1,
            FAR const struct uint64_s *term2,
            FAR struct uint64_s *sum);

/****************************************************************************
 * Name: usub64x32
 *
 * Description:
 *   Subtract a 32-bit value from a 64-bit value and return the 64-bit
 *   difference.
 *
 * Input Parameters:
 *   minuend    - The number from which another number (the Subtrahend) is
 *     to be subtracted.
 *   subtrahend - The number that is to be subtracted.
 *   difference - The location to return the difference of the two values.
 *     difference may the same as one of minuend or subtrahend.
 *
 ****************************************************************************/

void usub64x32(FAR const struct uint64_s *minuend, uint32_t subtrahend,
               FAR struct uint64_s *difference);

/****************************************************************************
 * Name: usub64
 *
 * Description:
 *   Subtract two 64-bit values and return the 64-bit difference.
 *
 * Input Parameters:
 *   minuend    - The number from which another number (the Subtrahend) is
 *     to be subtracted.
 *   subtrahend - The number that is to be subtracted.
 *   difference - The location to return the difference of the two values.
 *     difference may the same as one of minuend or subtrahend.
 *
 ****************************************************************************/

void usub64(FAR const struct uint64_s *minuend,
            FAR const struct uint64_s *subtrahend,
            FAR struct uint64_s *difference);

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

void umul32(uint32_t factor1, uint32_t factor2,
            FAR struct uint64_s *product);

/****************************************************************************
 * Name: umul32x64
 *
 * Description:
 *   Multiply one 32-bit and one 64-bit values, factor1 and factor2,
 *   respectively, and return the truncated 64-bit product.
 *
 * Input Parameters:
 *   factor1 and factor2 - The values to be multiplied
 *   product - The location to return the product of the two values.
 *
 ****************************************************************************/

void umul32x64(uint32_t factor1, FAR const struct uint64_s *factor2,
              FAR struct uint64_s *product);

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
            FAR struct uint64_s *product);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LIB_MATH32_H */
