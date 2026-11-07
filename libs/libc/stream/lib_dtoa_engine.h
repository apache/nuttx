/****************************************************************************
 * libs/libc/stream/lib_dtoa_engine.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2018, Keith Packard. All rights reserved.
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

#ifndef __LIBS_LIBC_STREAM_LIB_DTOA_ENGINE_H
#define __LIBS_LIBC_STREAM_LIB_DTOA_ENGINE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdbool.h>
#include <stdint.h>
#include <float.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The most digits that the first path has: enough to read a value back */

#if DBL_MANT_DIG == 53
#  define DTOA_MAX_DIG      17
#else
#  define DTOA_MAX_DIG      9
#endif

/* No value has more decimals than this, so a larger number means the same */

#define DTOA_MAX_DECIMALS   20000

/* The limbs of the largest value in base 10000, or of the smallest one in
 * base 65536, whichever has more.
 */

#if DBL_MANT_DIG == 53
#  define DTOA_BIG_LIMBS    78
#else
#  define DTOA_BIG_LIMBS    11
#endif

/* A long double with more bits than a double.  The 64-bit mantissa format is
 * the little-endian one of x86.  It needs 16384 bits in base 10000.
 */

#if defined(CONFIG_LIBC_PRINTF_EXACT_LDOUBLE) && \
    defined(CONFIG_HAVE_LONG_DOUBLE) && \
    (LDBL_MANT_DIG == 113 || \
     (LDBL_MANT_DIG == 64 && !defined(CONFIG_ENDIAN_BIG)))
#  define HAVE_DTOA_LDOUBLE
#  define DTOA_BIG_LD_LIMBS 1240
#endif

#define DTOA_MINUS          1
#define DTOA_ZERO           2
#define DTOA_INF            4
#define DTOA_NAN            8

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_LIBC_PRINTF_EXACT

/* The state of the exact digits */

struct dtoa_big_s
{
  FAR uint16_t *limbs;
  uint64_t hi;              /* The mantissa */
  uint64_t lo;
  int32_t e2;               /* The value is the mantissa * 2^e2 */
  int32_t n;                /* The digits that count */
  int32_t nz;               /* The digits up to the last one that is not 0 */
  int32_t up;               /* The digit that grows by 1 */
  int32_t pos;              /* The digits that the output took */
  int limb;                 /* The limbs left of the integer part */
  int nl;                   /* The limbs left of the fraction */
  uint16_t group;           /* The last 4 digits */
  uint8_t ingroup;          /* The digits left in it */
  bool infrac;
  char cur;                 /* Digit pos - 1 */
};
#endif

struct dtoa_s
{
  int32_t exp;
  uint8_t flags;
  char digits[DTOA_MAX_DIG + 1];
#ifdef CONFIG_LIBC_PRINTF_EXACT
  FAR struct dtoa_big_s *big; /* Not NULL: the digits come from here */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int __dtoa_engine(double x, FAR struct dtoa_s *dtoa, int max_digits,
                  int max_decimals);

#ifdef CONFIG_LIBC_PRINTF_EXACT
int32_t __dtoa_big_start(FAR struct dtoa_big_s *b, FAR uint16_t *limbs,
                         double x, FAR struct dtoa_s *dtoa,
                         int32_t max_digits, int32_t decimals);
char __dtoa_big_digit(FAR struct dtoa_big_s *b, int32_t i);
#  ifdef HAVE_DTOA_LDOUBLE
int32_t __dtoa_big_start_ld(FAR struct dtoa_big_s *b, FAR uint16_t *limbs,
                            long double x, FAR struct dtoa_s *dtoa,
                            int32_t max_digits, int32_t decimals);
#  endif
#endif

#endif /* __LIBS_LIBC_STREAM_LIB_DTOA_ENGINE_H */
