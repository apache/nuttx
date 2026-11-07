/****************************************************************************
 * libs/libc/stream/lib_hexfloat.h
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

#ifndef __LIBS_LIBC_STREAM_LIB_HEXFLOAT_H
#define __LIBS_LIBC_STREAM_LIB_HEXFLOAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <float.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A long double with more bits than a double has its own entry point.  The
 * 64-bit mantissa format is the little-endian one of x86.
 */

#if defined(CONFIG_HAVE_LONG_DOUBLE) && \
    (LDBL_MANT_DIG == 113 || \
     (LDBL_MANT_DIG == 64 && !defined(CONFIG_ENDIAN_BIG)))
#  define HAVE_HEXFLOAT_LDOUBLE
#  define HEXFLOAT_MAX_DIG  ((LDBL_MANT_DIG + 2) / 4)
#else
#  define HEXFLOAT_MAX_DIG  ((DBL_MANT_DIG + 2) / 4)
#endif

#define HEXFLOAT_MINUS      1
#define HEXFLOAT_INF        2
#define HEXFLOAT_NAN        4

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hexfloat_s
{
  int16_t exp;                        /* The exponent of two */
  uint8_t flags;
  uint8_t ndigs;                      /* The digits after the point */
  char digits[HEXFLOAT_MAX_DIG + 1];  /* The first one is before the point */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* A negative precision means as many digits as the value needs */

void __hexfloat_engine(double x, FAR struct hexfloat_s *hex, int prec,
                       bool upper);

#ifdef HAVE_HEXFLOAT_LDOUBLE
void __hexfloat_engine_ld(long double x, FAR struct hexfloat_s *hex,
                          int prec, bool upper);
#endif

#endif /* __LIBS_LIBC_STREAM_LIB_HEXFLOAT_H */
