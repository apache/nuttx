/****************************************************************************
 * libs/libc/limits_check.c
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

/* NuttX prefers defining limits per architecture directly as constants
 * instead of using compiler provided definitions. That is to support
 * compilers that do not provide such definitions. On the other hand in case
 * compiler provides these definitions we should not deviate from them. This
 * file thus contains only checks of NuttX's defined limits against compiler
 * definitions if available.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <limits.h>
#include <float.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifdef static_assert
#  define static_assert_equal(COMPILER_DEF, DEF) \
  static_assert(COMPILER_DEF == DEF, "Compiler definition mismatch for " #DEF)
#else
#  define DEFINITION_ASSERT(COMPILER_DEF, DEF)
#endif

#ifdef __CHAR_BIT__
static_assert_equal(__CHAR_BIT__, CHAR_BIT);
#endif

#ifdef __SCHAR_MAX__
static_assert_equal(__SCHAR_MAX__, SCHAR_MAX);
#endif

#ifdef __SHRT_MAX__
static_assert_equal(__SHRT_MAX__, SHRT_MAX);
#endif

#ifdef __LONG_MAX__
static_assert_equal(__LONG_MAX__, LONG_MAX);
#endif

#ifdef __LONG_LONG_MAX__
static_assert_equal(__LONG_LONG_MAX__, LLONG_MAX);
#endif

#ifdef __UINTPTR_MAX__
static_assert_equal(__UINTPTR_MAX__, UPTR_MAX);
#endif

#ifdef __WCHAR_MAX__
static_assert_equal(__WCHAR_MAX__, WCHAR_MAX);
#endif

#ifdef __FLT_MANT_DIG__
static_assert_equal(__FLT_MANT_DIG__, FLT_MANT_DIG);
#endif

#ifdef __DBL_MANT_DIG__
static_assert_equal(__DBL_MANT_DIG__, DBL_MANT_DIG);
#endif

#ifdef __LDBL_MANT_DIG__
static_assert_equal(__LDBL_MANT_DIG__, LDBL_MANT_DIG);
#endif

#ifdef __FLT_DIG__
static_assert_equal(__FLT_DIG__, FLT_DIG);
#endif

#ifdef __DBL_DIG__
static_assert_equal(__DBL_DIG__, DBL_DIG);
#endif

#ifdef __LDBL_DIG__
static_assert_equal(__LDBL_DIG__, LDBL_DIG);
#endif

#ifdef __FLT_MIN_EXP__
static_assert_equal(__FLT_MIN_EXP__, FLT_MIN_EXP);
#endif

#ifdef __DBL_MIN_EXP__
static_assert_equal(__DBL_MIN_EXP__, DBL_MIN_EXP);
#endif

#ifdef __LDBL_MIN_EXP__
static_assert_equal(__LDBL_MIN_EXP__, LDBL_MIN_EXP);
#endif

#ifdef __FLT_MIN_10_EXP__
static_assert_equal(__FLT_MIN_10_EXP__, FLT_MIN_10_EXP);
#endif

#ifdef __DBL_MIN_10_EXP__
static_assert_equal(__DBL_MIN_10_EXP__, DBL_MIN_10_EXP);
#endif

#ifdef __LDBL_MIN_10_EXP__
static_assert_equal(__LDBL_MIN_10_EXP__, LDBL_MIN_10_EXP);
#endif

#ifdef __FLT_MAX_EXP__
static_assert_equal(__FLT_MAX_EXP__, FLT_MAX_EXP);
#endif

#ifdef __DBL_MAX_EXP__
static_assert_equal(__DBL_MAX_EXP__, DBL_MAX_EXP);
#endif

#ifdef __LDBL_MAX_EXP__
static_assert_equal(__LDBL_MAX_EXP__, LDBL_MAX_EXP);
#endif

#ifdef __FLT_MAX_10_EXP__
static_assert_equal(__FLT_MAX_10_EXP__, FLT_MAX_10_EXP);
#endif

#ifdef __DBL_MAX_10_EXP__
static_assert_equal(__DBL_MAX_10_EXP__, DBL_MAX_10_EXP);
#endif

#ifdef __LDBL_MAX_10_EXP__
static_assert_equal(__LDBL_MAX_10_EXP__, LDBL_MAX_10_EXP);
#endif

#ifdef __FLT_MAX__
static_assert_equal(__FLT_MAX__, FLT_MAX);
#endif

#ifdef __DBL_MAX__
static_assert_equal(__DBL_MAX__, DBL_MAX);
#endif

#ifdef __LDBL_MAX__
static_assert_equal(__LDBL_MAX__, LDBL_MAX);
#endif

#ifdef __FLT_EPSILON__
static_assert_equal(__FLT_EPSILON__, FLT_EPSILON);
#endif

#ifdef __DBL_EPSILON__
static_assert_equal(__DBL_EPSILON__, DBL_EPSILON);
#endif

#ifdef __LDBL_EPSILON__
static_assert_equal(__LDBL_EPSILON__, LDBL_EPSILON);
#endif

#ifdef __FLT_MIN__
static_assert_equal(__FLT_MIN__, FLT_MIN);
#endif

#ifdef __DBL_MIN__
static_assert_equal(__DBL_MIN__, DBL_MIN);
#endif

#ifdef __LDBL_MIN__
static_assert_equal(__LDBL_MIN__, LDBL_MIN);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/
