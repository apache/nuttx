/****************************************************************************
 * arch/ceva/include/limits.h
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

#ifndef __ARCH_CEVA_INCLUDE_LIMITS_H
#define __ARCH_CEVA_INCLUDE_LIMITS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include_next <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Assume ILP32 or LP64 */

#define PTR_MIN     LONG_MIN
#define PTR_MAX     LONG_MAX
#define UPTR_MAX    ULONG_MAX

#if !defined(__WCHAR_TYPE__)
#  define WCHAR_MIN INT_MIN
#  define WCHAR_MAX INT_MAX
#elif defined(__WCHAR_UNSIGNED__)
#  define WCHAR_MIN 0
#  define WCHAR_MAX __WCHAR_MAX__
#else
#  define WCHAR_MIN (-__WCHAR_MAX__ - 1)
#  define WCHAR_MAX __WCHAR_MAX__
#endif

#endif /* __ARCH_CEVA_INCLUDE_LIMITS_H */
