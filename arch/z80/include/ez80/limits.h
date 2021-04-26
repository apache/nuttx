/****************************************************************************
 * arch/z80/include/ez80/limits.h
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

#ifndef __ARCH_Z80_INCLUDE_EZ80_LIMITS_H
#define __ARCH_Z80_INCLUDE_EZ80_LIMITS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHAR_BIT    8
#define SCHAR_MIN   (-SCHAR_MAX - 1)
#define SCHAR_MAX   127
#define UCHAR_MAX   255

/* These could be different on machines where char is unsigned */

#ifdef __CHAR_UNSIGNED__
#define CHAR_MIN    0
#define CHAR_MAX    UCHAR_MAX
#else
#define CHAR_MIN    SCHAR_MIN
#define CHAR_MAX    SCHAR_MAX
#endif

#define SHRT_MIN    (-SHRT_MAX - 1)
#define SHRT_MAX    32767
#define USHRT_MAX   65535U

#define INT_MIN     (-INT_MAX - 1)
#define INT_MAX     32767
#define UINT_MAX    65535U

/* These change on 32-bit and 64-bit platforms */

#define LONG_MIN    (-LONG_MAX - 1)
#define LONG_MAX    2147483647L
#define ULONG_MAX   4294967295UL

/* A pointer is 2 or 3 bytes, depending upon if the ez80 is in z80
 * compatibility mode or not
 *
 *   Z80 mode - 16 bits
 *   ADL mode - 24 bits
 */

#define PTR_MIN     (-PTR_MAX - 1)
#ifdef CONFIG_EZ80_Z80MODE
#define PTR_MAX     32767
#define UPTR_MAX    65535U
#else
#define PTR_MAX     8388607
#define UPTR_MAX    16777215U
#endif

#ifdef __clang__
#define LLONG_MIN       (-LLONG_MAX - 1)
#define LLONG_MAX       9223372036854775807LL
#define ULLONG_MAX      18446744073709551615ULL
#endif

#endif /* __ARCH_Z80_INCLUDE_EZ80_LIMITS_H */
