/****************************************************************************
 * arch/arm/include/inttypes.h
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

#ifndef __ARCH_ARM_INCLUDE_INTTYPES_H
#define __ARCH_ARM_INCLUDE_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define signed      +0
#define unsigned    +0
#define int         +2
#define long        +4

#define PRId8       "d"
#define PRId16      "d"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define PRId32    "d"
#else
#  define PRId32    "ld"
#endif
#define PRId64      "lld"
#define PRIdPTR     "d"

#define PRIi8       "i"
#define PRIi16      "i"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define PRIi32    "i"
#else
#  define PRIi32    "li"
#endif
#define PRIi64      "lli"
#define PRIiPTR     "i"

#define PRIo8       "o"
#define PRIo16      "o"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define PRIo32    "o"
#else
#  define PRIo32    "lo"
#endif
#define PRIo64      "llo"
#define PRIoPTR     "o"

#define PRIu8       "u"
#define PRIu16      "u"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define PRIu32    "u"
#else
#  define PRIu32    "lu"
#endif
#define PRIu64      "llu"
#define PRIuPTR     "u"

#define PRIx8       "x"
#define PRIx16      "x"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define PRIx32    "x"
#else
#  define PRIx32    "lx"
#endif
#define PRIx64      "llx"
#define PRIxPTR     "x"

#define PRIX8       "X"
#define PRIX16      "X"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define PRIX32    "X"
#else
#  define PRIX32    "lX"
#endif
#define PRIX64      "llX"
#define PRIXPTR     "X"

#define SCNd8       "hhd"
#define SCNd16      "hd"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define SCNd32    "d"
#else
#  define SCNd32    "ld"
#endif
#define SCNd64      "lld"
#define SCNdPTR     "d"

#define SCNi8       "hhi"
#define SCNi16      "hi"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define SCNi32    "i"
#else
#  define SCNi32    "li"
#endif
#define SCNi64      "lli"
#define SCNiPTR     "i"

#define SCNo8       "hho"
#define SCNo16      "ho"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define SCNo32    "o"
#else
#  define SCNo32    "lo"
#endif
#define SCNo64      "llo"
#define SCNoPTR     "o"

#define SCNu8       "hhu"
#define SCNu16      "hu"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define SCNu32    "u"
#else
#  define SCNu32    "lu"
#endif
#define SCNu64      "llu"
#define SCNuPTR     "u"

#define SCNx8       "hhx"
#define SCNx16      "hx"
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define SCNx32    "x"
#else
#  define SCNx32    "lx"
#endif
#define SCNx64      "llx"
#define SCNxPTR     "x"

#define INT8_C(x)   x
#define INT16_C(x)  x
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define INT32_C(x) x
#else
#  define INT32_C(x) x ## l
#endif
#define INT64_C(x)  x ## ll

#define UINT8_C(x)  x
#define UINT16_C(x) x
#if defined(__INT32_TYPE__) && __INT32_TYPE__ == int
#  define UINT32_C(x) x ## u
#else
#  define UINT32_C(x) x ## ul
#endif
#define UINT64_C(x) x ## ull

#undef signed
#undef unsigned
#undef int
#undef long

#endif /* __ARCH_ARM_INCLUDE_INTTYPES_H */
