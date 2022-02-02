/****************************************************************************
 * arch/sim/include/inttypes.h
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

#ifndef __ARCH_SIM_INCLUDE_INTTYPES_H
#define __ARCH_SIM_INCLUDE_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(__APPLE_CC__) || !defined(_LP64)
#  define _PRI64PREFIX "ll"
#  define _SCN64PREFIX "ll"
#  define INT64_C(x)  x ## ll
#  define UINT64_C(x) x ## ull
#else
#  define _PRI64PREFIX "l"
#  define _SCN64PREFIX "l"
#  define INT64_C(x)  x ## l
#  define UINT64_C(x) x ## ul
#endif

#  define PRId8       "d"
#  define PRId16      "d"
#  define PRId32      "d"
#  define PRId64      _PRI64PREFIX "d"

#  define PRIi8       "i"
#  define PRIi16      "i"
#  define PRIi32      "i"
#  define PRIi64      _PRI64PREFIX "i"

#  define PRIo8       "o"
#  define PRIo16      "o"
#  define PRIo32      "o"
#  define PRIo64      _PRI64PREFIX "o"

#  define PRIu8       "u"
#  define PRIu16      "u"
#  define PRIu32      "u"
#  define PRIu64      _PRI64PREFIX "u"

#  define PRIx8       "x"
#  define PRIx16      "x"
#  define PRIx32      "x"
#  define PRIx64      _PRI64PREFIX "x"

#  define PRIX8       "X"
#  define PRIX16      "X"
#  define PRIX32      "X"
#  define PRIX64      _PRI64PREFIX "X"

#  define SCNd8       "hhd"
#  define SCNd16      "hd"
#  define SCNd32      "d"
#  define SCNd64      _SCN64PREFIX "d"

#  define SCNi8       "hhi"
#  define SCNi16      "hi"
#  define SCNi32      "i"
#  define SCNi64      _SCN64PREFIX "i"

#  define SCNo8       "hho"
#  define SCNo16      "ho"
#  define SCNo32      "o"
#  define SCNo64      _SCN64PREFIX "o"

#  define SCNu8       "hhu"
#  define SCNu16      "hu"
#  define SCNu32      "u"
#  define SCNu64      _SCN64PREFIX "u"

#  define SCNx8       "hhx"
#  define SCNx16      "hx"
#  define SCNx32      "x"
#  define SCNx64      _SCN64PREFIX "x"

#  define INT8_C(x)   x
#  define INT16_C(x)  x
#  define INT32_C(x)  x

#  define UINT8_C(x)  x
#  define UINT16_C(x) x
#  define UINT32_C(x) x ## u

#if (defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)) || \
     defined(CONFIG_HOST_ARM64)
#  define PRIdPTR     "ld"
#  define PRIiPTR     "li"
#  define PRIoPTR     "lo"
#  define PRIuPTR     "lu"
#  define PRIxPTR     "lx"
#  define PRIXPTR     "lX"
#  define SCNdPTR     "ld"
#  define SCNiPTR     "li"
#  define SCNoPTR     "lo"
#  define SCNuPTR     "lu"
#  define SCNxPTR     "lx"
#else
#  define PRIdPTR     "d"
#  define PRIiPTR     "i"
#  define PRIoPTR     "o"
#  define PRIuPTR     "u"
#  define PRIxPTR     "x"
#  define PRIXPTR     "X"
#  define SCNdPTR     "d"
#  define SCNiPTR     "i"
#  define SCNoPTR     "o"
#  define SCNuPTR     "u"
#  define SCNxPTR     "x"
#endif

#endif /* __ARCH_SIM_INCLUDE_INTTYPES_H */
