/****************************************************************************
 * arch/z80/include/ez80/inttypes.h
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

#ifndef __ARCH_Z80_INCLUDE_EZ80_INTTYPES_H
#define __ARCH_Z80_INCLUDE_EZ80_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EZ80_Z80MODE

#  define PRId8       "d"
#  define PRId16      "d"
#  define PRId32      "ld"

#  define PRIdPTR     "d"

#  define PRIi8       "i"
#  define PRIi16      "i"
#  define PRIi32      "li"

#  define PRIiPTR     "i"

#  define PRIo8       "o"
#  define PRIo16      "o"
#  define PRIo32      "lo"

#  define PRIoPTR     "o"

#  define PRIu8       "u"
#  define PRIu16      "u"
#  define PRIu32      "lu"

#  define PRIuPTR     "u"

#  define PRIx8       "x"
#  define PRIx16      "x"
#  define PRIx32      "lx"

#  define PRIxPTR     "x"

#  define PRIX8       "X"
#  define PRIX16      "X"
#  define PRIX32      "lX"

#  define PRIXPTR     "X"

#  define SCNd8       "hhd"
#  define SCNd16      "hd"
#  define SCNd32      "ld"

#  define SCNdPTR     "hd"

#  define SCNi8       "hhi"
#  define SCNi16      "hi"
#  define SCNi32      "li"

#  define SCNiPTR     "hi"

#  define SCNo8       "hho"
#  define SCNo16      "ho"
#  define SCNo32      "lo"

#  define SCNoPTR     "ho"

#  define SCNu8       "hhu"
#  define SCNu16      "hu"
#  define SCNu32      "lu"

#  define SCNuPTR     "hu"

#  define SCNx8       "hhx"
#  define SCNx16      "hx"
#  define SCNx32      "lx"

#  define SCNxPTR     "hx"

#  define INT8_C(x)   x
#  define INT16_C(x)  x
#  define INT24_C(x)  x
#  define INT32_C(x)  x ## l

#  define UINT8_C(x)  x
#  define UINT16_C(x) x
#  define UINT24_C(x) x
#  define UINT32_C(x) x ## ul

#else

#  define PRId8       "d"
#  define PRId16      "d"
#  define PRId32      "ld"

#  define PRIdPTR     "d"

#  define PRIi8       "i"
#  define PRIi16      "i"
#  define PRIi32      "li"

#  define PRIiPTR     "i"

#  define PRIo8       "o"
#  define PRIo16      "o"
#  define PRIo32      "lo"

#  define PRIoPTR     "o"

#  define PRIu8       "u"
#  define PRIu16      "u"
#  define PRIu32      "lu"

#  define PRIuPTR     "u"

#  define PRIx8       "x"
#  define PRIx16      "x"
#  define PRIx32      "lx"

#  define PRIxPTR     "x"

#  define PRIX8       "X"
#  define PRIX16      "X"
#  define PRIX32      "lX"

#  define PRIXPTR     "X"

#  define SCNd8       "hhd"
#  define SCNd16      "hd"
#  define SCNd32      "ld"

#  define SCNdPTR     "d"

#  define SCNi8       "hhi"
#  define SCNi16      "hi"
#  define SCNi32      "li"

#  define SCNiPTR     "i"

#  define SCNo8       "hho"
#  define SCNo16      "ho"
#  define SCNo32      "lo"

#  define SCNoPTR     "o"

#  define SCNu8       "hhu"
#  define SCNu16      "hu"
#  define SCNu32      "lu"

#  define SCNuPTR     "u"

#  define SCNx8       "hhx"
#  define SCNx16      "hx"
#  define SCNx32      "lx"

#  define SCNxPTR     "x"

#  define INT8_C(x)   x
#  define INT16_C(x)  x
#  define INT24_C(x)  x
#  define INT32_C(x)  x ## l

#  define UINT8_C(x)  x
#  define UINT16_C(x) x
#  define UINT24_C(x) x
#  define UINT32_C(x) x ## ul

#  ifdef __clang__
#    define PRId64      "lld"
#    define PRIi64      "lli"
#    define PRIo64      "llo"
#    define PRIu64      "llu"
#    define PRIx64      "llx"
#    define PRIX64      "llX"
#    define SCNd64      "lld"
#    define SCNi64      "lli"
#    define SCNo64      "llo"
#    define SCNu64      "llu"
#    define SCNx64      "llx"
#    define INT64_C(x) x ## ll
#    define UINT64_C(x) x ## ull
#  endif

#endif

#endif /* __ARCH_Z80_INCLUDE_EZ80_INTTYPES_H */
