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

#define PRId8       "d"
#define PRId16      "d"
#define PRId32      "ld"
#define PRId64      "lld"

#define PRIdPTR     "d"

#define PRIi8       "i"
#define PRIi16      "i"
#define PRIi32      "li"
#define PRIi64      "lli"

#define PRIiPTR     "i"

#define PRIo8       "o"
#define PRIo16      "o"
#define PRIo32      "lo"
#define PRIo64      "llo"

#define PRIoPTR     "o"

#define PRIu8       "u"
#define PRIu16      "u"
#define PRIu32      "lu"
#define PRIu64      "llu"

#define PRIuPTR     "u"

#define PRIx8       "x"
#define PRIx16      "x"
#define PRIx32      "lx"
#define PRIx64      "llx"

#define PRIxPTR     "x"

#define PRIX8       "X"
#define PRIX16      "X"
#define PRIX32      "lX"
#define PRIX64      "llX"

#define PRIXPTR     "X"

#define SCNd8       "hhd"
#define SCNd16      "hd"
#define SCNd32      "ld"
#define SCNd64      "lld"

#define SCNdPTR     "d"

#define SCNi8       "hhi"
#define SCNi16      "hi"
#define SCNi32      "li"
#define SCNi64      "lli"

#define SCNiPTR     "i"

#define SCNo8       "hho"
#define SCNo16      "ho"
#define SCNo32      "lo"
#define SCNo64      "llo"

#define SCNoPTR     "o"

#define SCNu8       "hhu"
#define SCNu16      "hu"
#define SCNu32      "lu"
#define SCNu64      "llu"

#define SCNuPTR     "u"

#define SCNx8       "hhx"
#define SCNx16      "hx"
#define SCNx32      "lx"
#define SCNx64      "llx"

#define SCNxPTR     "x"

#define INT8_C(x)   x
#define INT16_C(x)  x
#define INT32_C(x)  x
#define INT64_C(x)  x ## ll

#define UINT8_C(x)  x
#define UINT16_C(x) x
#define UINT32_C(x) x ## ul
#define UINT64_C(x) x ## ull

#endif /* __ARCH_ARM_INCLUDE_INTTYPES_H */
