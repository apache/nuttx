/****************************************************************************
 * arch/x86_64/include/intel64/inttypes.h
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

#ifndef __ARCH_X86_64_INCLUDE_INTEL64_INTTYPES_H
#define __ARCH_X86_64_INCLUDE_INTEL64_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRId8       "d"
#define PRId16      "d"
#define PRId32      "d"
#define PRId64      "ld"

#define PRIdPTR     "ld"

#define PRIi8       "i"
#define PRIi16      "i"
#define PRIi32      "i"
#define PRIi64      "li"

#define PRIiPTR     "li"

#define PRIo8       "o"
#define PRIo16      "o"
#define PRIo32      "o"
#define PRIo64      "lo"

#define PRIoPTR     "lo"

#define PRIu8       "u"
#define PRIu16      "u"
#define PRIu32      "u"
#define PRIu64      "lu"

#define PRIuPTR     "lu"

#define PRIx8       "x"
#define PRIx16      "x"
#define PRIx32      "x"
#define PRIx64      "lx"

#define PRIxPTR     "lx"

#define PRIX8       "X"
#define PRIX16      "X"
#define PRIX32      "X"
#define PRIX64      "lX"

#define PRIXPTR     "lX"

#define SCNd8       "hhd"
#define SCNd16      "hd"
#define SCNd32      "d"
#define SCNd64      "ld"

#define SCNdPTR     "ld"

#define SCNi8       "hhi"
#define SCNi16      "hi"
#define SCNi32      "i"
#define SCNi64      "li"

#define SCNiPTR     "li"

#define SCNo8       "hho"
#define SCNo16      "ho"
#define SCNo32      "o"
#define SCNo64      "lo"

#define SCNoPTR     "lo"

#define SCNu8       "hhu"
#define SCNu16      "hu"
#define SCNu32      "u"
#define SCNu64      "lu"

#define SCNuPTR     "u"

#define SCNx8       "hhx"
#define SCNx16      "hx"
#define SCNx32      "x"
#define SCNx64      "lx"

#define SCNxPTR     "lx"

#define INT8_C(x)   x
#define INT16_C(x)  x
#define INT32_C(x)  x
#define INT64_C(x)  x ## l

#define UINT8_C(x)  x
#define UINT16_C(x) x
#define UINT32_C(x) x ## u
#define UINT64_C(x) x ## ul

#endif /* __ARCH_X86_64_INCLUDE_INTEL64_INTTYPES_H */
