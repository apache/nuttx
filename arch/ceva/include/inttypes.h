/****************************************************************************
 * arch/ceva/include/inttypes.h
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

#ifndef __ARCH_CEVA_INCLUDE_INTTYPES_H
#define __ARCH_CEVA_INCLUDE_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRId8       "d"
#define PRId16      "d"
#define PRId32      "d"
#define PRId64      "lld"

#define PRIdLEAST8  "d"
#define PRIdLEAST16 "d"
#define PRIdLEAST32 "d"
#define PRIdLEAST64 "lld"

#define PRIdFAST8   "d"
#define PRIdFAST16  "d"
#define PRIdFAST32  "d"
#define PRIdFAST64  "lld"

#define PRIdMAX     "lld"
#define PRIdPTR     "d"

#define PRIi8       "i"
#define PRIi16      "i"
#define PRIi32      "i"
#define PRIi64      "lli"

#define PRIiLEAST8  "i"
#define PRIiLEAST16 "i"
#define PRIiLEAST32 "i"
#define PRIiLEAST64 "lli"

#define PRIiFAST8   "i"
#define PRIiFAST16  "i"
#define PRIiFAST32  "i"
#define PRIiFAST64  "lli"

#define PRIiMAX     "lli"
#define PRIiPTR     "i"

#define PRIo8       "o"
#define PRIo16      "o"
#define PRIo32      "o"
#define PRIo64      "llo"

#define PRIoLEAST8  "o"
#define PRIoLEAST16 "o"
#define PRIoLEAST32 "o"
#define PRIoLEAST64 "llo"

#define PRIoFAST8   "o"
#define PRIoFAST16  "o"
#define PRIoFAST32  "o"
#define PRIoFAST64  "llo"

#define PRIoMAX     "llo"
#define PRIoPTR     "o"

#define PRIu8       "u"
#define PRIu16      "u"
#define PRIu32      "u"
#define PRIu64      "llu"

#define PRIuLEAST8  "u"
#define PRIuLEAST16 "u"
#define PRIuLEAST32 "u"
#define PRIuLEAST64 "llu"

#define PRIuFAST8   "u"
#define PRIuFAST16  "u"
#define PRIuFAST32  "u"
#define PRIuFAST64  "llu"

#define PRIuMAX     "llu"
#define PRIuPTR     "u"

#define PRIx8       "x"
#define PRIx16      "x"
#define PRIx32      "x"
#define PRIx64      "llx"

#define PRIxLEAST8  "x"
#define PRIxLEAST16 "x"
#define PRIxLEAST32 "x"
#define PRIxLEAST64 "llx"

#define PRIxFAST8   "x"
#define PRIxFAST16  "x"
#define PRIxFAST32  "x"
#define PRIxFAST64  "llx"

#define PRIxMAX     "llx"
#define PRIxPTR     "x"

#define PRIX8       "X"
#define PRIX16      "X"
#define PRIX32      "X"
#define PRIX64      "llX"

#define PRIXLEAST8  "X"
#define PRIXLEAST16 "X"
#define PRIXLEAST32 "X"
#define PRIXLEAST64 "llX"

#define PRIXFAST8   "X"
#define PRIXFAST16  "X"
#define PRIXFAST32  "X"
#define PRIXFAST64  "llX"

#define PRIXMAX     "llX"
#define PRIXPTR     "X"

#define SCNd8       "hhd"
#define SCNd16      "hd"
#define SCNd32      "d"
#define SCNd64      "lld"

#define SCNdLEAST8  "hhd"
#define SCNdLEAST16 "hd"
#define SCNdLEAST32 "d"
#define SCNdLEAST64 "lld"

#define SCNdFAST8   "hhd"
#define SCNdFAST16  "hd"
#define SCNdFAST32  "d"
#define SCNdFAST64  "lld"

#define SCNdMAX     "lld"
#define SCNdPTR     "d"

#define SCNi8       "hhi"
#define SCNi16      "hi"
#define SCNi32      "i"
#define SCNi64      "lli"

#define SCNiLEAST8  "hhi"
#define SCNiLEAST16 "hi"
#define SCNiLEAST32 "i"
#define SCNiLEAST64 "lli"

#define SCNiFAST8   "hhi"
#define SCNiFAST16  "hi"
#define SCNiFAST32  "i"
#define SCNiFAST64  "lli"

#define SCNiMAX     "lli"
#define SCNiPTR     "i"

#define SCNo8       "hho"
#define SCNo16      "ho"
#define SCNo32      "o"
#define SCNo64      "llo"

#define SCNoLEAST8  "hho"
#define SCNoLEAST16 "ho"
#define SCNoLEAST32 "o"
#define SCNoLEAST64 "llo"

#define SCNoFAST8   "hho"
#define SCNoFAST16  "ho"
#define SCNoFAST32  "o"
#define SCNoFAST64  "llo"

#define SCNoMAX     "llo"
#define SCNoPTR     "o"

#define SCNu8       "hhu"
#define SCNu16      "hu"
#define SCNu32      "u"
#define SCNu64      "llu"

#define SCNuLEAST8  "hhu"
#define SCNuLEAST16 "hu"
#define SCNuLEAST32 "u"
#define SCNuLEAST64 "llu"

#define SCNuFAST8   "hhu"
#define SCNuFAST16  "hu"
#define SCNuFAST32  "u"
#define SCNuFAST64  "llu"

#define SCNuMAX     "llu"
#define SCNuPTR     "u"

#define SCNx8       "hhx"
#define SCNx16      "hx"
#define SCNx32      "x"
#define SCNx64      "llx"

#define SCNxLEAST8  "hhx"
#define SCNxLEAST16 "hx"
#define SCNxLEAST32 "x"
#define SCNxLEAST64 "llx"

#define SCNxFAST8   "hhx"
#define SCNxFAST16  "hx"
#define SCNxFAST32  "x"
#define SCNxFAST64  "llx"

#define SCNxMAX     "llx"
#define SCNxPTR     "x"

#endif /* __ARCH_CEVA_INCLUDE_INTTYPES_H */
