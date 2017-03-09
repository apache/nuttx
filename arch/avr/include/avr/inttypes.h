/****************************************************************************
 * arch/avr/include/avr/inttypes.h
 *
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_AVR_INCLUDE_AVR_INTTYPES_H
#define __ARCH_AVR_INCLUDE_AVR_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRId8       "d"
#define PRId16      "d"
#define PRId32      "ld"
#define PRId64      "lld"

#define PRIdLEAST8  "d"
#define PRIdLEAST16 "d"
#define PRIdLEAST32 "ld"
#define PRIdLEAST64 "lld"

#define PRIdFAST8   "d"
#define PRIdFAST16  "d"
#define PRIdFAST32  "ld"
#define PRIdFAST64  "lld"

#define PRIdMAX     "lld"
#define PRIdPTR     "d"

#define PRIi8       "i"
#define PRIi16      "i"
#define PRIi32      "li"
#define PRIi64      "lli"

#define PRIiLEAST8  "i"
#define PRIiLEAST16 "i"
#define PRIiLEAST32 "li"
#define PRIiLEAST64 "lli"

#define PRIiFAST8   "i"
#define PRIiFAST16  "i"
#define PRIiFAST32  "li"
#define PRIiFAST64  "lli"

#define PRIiMAX     "lli"
#define PRIiPTR     "i"

#define PRIo8       "o"
#define PRIo16      "o"
#define PRIo32      "lo"
#define PRIo64      "llo"

#define PRIoLEAST8  "o"
#define PRIoLEAST16 "o"
#define PRIoLEAST32 "lo"
#define PRIoLEAST64 "llo"

#define PRIoFAST8   "o"
#define PRIoFAST16  "o"
#define PRIoFAST32  "lo"
#define PRIoFAST64  "llo"

#define PRIoMAX     "llo"
#define PRIoPTR     "o"

#define PRIu8       "u"
#define PRIu16      "u"
#define PRIu32      "lu"
#define PRIu64      "llu"

#define PRIuLEAST8  "u"
#define PRIuLEAST16 "u"
#define PRIuLEAST32 "lu"
#define PRIuLEAST64 "llu"

#define PRIuFAST8   "u"
#define PRIuFAST16  "u"
#define PRIuFAST32  "lu"
#define PRIuFAST64  "llu"

#define PRIuMAX     "llu"
#define PRIuPTR     "u"

#define PRIx8       "x"
#define PRIx16      "x"
#define PRIx32      "lx"
#define PRIx64      "llx"

#define PRIxLEAST8  "x"
#define PRIxLEAST16 "x"
#define PRIxLEAST32 "lx"
#define PRIxLEAST64 "llx"

#define PRIxFAST8   "x"
#define PRIxFAST16  "x"
#define PRIxFAST32  "lx"
#define PRIxFAST64  "llx"

#define PRIxMAX     "llx"
#define PRIxPTR     "x"

#define PRIX8       "X"
#define PRIX16      "X"
#define PRIX32      "lX"
#define PRIX64      "llX"

#define PRIXLEAST8  "X"
#define PRIXLEAST16 "X"
#define PRIXLEAST32 "lX"
#define PRIXLEAST64 "llX"

#define PRIXFAST8   "X"
#define PRIXFAST16  "X"
#define PRIXFAST32  "lX"
#define PRIXFAST64  "llX"

#define PRIXMAX     "llX"
#define PRIXPTR     "X"

#define SCNd8       "hhd"
#define SCNd16      "d"
#define SCNd32      "ld"
#define SCNd64      "lld"

#define SCNdLEAST8  "hhd"
#define SCNdLEAST16 "d"
#define SCNdLEAST32 "ld"
#define SCNdLEAST64 "lld"

#define SCNdFAST8   "hhd"
#define SCNdFAST16  "d"
#define SCNdFAST32  "ld"
#define SCNdFAST64  "lld"

#define SCNdMAX     "lld"
#define SCNdPTR     "d"

#define SCNi8       "hhi"
#define SCNi16      "i"
#define SCNi32      "li"
#define SCNi64      "lli"

#define SCNiLEAST8  "hhi"
#define SCNiLEAST16 "i"
#define SCNiLEAST32 "li"
#define SCNiLEAST64 "lli"

#define SCNiFAST8   "hhi"
#define SCNiFAST16  "i"
#define SCNiFAST32  "li"
#define SCNiFAST64  "lli"

#define SCNiMAX     "lli"
#define SCNiPTR     "i"

#define SCNo8       "hho"
#define SCNo16      "o"
#define SCNo32      "lo"
#define SCNo64      "llo"

#define SCNoLEAST8  "hho"
#define SCNoLEAST16 "o"
#define SCNoLEAST32 "lo"
#define SCNoLEAST64 "llo"

#define SCNoFAST8   "hho"
#define SCNoFAST16  "o"
#define SCNoFAST32  "lo"
#define SCNoFAST64  "llo"

#define SCNoMAX     "llo"
#define SCNoPTR     "o"

#define SCNu8       "hhu"
#define SCNu16      "u"
#define SCNu32      "lu"
#define SCNu64      "llu"

#define SCNuLEAST8  "hhu"
#define SCNuLEAST16 "u"
#define SCNuLEAST32 "lu"
#define SCNuLEAST64 "llu"

#define SCNuFAST8   "hhu"
#define SCNuFAST16  "u"
#define SCNuFAST32  "lu"
#define SCNuFAST64  "llu"

#define SCNuMAX     "llu"
#define SCNuPTR     "u"

#define SCNx8       "hhx"
#define SCNx16      "x"
#define SCNx32      "lx"
#define SCNx64      "llx"

#define SCNxLEAST8  "hhx"
#define SCNxLEAST16 "x"
#define SCNxLEAST32 "lx"
#define SCNxLEAST64 "llx"

#define SCNxFAST8   "hhx"
#define SCNxFAST16  "x"
#define SCNxFAST32  "lx"
#define SCNxFAST64  "llx"

#define SCNxMAX     "llx"
#define SCNxPTR     "x"

#endif /* __ARCH_AVR_INCLUDE_AVR_INTTYPES_H */
