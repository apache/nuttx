/****************************************************************************
 * arch/z16/include/inttypes.h
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

#ifndef __ARCH_Z16_INCLUDE_INTTYPES_H
#define __ARCH_Z16_INCLUDE_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRId8       "d"
#define PRId16      "d"
#define PRId32      "d"

#define PRIdLEAST8  "d"
#define PRIdLEAST16 "d"
#define PRIdLEAST32 "d"

#define PRIdFAST8   "d"
#define PRIdFAST16  "d"
#define PRIdFAST32  "d"

#define PRIdPTR     "d"

#define PRIi8       "i"
#define PRIi16      "i"
#define PRIi32      "i"

#define PRIiLEAST8  "i"
#define PRIiLEAST16 "i"
#define PRIiLEAST32 "i"

#define PRIiFAST8   "i"
#define PRIiFAST16  "i"
#define PRIiFAST32  "i"

#define PRIiPTR     "i"

#define PRIo8       "o"
#define PRIo16      "o"
#define PRIo32      "o"

#define PRIoLEAST8  "o"
#define PRIoLEAST16 "o"
#define PRIoLEAST32 "o"

#define PRIoFAST8   "o"
#define PRIoFAST16  "o"
#define PRIoFAST32  "o"

#define PRIoPTR     "o"

#define PRIu8       "u"
#define PRIu16      "u"
#define PRIu32      "u"

#define PRIuLEAST8  "u"
#define PRIuLEAST16 "u"
#define PRIuLEAST32 "u"

#define PRIuFAST8   "u"
#define PRIuFAST16  "u"
#define PRIuFAST32  "u"

#define PRIuPTR     "u"

#define PRIx8       "x"
#define PRIx16      "x"
#define PRIx32      "x"

#define PRIxLEAST8  "x"
#define PRIxLEAST16 "x"
#define PRIxLEAST32 "x"

#define PRIxFAST8   "x"
#define PRIxFAST16  "x"
#define PRIxFAST32  "x"

#define PRIxPTR     "x"

#define PRIX8       "X"
#define PRIX16      "X"
#define PRIX32      "X"

#define PRIXLEAST8  "X"
#define PRIXLEAST16 "X"
#define PRIXLEAST32 "X"

#define PRIXFAST8   "X"
#define PRIXFAST16  "X"
#define PRIXFAST32  "X"

#define PRIXPTR     "X"

#define SCNd8       "hhd"
#define SCNd16      "hd"
#define SCNd32      "d"

#define SCNdLEAST8  "hhd"
#define SCNdLEAST16 "hd"
#define SCNdLEAST32 "d"

#define SCNdFAST8   "hhd"
#define SCNdFAST16  "hd"
#define SCNdFAST32  "d"

#define SCNdPTR     "d"

#define SCNi8       "hhi"
#define SCNi16      "hi"
#define SCNi32      "i"

#define SCNiLEAST8  "hhi"
#define SCNiLEAST16 "hi"
#define SCNiLEAST32 "i"

#define SCNiFAST8   "hhi"
#define SCNiFAST16  "hi"
#define SCNiFAST32  "i"

#define SCNiPTR     "i"

#define SCNo8       "hho"
#define SCNo16      "ho"
#define SCNo32      "o"

#define SCNoLEAST8  "hho"
#define SCNoLEAST16 "ho"
#define SCNoLEAST32 "o"

#define SCNoFAST8   "hho"
#define SCNoFAST16  "ho"
#define SCNoFAST32  "o"

#define SCNoPTR     "o"

#define SCNu8       "hhu"
#define SCNu16      "hu"
#define SCNu32      "u"

#define SCNuLEAST8  "hhu"
#define SCNuLEAST16 "hu"
#define SCNuLEAST32 "u"

#define SCNuFAST8   "hhu"
#define SCNuFAST16  "hu"
#define SCNuFAST32  "u"

#define SCNuPTR     "u"

#define SCNx8       "hhx"
#define SCNx16      "hx"
#define SCNx32      "x"

#define SCNxLEAST8  "hhx"
#define SCNxLEAST16 "hx"
#define SCNxLEAST32 "x"

#define SCNxFAST8   "hhx"
#define SCNxFAST16  "hx"
#define SCNxFAST32  "x"

#define SCNxPTR     "x"

#define INT8_C(x)   x
#define INT16_C(x)  x
#define INT32_C(x)  x

#define UINT8_C(x)  x
#define UINT16_C(x) x
#define UINT32_C(x) x ## u

#endif /* __ARCH_Z16_INCLUDE_INTTYPES_H */
