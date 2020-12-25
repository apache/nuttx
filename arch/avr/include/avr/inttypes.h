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
#define SCNd16      "d"
#define SCNd32      "ld"
#define SCNd64      "lld"

#define SCNdPTR     "d"

#define SCNi8       "hhi"
#define SCNi16      "i"
#define SCNi32      "li"
#define SCNi64      "lli"

#define SCNiPTR     "i"

#define SCNo8       "hho"
#define SCNo16      "o"
#define SCNo32      "lo"
#define SCNo64      "llo"

#define SCNoPTR     "o"

#define SCNu8       "hhu"
#define SCNu16      "u"
#define SCNu32      "lu"
#define SCNu64      "llu"

#define SCNuPTR     "u"

#define SCNx8       "hhx"
#define SCNx16      "x"
#define SCNx32      "lx"
#define SCNx64      "llx"

#define SCNxPTR     "x"

#define INT8_C(x)   x
#define INT16_C(x)  x
#define INT32_C(x)  x ## l
#define INT64_C(x)  x ## ll

#define UINT8_C(x)  x
#define UINT16_C(x) x
#define UINT32_C(x) x ## ul
#define UINT64_C(x) x ## ull

#endif /* __ARCH_AVR_INCLUDE_AVR_INTTYPES_H */
