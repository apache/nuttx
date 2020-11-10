/****************************************************************************
 * arch/sim/include/inttypes.h
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

#ifndef __ARCH_SIM_INCLUDE_INTTYPES_H
#define __ARCH_SIM_INCLUDE_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#  define PRId8       "d"
#  define PRId16      "d"
#  define PRId32      "d"
#  define PRId64      "lld"

#  define PRIi8       "i"
#  define PRIi16      "i"
#  define PRIi32      "i"
#  define PRIi64      "lli"

#  define PRIo8       "o"
#  define PRIo16      "o"
#  define PRIo32      "o"
#  define PRIo64      "llo"

#  define PRIu8       "u"
#  define PRIu16      "u"
#  define PRIu32      "u"
#  define PRIu64      "llu"

#  define PRIuMAX     "llu"

#  define PRIx8       "x"
#  define PRIx16      "x"
#  define PRIx32      "x"
#  define PRIx64      "llx"

#  define PRIX8       "X"
#  define PRIX16      "X"
#  define PRIX32      "X"
#  define PRIX64      "llX"

#  define SCNd8       "hhd"
#  define SCNd16      "hd"
#  define SCNd32      "d"
#  define SCNd64      "lld"

#  define SCNdMAX     "lld"

#  define SCNi8       "hhi"
#  define SCNi16      "hi"
#  define SCNi32      "i"
#  define SCNi64      "lli"

#  define SCNiMAX     "lli"

#  define SCNo8       "hho"
#  define SCNo16      "ho"
#  define SCNo32      "o"
#  define SCNo64      "llo"

#  define SCNoMAX     "llo"

#  define SCNu8       "hhu"
#  define SCNu16      "hu"
#  define SCNu32      "u"
#  define SCNu64      "llu"

#  define SCNx8       "hhx"
#  define SCNx16      "hx"
#  define SCNx32      "x"
#  define SCNx64      "llx"

#  define INT8_C(x)   x
#  define INT16_C(x)  x
#  define INT32_C(x)  x
#  define INT64_C(x)  x ## ll

#  define UINT8_C(x)  x
#  define UINT16_C(x) x
#  define UINT32_C(x) x ## u
#  define UINT64_C(x) x ## ull

#if defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)
#  define PRIdPTR     "lld"
#  define PRIiPTR     "lli"
#  define PRIoPTR     "llo"
#  define PRIuPTR     "llu"
#  define PRIxPTR     "llx"
#  define PRIXPTR     "llX"
#  define SCNdPTR     "lld"
#  define SCNiPTR     "lli"
#  define SCNoPTR     "llo"
#  define SCNuPTR     "llu"
#  define SCNxPTR     "llx"
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
