/****************************************************************************
 * arch/risc-v/include/inttypes.h
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

#ifndef __ARCH_RISCV_INCLUDE_INTTYPES_H
#define __ARCH_RISCV_INCLUDE_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(__LP64__)
#define _PRI32PREFIX
#define _PRI64PREFIX "l"
#define _PRIPTRPREFIX "l"
#define _SCN32PREFIX
#define _SCN64PREFIX "l"
#define _SCNPTRPREFIX "l"
#define INT32_C(x)  x
#define INT64_C(x)  x ## l
#define UINT32_C(x) x ## u
#define UINT64_C(x) x ## ul
#else /* defined(__LP64__) */
#define _PRI32PREFIX "l"
#define _PRI64PREFIX "ll"
#define _PRIPTRPREFIX
#define _SCN32PREFIX "l"
#define _SCN64PREFIX "ll"
#define _SCNPTRPREFIX
#define INT32_C(x)  x ## l
#define INT64_C(x)  x ## ll
#define UINT32_C(x) x ## ul
#define UINT64_C(x) x ## ull
#endif /* defined(__LP64__) */

#define PRId8       "d"
#define PRId16      "d"
#define PRId32      _PRI32PREFIX "d"
#define PRId64      _PRI64PREFIX "d"

#define PRIdPTR     _PRIPTRPREFIX "d"

#define PRIi8       "i"
#define PRIi16      "i"
#define PRIi32      _PRI32PREFIX "i"
#define PRIi64      _PRI64PREFIX "i"

#define PRIiPTR     _PRIPTRPREFIX "i"

#define PRIo8       "o"
#define PRIo16      "o"
#define PRIo32      _PRI32PREFIX "o"
#define PRIo64      _PRI64PREFIX "o"

#define PRIoPTR     _PRIPTRPREFIX "o"

#define PRIu8       "u"
#define PRIu16      "u"
#define PRIu32      _PRI32PREFIX "u"
#define PRIu64      _PRI64PREFIX "u"

#define PRIuPTR     _PRIPTRPREFIX "u"

#define PRIx8       "x"
#define PRIx16      "x"
#define PRIx32      _PRI32PREFIX "x"
#define PRIx64      _PRI64PREFIX "x"

#define PRIxPTR     _PRIPTRPREFIX "x"

#define PRIX8       "X"
#define PRIX16      "X"
#define PRIX32      _PRI32PREFIX "X"
#define PRIX64      _PRI64PREFIX "X"

#define PRIXPTR     _PRIPTRPREFIX "X"

#define SCNd8       "hhd"
#define SCNd16      "hd"
#define SCNd32      _SCN32PREFIX "d"
#define SCNd64      _SCN64PREFIX "d"

#define SCNdPTR     _SCNPTRPREFIX "d"

#define SCNi8       "hhi"
#define SCNi16      "hi"
#define SCNi32      _SCN32PREFIX "i"
#define SCNi64      _SCN64PREFIX "i"

#define SCNiPTR     _SCNPTRPREFIX "i"

#define SCNo8       "hho"
#define SCNo16      "ho"
#define SCNo32      _SCN32PREFIX "o"
#define SCNo64      _SCN64PREFIX "o"

#define SCNoPTR     _SCNPTRPREFIX "o"

#define SCNu8       "hhu"
#define SCNu16      "hu"
#define SCNu32      _SCN32PREFIX "u"
#define SCNu64      _SCN64PREFIX "u"

#define SCNuPTR     _SCNPTRPREFIX "u"

#define SCNx8       "hhx"
#define SCNx16      "hx"
#define SCNx32      _SCN32PREFIX "x"
#define SCNx64      _SCN64PREFIX "x"

#define SCNxPTR     _SCNPTRPREFIX "x"

#define INT8_C(x)   x
#define INT16_C(x)  x

#define UINT8_C(x)  x
#define UINT16_C(x) x

#endif /* __ARCH_RISCV_INCLUDE_INTTYPES_H */
