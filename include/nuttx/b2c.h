/****************************************************************************
 * include/nuttx/b2c.h
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
 *   Author:  Xiang Xiao <xiaoxiang@xiaomi.com>
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

#ifndef __INCLUDE_NUTTX_B2C_H
#define __INCLUDE_NUTTX_B2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <string.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CHAR_BIT == 8
#  define CHAR_BYTE  1
#  define CHAR_SHIFT 0
#elif CHAR_BIT == 16
#  define CHAR_BYTE  2
#  define CHAR_SHIFT 1
#elif CHAR_BIT == 32
#  define CHAR_BYTE  4
#  define CHAR_SHIFT 2
#else
#  error unsupport CHAR_BIT value
#endif

/* Macros convert between chars and bytes */

#define B2C(x) (((x) + CHAR_BYTE - 1) >> CHAR_SHIFT)
#define C2B(x) ((x) << CHAR_SHIFT)

#define B2C_OFF(x) ((x) >> CHAR_SHIFT)
#define C2B_OFF(x) ((x) << CHAR_SHIFT)

#define B2C_REM(x) ((x) - C2B_OFF(B2C_OFF(x)))

#define B2C_SHIFT(x) ((x) - CHAR_SHIFT)
#define C2B_SHIFT(x) ((x) + CHAR_SHIFT)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Functions convert between chars and bytes */

#if CHAR_BIT != 8

size_t bstrnlen(FAR const char *src, size_t maxlen);

FAR char *ancstr2bstr(FAR const char *src, size_t maxlen);
FAR char *anbstr2cstr(FAR const char *src, size_t maxlen);

void ncstr2bstr(FAR char *dst, FAR const char *src, size_t maxlen);
void nbstr2cstr(FAR char *dst, FAR const char *src, size_t maxlen);

void cmem2bmem(FAR void *dst, size_t rem, FAR const void *src, size_t len);
void bmem2cmem(FAR void *dst, FAR const void *src, size_t rem, size_t len);

#  define cstr2bstr(dst, src) ncstr2bstr(dst, src, SIZE_MAX)
#  define bstr2cstr(dst, src) nbstr2cstr(dst, src, SIZE_MAX)

#else

#  define bstrnlen(src, maxlen) strnlen(src, maxlen)

#  define ancstr2bstr(src, maxlen) strndup(src, maxlen)
#  define anbstr2cstr(src, maxlen) strndup(src, maxlen)

#  define ncstr2bstr(dst, src, maxlen) strncpy(dst, src, maxlen)
#  define nbstr2cstr(dst, src, maxlen) strncpy(dst, src, maxlen)

#  define cmem2bmem(dst, rem, src, len) memcpy((FAR char *)(dst) + rem, src, len)
#  define bmem2cmem(dst, src, rem, len) memcpy(dst, (FAR char *)(src) + rem, len)

#  define cstr2bstr(dst, src) strcpy(dst, src)
#  define bstr2cstr(dst, src) strcpy(dst, src)

#endif

#define bstrlen(src) bstrnlen(src, SIZE_MAX)

#define acstr2bstr(src) ancstr2bstr(src, SIZE_MAX)
#define abstr2cstr(src) anbstr2cstr(src, SIZE_MAX)

#endif /* __INCLUDE_NUTTX_B2C_H */
