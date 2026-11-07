/****************************************************************************
 * libs/libc/stream/lib_ultoa_invert.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2005, Dmitry Xmelkov. All rights reserved.
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

#ifndef __LIBS_LIBC_STREAM_LIB_ULTOA_INVERT_H
#define __LIBS_LIBC_STREAM_LIB_ULTOA_INVERT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Next flags are to use with `base'. Unused fields are reserved. */

#define XTOA_PREFIX  0x0100    /* Put prefix for octal or hex */
#define XTOA_UPPER   0x0200    /* Use upper case letters */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The integer that printf() can print */

#if defined(CONFIG_LIBC_PRINTF_LONG_LONG) || ULLONG_MAX == ULONG_MAX
typedef long long ultoa_int_t;
typedef unsigned long long ultoa_uint_t;
#else
typedef long ultoa_int_t;
typedef unsigned long ultoa_uint_t;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Internal function for use from `printf'. */

FAR char *__ultoa_invert(ultoa_uint_t val, FAR char *str, int base);

#endif /* __LIBS_LIBC_STREAM_LIB_ULTOA_INVERT_H */
