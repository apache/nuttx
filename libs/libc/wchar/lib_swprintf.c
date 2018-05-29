/****************************************************************************
 * libs/libc/wchar/lib_swprintf.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdarg.h>
#include <stdio.h>
#include <wchar.h>
#include "libc.h"

#include <nuttx/streams.h>

#ifdef CONFIG_LIBC_WCHAR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * swprintf
 ****************************************************************************/

int swprintf(FAR wchar_t *buf, size_t maxlen, FAR const wchar_t *fmt, ...)
{
  struct lib_memoutstream_s memoutstream;
  va_list ap;
  int n;

  /* Initialize a memory stream to write to the buffer */

  lib_memoutstream((FAR struct lib_memoutstream_s *)&memoutstream,
                   (FAR char *) buf, LIB_BUFLEN_UNKNOWN);

  /* Then let lib_vsprintf do the real work */

  va_start(ap, fmt);
  n = lib_vsprintf((FAR struct lib_outstream_s *)&memoutstream.public,
                   (FAR const char *)fmt, ap);
  va_end(ap);

  return n;
}

#endif /* CONFIG_LIBC_WCHAR */
