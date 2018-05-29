/****************************************************************************
 * libs/libc/stdio/lib_setbuf.c
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
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT will THE
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

#include <stdio.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setbuf
 *
 * Description:
 *   Except that it returns no value, the function call setbuf(stream, buf)
 *   is equivalent to:
 *
 *     setvbuf(stream, buf, _IOFBF, BUFSIZ)
 *
 *   if buf is not a null pointer, or to:
 *
 *     setvbuf(stream, buf, _IONBF, BUFSIZ)
 *
 *   if buf is a null pointer.
 *
 * Input Parameters:
 *   stream - The stream whose buffer will be modified
 *   buffer - If non-NULL, this is user allocated buffer of size BUFSIZ. If
 *            NULL, setvbuf will disable buffering
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void setbuf(FAR FILE *stream, FAR char *buf)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  int mode;

  DEBUGASSERT(stream != NULL);

  mode = (buf != NULL) ? _IOFBF : _IONBF;
  (void)setvbuf(stream, buf, mode, BUFSIZ);

#else
  DEBUGASSERT(stream != NULL);
#endif
}
