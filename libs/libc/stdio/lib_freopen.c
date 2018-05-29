/****************************************************************************
 * libs/libc/stdio/lib_freopen.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: freopen
 *
 * Description:
 *   Reuses stream to either open the file specified by path or to change
 *   its access mode.
 *
 *   If a new path is specified, the function first attempts to close
 *   any file already associated with stream (third parameter) and
 *   disassociates it. Then, independently of whether that stream was
 *   successfuly closed or not, freopen opens the file specified by path
 *   and associates it with the stream just as fopen would do using the
 *   specified mode.
 *
 *   If path is a null pointer, the function attempts to change the mode
 *   of the stream. Although a particular library implementation is allowed
 *   to restrict the changes permitted, and under which circumstances.
 *
 *   The error indicator and eof indicator are automatically cleared (as if
 *   clearerr was called).
 *
 * Input Parameters:
 *   path   - If non-NULL, refers to the name of the file to be opened.
 *   mode   - String describing the new file access mode
 *   stream - Pointer to the type FILE to be reopened.
 *
 * Returned Value:
 *   If the file is successfully reopened, the function returns the pointer
 *   passed as parameter stream, which can be used to identify the reopened
 *   stream.   Otherwise, a null pointer is returned and the errno variable
 *   is also set to a system-specific error code on failure.
 *
 ****************************************************************************/

FAR FILE *freopen(FAR const char *path, FAR const char *mode,
                  FAR FILE *stream)
{
  int oflags;
  int ret;
  int fd;

  /* Was a file name provided? */

  if (path != NULL)
    {
      /* Yes, close the stream */

      if (stream)
        {
          (void)fclose(stream);
        }

      /* And attempt to reopen the file at the provided path */

      return fopen(path, mode);
    }

  /* Otherwise, we are just changing the mode of the current file. */

  if (stream)
    {
      /* Convert the mode string into standard file open mode flags. */

      oflags = lib_mode2oflags(mode);
      if (oflags < 0)
        {
          return NULL;
        }

      /* Get the underlying file descriptor from the stream */

      fd = fileno(stream);
      if (fd < 0)
        {
          return NULL;
        }

      /* Set the new file mode for the file descriptor */

      ret = fcntl(fd, F_SETFL, oflags);
      if (ret < 0)
        {
          return NULL;
        }

      clearerr(stream);
      return stream;
    }

  set_errno(EINVAL);
  return NULL;
}
