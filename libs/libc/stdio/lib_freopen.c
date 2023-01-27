/****************************************************************************
 * libs/libc/stdio/lib_freopen.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
 *   successfully closed or not, freopen opens the file specified by path
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
      /* Yes, open the file directly if no stream is opened yet */

      if (stream == NULL)
        {
          return fopen(path, mode);
        }

      /* Otherwise, open the file */

      oflags = lib_mode2oflags(mode);
      if (oflags < 0)
        {
          return NULL;
        }

      fd = open(path, oflags, 0666);
      if (fd < 0)
        {
          return NULL;
        }

      /* Flush the stream and invalidate the read buffer. */

      fflush(stream);

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
      lib_rdflush(stream);
#endif

      /* Duplicate the new fd to the stream. */

      ret = dup2(fd, fileno(stream));
      close(fd);
      if (ret < 0)
        {
          return NULL;
        }

      clearerr(stream);
      return stream;
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
