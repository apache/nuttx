/****************************************************************************
 * libs/libc/stdio/lib_libfflush.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/types.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_fflush
 *
 * Description:
 *  The function lib_fflush() forces a write of all user-space buffered data
 *  for the given output or update stream via the stream's underlying write
 *  function.  The open status of the stream is unaffected.
 *
 * Input Parameters:
 *  stream - the stream to flush
 *
 * Returned Value:
 *  A negated errno value on failure, otherwise the number of bytes remaining
 *  in the buffer.
 *
 ****************************************************************************/

ssize_t lib_fflush_unlocked(FAR FILE *stream)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  FAR const char *src;
  ssize_t bytes_written;
  ssize_t nbuffer;

  /* Return EBADF if the file is not opened for writing */

  if ((stream->fs_oflags & O_WROK) == 0)
    {
      return -EBADF;
    }

  /* Check if there is an allocated I/O buffer */

  if (stream->fs_bufstart == NULL)
    {
      /* No, then there can be nothing remaining in the buffer. */

      return 0;
    }

  /* Make sure that the buffer holds valid data */

  if (stream->fs_bufpos != stream->fs_bufstart)
    {
      /* Make sure that the buffer holds buffered write data.  We do not
       * support concurrent read/write buffer usage.
       */

      if (stream->fs_bufread != stream->fs_bufstart)
        {
          /* The buffer holds read data... just return zero meaning "no bytes
           * remaining in the buffer."
           */

          return 0;
        }

      /* How many bytes of write data are used in the buffer now */

      nbuffer = stream->fs_bufpos - stream->fs_bufstart;

      /* Try to write that amount */

      src = stream->fs_bufstart;
      do
        {
          /* Perform the write */

          if (stream->fs_iofunc.write != NULL)
            {
              bytes_written = stream->fs_iofunc.write(stream->fs_cookie,
                                                      src,
                                                      nbuffer);
            }
          else
            {
              bytes_written = _NX_WRITE((int)(intptr_t)stream->fs_cookie,
                                        src, nbuffer);
            }

          if (bytes_written < 0)
            {
              /* Write failed.  The cause of the failure is in 'errno'.
               * returned the negated errno value.
               */

              stream->fs_flags |= __FS_FLAG_ERROR;
              return _NX_GETERRVAL(bytes_written);
            }

          /* Handle partial writes.  fflush() must either return with
           * an error condition or with the data successfully flushed
           * from the buffer.
           */

          src     += bytes_written;
          nbuffer -= bytes_written;
        }
      while (nbuffer > 0);

      /* Reset the buffer position to the beginning of the buffer */

      stream->fs_bufpos = stream->fs_bufstart;

      /* For the case of an incomplete write, nbuffer will be non-zero
       * It will hold the number of bytes that were not written.
       * Move the data down in the buffer to handle this (rare) case
       */

      while (nbuffer)
        {
          *stream->fs_bufpos++ = *src++;
          --nbuffer;
        }
    }

  /* Restore normal access to the stream and return the number of bytes
   * remaining in the buffer.
   */

  return stream->fs_bufpos - stream->fs_bufstart;

#else
  /* Return no bytes remaining in the buffer */

  return 0;
#endif
}

ssize_t lib_fflush(FAR FILE *stream)
{
  ssize_t ret;

  /* Make sure that we have exclusive access to the stream */

  flockfile(stream);
  ret = lib_fflush_unlocked(stream);
  funlockfile(stream);
  return ret;
}
