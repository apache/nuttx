/****************************************************************************
 * libs/libc/stdio/lib_libfwrite.c
 *
 *   Copyright (C) 2007-2009, 2011, 2013-2014, 2017 Gregory Nutt. All rights
 *   reserved.
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
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_fwrite
 ****************************************************************************/

ssize_t lib_fwrite(FAR const void *ptr, size_t count, FAR FILE *stream)
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
{
  FAR const unsigned char *start = ptr;
  FAR const unsigned char *src   = ptr;
  ssize_t ret = ERROR;
  unsigned char *dest;

  /* Make sure that writing to this stream is allowed */

  if (stream == NULL)
    {
      set_errno(EBADF);
      return ret;
    }

  /* Check if write access is permitted */

  if ((stream->fs_oflags & O_WROK) == 0)
    {
      set_errno(EBADF);
      goto errout;
    }

  /* If there is no I/O buffer, then output data immediately */

  if (stream->fs_bufstart == NULL)
    {
      ret = _NX_WRITE(stream->fs_fd, ptr, count);
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
      if (ret < 0)
        {
          _NX_SETERRNO(ret);
          ret = ERROR;
        }
#endif

      goto errout;
    }

  /* Get exclusive access to the stream */

  lib_take_semaphore(stream);

  /* If the buffer is currently being used for read access, then
   * discard all of the read-ahead data.  We do not support concurrent
   * buffered read/write access.
   */

  if (lib_rdflush(stream) < 0)
    {
      goto errout_with_semaphore;
    }

  /* Loop until all of the bytes have been buffered */

  while (count > 0)
    {
      /* Determine the number of bytes left in the buffer */

      size_t gulp_size = stream->fs_bufend - stream->fs_bufpos;

      /* Will the user data fit into the amount of buffer space
       * that we have left?
       */

      if (gulp_size > count)
        {
          /* Yes, clip the gulp to the size of the user data */

          gulp_size = count;
        }

      /* Adjust the number of bytes remaining to be transferred
       * on the next pass through the loop (might be zero).
       */

      count -= gulp_size;

      /* Transfer the data into the buffer */

      for (dest = stream->fs_bufpos; gulp_size > 0; gulp_size--)
        {
          *dest++ = *src++;
        }

      stream->fs_bufpos = dest;

      /* Is the buffer full? */

      if (dest >= stream->fs_bufend)
        {
          /* Flush the buffered data to the IO stream */

          int bytes_buffered = lib_fflush(stream, false);
          if (bytes_buffered < 0)
            {
              goto errout_with_semaphore;
            }
        }
    }

  /* Return the number of bytes written */

  ret = (uintptr_t)src - (uintptr_t)start;

errout_with_semaphore:
  lib_give_semaphore(stream);

errout:
  if (ret < 0)
    {
      stream->fs_flags |= __FS_FLAG_ERROR;
    }

  return ret;
}
#else
{
  ssize_t ret = _NX_WRITE(stream->fs_fd, ptr, count);
  if (ret < 0)
    {
      stream->fs_flags |= __FS_FLAG_ERROR;
      _NX_SETERRNO(ret);
    }

  return ret;
}
#endif /* CONFIG_STDIO_DISABLE_BUFFERING */
