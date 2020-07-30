/****************************************************************************
 * libs/libc/stdio/lib_libfread.c
 *
 *   Copyright (C) 2007-2009, 2011-2014, 2017 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_fread
 ****************************************************************************/

ssize_t lib_fread(FAR void *ptr, size_t count, FAR FILE *stream)
{
  FAR unsigned char *dest  = (FAR unsigned char*)ptr;
  ssize_t bytes_read;
  size_t remaining = count;
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  int ret;
#endif

  /* Make sure that reading from this stream is allowed */

  if (!stream || (stream->fs_oflags & O_RDOK) == 0)
    {
      set_errno(EBADF);
    }
  else
    {
      /* The stream must be stable until we complete the read */

      lib_take_semaphore(stream);

#if CONFIG_NUNGET_CHARS > 0
      /* First, re-read any previously ungotten characters */

      while (stream->fs_nungotten > 0 && remaining > 0)
        {
          /* Decrement the count of ungotten bytes to get an index */

          stream->fs_nungotten--;

          /* Return the last ungotten byte */

          *dest++ = stream->fs_ungotten[stream->fs_nungotten];

          /* That's one less byte that we have to read */

          remaining--;
        }
#endif

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
      /* Is there an I/O buffer? */

      if (stream->fs_bufstart != NULL)
        {
          /* If the buffer is currently being used for write access, then
           * flush all of the buffered write data.  We do not support concurrent
           * buffered read/write access.
           */

          ret = lib_wrflush(stream);
          if (ret < 0)
            {
              lib_give_semaphore(stream);
              return ret;
            }

          /* Now get any other needed chars from the buffer or the file. */

          while (remaining > 0)
            {
              /* Is there readable data in the buffer? */

              while (remaining > 0 && stream->fs_bufpos < stream->fs_bufread)
                {
                  /* Yes, copy a byte into the user buffer */

                  *dest++ = *stream->fs_bufpos++;
                  remaining--;
                }

              /* The buffer is empty OR we have already supplied the number of
               * bytes requested in the read.  Check if we need to read
               * more from the file.
               */

              if (remaining > 0)
                {
                  size_t buffer_available;

                  /* We need to read more data into the buffer from the file */

                  /* Mark the buffer empty */

                  stream->fs_bufpos = stream->fs_bufread = stream->fs_bufstart;

                  /* How much space is available in the buffer? */

                  buffer_available = stream->fs_bufend - stream->fs_bufread;

                  /* Will the number of bytes that we need to read fit into
                   * the buffer space that is available? If the read size is
                   * larger than the buffer, then read some of the data
                   * directly into the user's buffer.
                   */

                  if (remaining > buffer_available)
                    {
                      bytes_read = _NX_READ(stream->fs_fd, dest, remaining);
                      if (bytes_read < 0)
                        {
                          /* An error occurred on the read. */

                          _NX_SETERRNO(bytes_read);
                          goto errout_with_errno;
                        }
                      else if (bytes_read == 0)
                        {
                          /* We are at the end of the file.  But we may already
                           * have buffered data.  In that case, we will report
                           * the EOF indication later.
                           */

                          goto shortread;
                        }
                      else
                        {
                          /* Some (perhaps all) bytes were read. Adjust the dest
                           * pointer and remaining bytes to be read.
                           */

                          DEBUGASSERT(bytes_read <= remaining);
                          dest      += bytes_read;
                          remaining -= bytes_read;
                        }
                    }
                  else
                    {
                      /* The number of bytes required to satisfy the read
                       * is less than or equal to the size of the buffer
                       * space that we have left. Read as much as we can
                       * into the buffer.
                       */

                      bytes_read = _NX_READ(stream->fs_fd,
                                            stream->fs_bufread,
                                            buffer_available);
                      if (bytes_read < 0)
                        {
                          /* An error occurred on the read.  The error code is
                           * in the 'errno' variable.
                           */

                          _NX_SETERRNO(bytes_read);
                          goto errout_with_errno;
                        }
                      else if (bytes_read == 0)
                        {
                          /* We are at the end of the file.  But we may already
                           * have buffered data.  In that case, we will report
                           * the EOF indication later.
                           */

                          goto shortread;
                        }
                      else
                        {
                          /* Some (perhaps all) bytes were read */

                          stream->fs_bufread += bytes_read;
                        }
                    }
                }
            }
        }
      else
#endif
        {
          /* Now get any other needed chars from the file. */

          while (remaining > 0)
            {
              bytes_read = _NX_READ(stream->fs_fd, dest, remaining);
              if (bytes_read < 0)
                {
                  /* An error occurred on the read.  The error code is
                   * in the 'errno' variable.
                   */

                  _NX_SETERRNO(bytes_read);
                  goto errout_with_errno;
                }
              else if (bytes_read == 0)
                {
                  /* We are at the end of the file.  But we may already
                   * have buffered data.  In that case, we will report
                   * the EOF indication later.
                   */

                  break;
                }
              else
                {
                  DEBUGASSERT(bytes_read <= remaining);
                  dest      += bytes_read;
                  remaining -= bytes_read;
                }
            }
        }

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
      /* Here after a successful (but perhaps short) read.  A short read can
       * only occur is read() returns 0 (end-of-file).
       */

shortread:
#endif

      /* Set or clear the EOF indicator.  If we get here because of a
       * short read and the total number of bytes read is zero, then
       * we must be at the end-of-file.
       */

      if (remaining == 0)
        {
          stream->fs_flags &= ~__FS_FLAG_EOF;
        }
      else
        {
          stream->fs_flags |= __FS_FLAG_EOF;
        }

      lib_give_semaphore(stream);
    }

  return count - remaining;

/* Error exits */

errout_with_errno:
  stream->fs_flags |= __FS_FLAG_ERROR;
  lib_give_semaphore(stream);
  return -get_errno();
}
