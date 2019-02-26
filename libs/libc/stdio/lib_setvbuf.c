/****************************************************************************
 * libs/libc/stdio/lib_setvbuf.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *           Gregory Nutt <gnutt@nuttx.org>
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
 * Name: setvbuf
 *
 * Description:
 *   The setvbuf() function may be used after the stream pointed to by
 *   stream is associated with an open file but before any other operation
 *   (other than an unsuccessful call to setvbuf()) is performed on the
 *   stream. The argument type determines how stream will be buffered, as
 *   follows:
 *
 *   _IOFBF - Will cause input/output to be fully buffered.
 *   _IOLBF - Will cause input/output to be line buffered.
 *   _IONBF - Will cause input/output to be unbuffered.
 *
 *  If buf is not a null pointer, the array it points to may be used instead
 *  of a buffer allocated by setvbuf() and the argument size specifies the size
 *  of the array; otherwise, size may determine the size of a buffer allocated
 *  by the setvbuf() function. The contents of the array at any time are
 *  unspecified.
 *
 * Input Parameters:
 *   stream - the stream to flush
 *   buffer - the user allocate buffer. If NULL, will allocates a buffer of
 *            specified size
 *   mode   - specifies a mode for file buffering
 *   size   - size of buffer
 *
 * Returned Value:
 *   Upon successful completion, setvbuf() will return 0. Otherwise, it will
 *   return a non-zero value if an invalid value is given for type or if the
 *   request cannot be honored, [CX] and may set errno to indicate the error
 *
 ****************************************************************************/

int setvbuf(FAR FILE *stream, FAR char *buffer, int mode, size_t size)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  FAR unsigned char *newbuf = NULL;
  uint8_t flags;
  int errcode;

  /* Verify arguments */
  /* Make sure that a valid mode was provided */

  if (mode != _IOFBF && mode != _IOLBF && mode != _IONBF)
    {
      errcode = EINVAL;
      goto errout;
    }

  /* If a buffer pointer is provided, then it must have a non-zero size */

  if (buffer != NULL && size == 0)
    {
      errcode = EINVAL;
      goto errout;
    }

  /* My assumption is that if size is zero for modes {_IOFBF, _IOLBF} the
   * caller is only attempting to change the buffering mode.  In this case,
   * the existing buffer should be re-used (if there is one).  If there is no
   * existing buffer, then I suppose we should allocate one of the default
   * size?
   */

  if ((mode == _IOFBF || mode == _IOLBF) && size == 0 &&
      stream->fs_bufstart == NULL)
    {
      size = BUFSIZ;
    }

  /* A non-zero size (or a non-NULL buffer) with mode = _IONBF makes no
   * sense but is, apparently, permissible. We simply force the buffer to
   * NULL and size to zero in this case without complaining.
   */

  else if (mode == _IONBF)
    {
      buffer = NULL;
      size   = 0;
    }

  /* Make sure that we have exclusive access to the stream */

  lib_take_semaphore(stream);

  /* setvbuf() may only be called AFTER the stream has been opened and
   * BEFORE any operations have been performed on the stream.
   */

  /* Return EBADF if the file is not open */

  if (stream->fs_fd < 0)
    {
      errcode = EBADF;
      goto errout_with_semaphore;
    }

  /* Return EBUSY if operations have already been performed on the buffer.
   * Here we really only verify that there is no valid data in the existing
   * buffer.
   *
   * REVIST:  There could be race conditions here, could there not?
   */

  if (stream->fs_bufpos != stream->fs_bufstart)
    {
      errcode = EBUSY;
      goto errout_with_semaphore;
    }

  /* Initialize by clearing related flags.  We try to avoid any permanent
   * changes to the stream structure until we know that we will be
   * successful.
   */

  flags = stream->fs_flags & ~(__FS_FLAG_LBF | __FS_FLAG_UBF);

  /* Allocate a new buffer if one is needed or reuse the existing buffer it
   * is appropriate to do so.
   */

  switch (mode)
    {
      case _IOLBF:
        flags |= __FS_FLAG_LBF;

        /* Fall through */

      case _IOFBF:
        /* Use the existing buffer if size == 0 */

        if (size > 0)
          {
            /* A new buffer is needed.  Did the caller provide the buffer
             * memory?
             */

            if (buffer != NULL)
              {
                newbuf = (FAR unsigned char *)buffer;

                /* Indicate that we have an I/O buffer managed by the caller of
                 * setvbuf.
                 */

                flags |= __FS_FLAG_UBF;
              }
            else
              {
                newbuf = (FAR unsigned char *)lib_malloc(size);
                if (newbuf == NULL)
                  {
                    errcode = ENOMEM;
                    goto errout_with_semaphore;
                  }
              }
          }
        else
          {
            /* Re-use the existing buffer and retain some existing flags.
             * This supports changing the buffering mode without changing
             * the buffer.
             */

            DEBUGASSERT(stream->fs_bufstart != NULL);
            flags |= stream->fs_flags & __FS_FLAG_UBF;
            goto reuse_buffer;
          }

        break;

      case _IONBF:
        /* No buffer needed... We must be performing unbuffered I/O */

        newbuf = NULL;
        break;

      default:
        DEBUGPANIC();
    }

   /* Do not release the previous buffer if it was allocated by the user
    * on a previous call to setvbuf().
    */

   if (stream->fs_bufstart != NULL &&
       (stream->fs_flags & __FS_FLAG_UBF) == 0)
    {
      lib_free(stream->fs_bufstart);
    }

  /* Set the new buffer information */

  stream->fs_bufstart = newbuf;
  stream->fs_bufpos   = newbuf;
  stream->fs_bufread  = newbuf;
  stream->fs_bufend   = newbuf + size;

  /* Update the stream flags and return success */

reuse_buffer:

  stream->fs_flags    = flags;
  lib_give_semaphore(stream);
  return OK;

errout_with_semaphore:
  lib_give_semaphore(stream);

errout:
  set_errno(errcode);
  return ERROR;

#else
  /* Return that setvbuf() is not supported */

  set_errno(ENOSYS);
  return ERROR;
#endif
}
