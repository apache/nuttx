/****************************************************************************
 * libs/libc/stdio/lib_fclose.c
 *
 *   Copyright (C) 2007-2009, 2011, 2013, 2017 Gregory Nutt.
 *   All rights reserved.
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

#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fclose
 *
 * Description:
 *   The fclose() function will flush the stream pointed to by stream
 *   (writing any buffered output data using lib_fflush()) and close the
 *   underlying file descriptor.
 *
 * Returned Value:
 *   Upon successful completion 0 is returned. Otherwise, EOF is returned
 *   and the global variable errno is set to indicate the error. In either
 *   case any further access (including another call to fclose()) to the
 *   stream results in undefined behaviour.
 *
 ****************************************************************************/

int fclose(FAR FILE *stream)
{
  FAR struct streamlist *slist;
  FAR FILE *prev = NULL;
  FAR FILE *next;
  int errcode = EINVAL;
  int ret = ERROR;
  int status;

  /* Verify that a stream was provided. */

  if (stream)
    {
      /* Remove FILE structure from the stream list */

      slist = nxsched_get_streams();
      stream_semtake(slist);

      for (next = slist->sl_head; next; prev = next, next = next->fs_next)
        {
          if (next == stream)
            {
              if (next == slist->sl_head)
                {
                  slist->sl_head = next->fs_next;
                }
              else
                {
                  prev->fs_next = next->fs_next;
                }

              if (next == slist->sl_tail)
                {
                  slist->sl_tail = prev;
                }

              break;
            }
        }

      stream_semgive(slist);

      /* Check that the underlying file descriptor corresponds to an an open
       * file.
       */

      ret = OK;
      if (stream->fs_fd >= 0)
        {
          /* If the stream was opened for writing, then flush the stream */

          if ((stream->fs_oflags & O_WROK) != 0)
            {
              ret = lib_fflush(stream, true);
              errcode = get_errno();
            }

          /* Close the file descriptor and save the return status */

          status = close(stream->fs_fd);

          /* If close() returns an error but flush() did not then make sure
           * that we return the close() error condition.
           */

          if (ret == OK)
            {
              ret = status;
              errcode = get_errno();
            }
        }

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
      /* Destroy the semaphore */

      _SEM_DESTROY(&stream->fs_sem);

      /* Release the buffer */

      if (stream->fs_bufstart != NULL &&
          (stream->fs_flags & __FS_FLAG_UBF) == 0)
        {
          lib_free(stream->fs_bufstart);
        }
#endif

      lib_free(stream);
    }

  /* On an error, reset the errno to the first error encountered and return
   * EOF.
   */

  if (ret != OK)
    {
      set_errno(errcode);
      return EOF;
    }

  /* Return success */

  return OK;
}
