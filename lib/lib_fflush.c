/****************************************************************************
 * lib/lib_fflush.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>  /* for CONFIG_STDIO_BUFFER_SIZE */

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/fs.h>

#include "lib_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Name: fflush_internal
 *
 * Description:
 *  The function fflush() forces a write of all user-space buffered data for
 *  the given output or update stream via the stream's underlying write
 *  function.  The open status of the stream is unaffected.
 *
 * Parmeters:
 *  stream - the stream to flush
 *  bforce - flush must be complete.
 *
 * Return:
 *  ERROR on failure, otherwise the number of bytes remaining in the buffer.
 *  If bforce is set, then only the values ERROR and 0 will be returned.
 *
 ****************************************************************************/

ssize_t fflush_internal(FILE *stream, boolean bforce)
{
#if CONFIG_STDIO_BUFFER_SIZE > 0
  const unsigned char *src;
  size_t bytes_written;
  size_t nbuffer;

  /* Return EBADF if the file is not opened for writing */

  if (stream->fs_filedes < 0 || (stream->fs_oflags & O_WROK) == 0)
    {
      *get_errno_ptr() = EBADF;
      return ERROR;
    }

  /* Make sure that we have exclusive access to the stream */

  lib_take_semaphore(stream);

  /* Make sure tht the buffer holds valid data */

  if (stream->fs_bufpos  != stream->fs_bufstart)
    {
       /* Make sure that the buffer holds buffered write data.  We do not
        * support concurrent read/write buffer usage.
        */

       if (stream->fs_bufread != stream->fs_bufstart)
        {
          /* The buffer holds read data... just return zero */

          return 0;
        }

      /* How many bytes of write data are used in the buffer now */

      nbuffer = stream->fs_bufpos - stream->fs_bufstart;

      /* Try to write that amount */

      src = stream->fs_bufstart;
      do
        {
          /* Perform the write */

          bytes_written = write(stream->fs_filedes, src, nbuffer);
          if (bytes_written < 0)
            {
              lib_give_semaphore(stream);
              return ERROR;
            }

          /* Handle partial writes.  fflush() must either return with
           * an error condition or with the data successfully flushed
           * from the buffer.
           */

          src     += bytes_written;
          nbuffer -= bytes_written;
        }
      while (bforce && nbuffer > 0);

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

  lib_give_semaphore(stream);
  return stream->fs_bufpos - stream->fs_bufstart;
#else
  /* Return no bytes remaining in the buffer */

  return 0;
#endif
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_fflushall
 *
 * Description:
 *   Called either (1) by the OS when a task exits, or (2) from fflush()
 *   when a NULL stream argument is provided.
 *
 ****************************************************************************/

int lib_flushall(FAR struct streamlist *list)
{
  int lasterrno = OK;
  int ret;

  /* Make sure that there are streams associated with this thread */

  if (list)
    {
       int i;

       /* Process each stream in the thread's stream list */

       stream_semtake(list);
       for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
         {
           /* If the stream is open (i.e., assigned a non-negative file
            * descriptor), then flush all of the pending write data in the
            * stream.
            */

           if (list->sl_streams[i].fs_filedes >= 0)
             {
               if (fflush_internal(&list->sl_streams[i], TRUE) != 0)
                 {
                   lasterrno = *get_errno_ptr();
                   ret = ERROR;
                 }
             }
         }
       stream_semgive(list);
    }

  /* If any flush failed, return that last failed flush */

  *get_errno_ptr() = lasterrno;
  return ret;
}

/****************************************************************************
 * Name: fflush
 *
 * Description:
 *  The function fflush() forces a write of all user-space buffered data for
 *  the given output or update stream via the stream's underlying write
 *  function.  The open status of the stream is unaffected.
 *
 *  If the stream argument is NULL, fflush() flushes all open output streams.
 *
 * Return:
 *  OK on success EOF on failure (with errno set appropriately)
 *
 ****************************************************************************/

int fflush(FILE *stream)
{
  if (!stream)
    {
      return lib_flushall(sched_getstreams());
    }
  else if (fflush_internal(stream, TRUE) != 0)
    {
      return EOF;
    }
  return OK;
}

