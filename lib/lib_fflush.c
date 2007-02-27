/************************************************************
 * lib_fflush.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>  /* for CONFIG_STDIO_BUFFER_SIZE */

#if CONFIG_NFILE_STREAMS > 0

#include <stdio.h>
#include <errno.h>
#include <nuttx/fs.h>
#include "lib_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/************************************************************
 * Global Constant Data
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Constant Data
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Global Functions
 ************************************************************/

/************************************************************
 * fflush
 ************************************************************/

/* Called by the OS when a task exits */

void lib_flushall(FAR struct streamlist *list)
{
  /* Make sure that there are streams associated with this thread */
  if (list)
    {
       int i;

       /* Process each stream in the thread's stream list */

       stream_semtake(list);
       for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
         {
           /* If the stream is open (i.e., assigned a non-
            * negative file descriptor), then flush the
            * stream.
            */

           if (list->sl_streams[i].fs_filedes >= 0)
             {
               (void)fflush(&list->sl_streams[i]);
             }
         }
       stream_semgive(list);
    }
}

int fflush(FILE *stream)
{
#if CONFIG_STDIO_BUFFER_SIZE > 0
  const unsigned char *src;
  size_t bytes_written;
  size_t nbuffer;

  if (stream->fs_filedes < 0 || (stream->fs_oflags & O_WROK) == 0)
    {
      *get_errno_ptr() = EBADF;
      return ERROR;
    }

  /* How many bytes are used in the buffer now */

  nbuffer = stream->fs_bufpos - stream->fs_bufstart;

  /* Try to write that amount */

  src           = stream->fs_bufstart;
  bytes_written = write(stream->fs_filedes, src, nbuffer);
  if (bytes_written < 0)
    {
      return bytes_written;
    }

  /* Update pointers and counts */

  src     += bytes_written;
  nbuffer -= bytes_written;

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

  /* Return the number of bytes remaining in the buffer */

  return stream->fs_bufpos - stream->fs_bufstart;
#else
  return 0;
#endif
}

#endif /* CONFIG_NFILE_STREAMS */
