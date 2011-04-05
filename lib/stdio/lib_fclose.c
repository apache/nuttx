/****************************************************************************
 * lib/stdio/lib_fclose.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "lib_internal.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

int fclose(FAR FILE *stream)
{
  int ret = OK;
  if (stream)
    {
      if (stream->fs_filedes > 0)
        {
          ret = close(stream->fs_filedes);
        }

#if CONFIG_STDIO_BUFFER_SIZE > 0
      /* Destroy the semaphore */

      sem_destroy(&stream->fs_sem);

      /* release the buffer */

      if (stream->fs_bufstart)
        {
          lib_free(stream->fs_bufstart);
        }

      /* Clear the whole structure */

      memset(stream, 0, sizeof(FILE));
#else
#if CONFIG_NUNGET_CHARS > 0
      /* Reset the number of ungetc characters */

      stream->fs_nungotten = 0;
#endif
      /* Reset the flags */

      stream->fs_oflags = 0;
#endif
      /* Setting the fs_filedescriptor to -1 makes the stream available for reuse */

      stream->fs_filedes = -1;
    }

  return ret;
}

