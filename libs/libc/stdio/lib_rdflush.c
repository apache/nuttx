/****************************************************************************
 * libs/libc/stdio/lib_rdflush.c
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
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "libc.h"

#ifndef CONFIG_STDIO_DISABLE_BUFFERING

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rdflush
 *
 * Description:
 *   Flush read data from the I/O buffer and adjust the file pointer to
 *   account for the unread data
 *
 ****************************************************************************/

int lib_rdflush(FAR FILE *stream)
{
  /* Sanity checking */

  if (stream == NULL)
    {
      set_errno(EBADF);
      return ERROR;
    }

  /* Do nothing if there is no I/O buffer */

  if (stream->fs_bufstart == NULL)
    {
      return OK;
    }

  /* Get exclusive access to the stream */

  flockfile(stream);

  /* If the buffer is currently being used for read access, then discard all
   * of the read-ahead data. We do not support concurrent buffered read/write
   * access.
   */

  if (stream->fs_bufread != stream->fs_bufstart)
    {
      /* Now adjust the stream pointer to account for the read-ahead data
       * that was not actually read by the user.
       */

#if CONFIG_NUNGET_CHARS > 0
      off_t rdoffset = stream->fs_bufread - stream->fs_bufpos +
                       stream->fs_nungotten;
#else
      off_t rdoffset = stream->fs_bufread - stream->fs_bufpos;
#endif
      /* Mark the buffer as empty (do this before calling fseek() because
       * fseek() also calls this function).
       */

      stream->fs_bufpos = stream->fs_bufread = stream->fs_bufstart;
#if CONFIG_NUNGET_CHARS > 0
      stream->fs_nungotten = 0;
#endif
      /* Then seek to the position corresponding to the last data read by the
       * user
       */

      if (fseek(stream, -rdoffset, SEEK_CUR) < 0)
        {
          funlockfile(stream);
          return ERROR;
        }
    }

  funlockfile(stream);
  return OK;
}

#endif /* CONFIG_STDIO_DISABLE_BUFFERING */
