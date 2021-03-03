/****************************************************************************
 * libs/libc/stdio/lib_ftell.c
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
#include <fcntl.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_getrdoffset
 *
 * Description:
 *   It is insufficient to simply use the file offset; we must also account
 *   for the data offset in the any buffered data.  This function calculates
 *   that offset.
 *
 * Returned Value:
 *   The file position offset due to buffered data.
 *
 ****************************************************************************/

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
static off_t lib_getrdoffset(FAR FILE *stream)
{
  off_t rdoffset = 0;
  lib_take_semaphore(stream);

  if (stream->fs_bufstart !=
      NULL && stream->fs_bufread !=
      stream->fs_bufstart)
    {
#if CONFIG_NUNGET_CHARS > 0
      rdoffset = stream->fs_bufread - stream->fs_bufpos +
                 stream->fs_nungotten;
#else
      rdoffset = stream->fs_bufread - stream->fs_bufpos;
#endif
    }

  lib_give_semaphore(stream);
  return rdoffset;
}
#else
#  define lib_getrdoffset(stream) (0)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftell
 *
 * Description:
 *   ftell() returns the current value of the file position indicator for the
 *   stream pointed to by stream.
 *
 * Returned Value:
 *   Zero on success; -1 on failure with errno set appropriately.
 *
 ****************************************************************************/

long ftell(FAR FILE *stream)
{
  off_t position;

  /* Verify that we were provided with a stream */

  if (!stream)
    {
      set_errno(EBADF);
      return ERROR;
    }

  /* Perform the lseek to the current position.  This will not move the
   * file pointer, but will return its current setting
   */

  position = lseek(stream->fs_fd, 0, SEEK_CUR);
  if (position != (off_t)-1)
    {
      return (long)(position - lib_getrdoffset(stream));
    }
  else
    {
      return ERROR;
    }
}
