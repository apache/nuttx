/****************************************************************************
 * libs/libc/stdio/lib_ftell.c
 *
 *   Copyright (C) 2008, 2011, 2013, 2017 Gregory Nutt. All rights reserved.
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

  if (stream->fs_bufstart != NULL && stream->fs_bufread != stream->fs_bufstart)
    {
#if CONFIG_NUNGET_CHARS > 0
      rdoffset = stream->fs_bufread - stream->fs_bufpos + stream->fs_nungotten;
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
