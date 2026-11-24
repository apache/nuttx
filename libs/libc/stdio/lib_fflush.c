/****************************************************************************
 * libs/libc/stdio/lib_fflush.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
/****************************************************************************
 * Name: fflush_seekable
 *
 * Description:
 *   Return true if the file of the stream can seek.  errno is preserved.
 *
 ****************************************************************************/

static bool fflush_seekable(FAR FILE *stream)
{
  int errcode = get_errno();
  off_t offset = 0;
  bool seekable;

  if (stream->fs_iofunc.seek != NULL)
    {
      seekable = stream->fs_iofunc.seek(stream->fs_cookie, &offset,
                                        SEEK_CUR) >= 0;
    }
  else
    {
      seekable = lseek((int)(intptr_t)stream->fs_cookie, 0, SEEK_CUR) >= 0;
    }

  set_errno(errcode);
  return seekable;
}
#endif

/****************************************************************************
 * Name: fflush_stream
 *
 * Description:
 *   Flush one stream.  Buffered write data is written out.  If the stream
 *   holds read-ahead data or pushed back characters and its file can seek,
 *   they are discarded and the file offset is set to the stream position,
 *   as POSIX requires.  The read data of a stream that cannot seek (a pipe
 *   or a terminal) is kept, because it could not be read again.
 *
 ****************************************************************************/

static ssize_t fflush_stream(FAR FILE *stream)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  bool rddata = stream->fs_bufstart != NULL &&
                stream->fs_bufread != stream->fs_bufstart;

#  if CONFIG_NUNGET_CHARS > 0
  rddata = rddata || stream->fs_nungotten > 0;
#  endif

  if (rddata && fflush_seekable(stream))
    {
      if (lib_rdflush_unlocked(stream) < 0)
        {
          return -get_errno();
        }

#  if CONFIG_NUNGET_CHARS > 0
      stream->fs_nungotten = 0;
#  endif
    }
#endif

  /* A stream that is not open for writing has no write data */

  if ((stream->fs_oflags & O_ACCMODE) == O_RDONLY)
    {
      return 0;
    }

  return lib_fflush_unlocked(stream);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fflush
 *
 * Description:
 *  The function fflush() forces a write of all user-space buffered data for
 *  the given output or update stream via the stream's underlying write
 *  function.  The open status of the stream is unaffected.  For a stream
 *  whose file can seek, read-ahead data and pushed back characters are
 *  discarded and the file offset is set to the stream position.
 *
 *  If the stream argument is NULL, fflush() flushes all open output streams.
 *
 * Returned Value:
 *  OK on success EOF on failure (with errno set appropriately)
 *
 ****************************************************************************/

int fflush_unlocked(FAR FILE *stream)
{
  int ret;

  /* Is the stream argument NULL? */

  if (stream == NULL)
    {
      /* Yes... then this is a request to flush all streams */

      ret = lib_flushall_unlocked(lib_get_streams());
    }
  else
    {
      ret = fflush_stream(stream);
    }

  /* Check the return value */

  if (ret < 0)
    {
      /* An error occurred during the flush AND/OR we were unable to flush
       * all of the buffered write data. Set the errno value.
       */

      set_errno(-ret);

      /* And return EOF on failure. */

      return EOF;
    }

  return OK;
}

int fflush(FAR FILE *stream)
{
  int ret;

  /* Is the stream argument NULL? */

  if (stream == NULL)
    {
      /* Yes... then this is a request to flush all streams */

      ret = lib_flushall(lib_get_streams());
    }
  else
    {
      flockfile(stream);
      ret = fflush_stream(stream);
      funlockfile(stream);
    }

  /* Check the return value */

  if (ret < 0)
    {
      /* An error occurred during the flush AND/OR we were unable to flush
       * all of the buffered write data. Set the errno value.
       */

      set_errno(-ret);

      /* And return EOF on failure. */

      return EOF;
    }

  return OK;
}
