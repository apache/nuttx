/****************************************************************************
 * libs/libc/stdio/lib_stdio_ext.c
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

#include <fcntl.h>
#include <stdio.h>
#include <stdio_ext.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Does the buffer hold read data?  The buffer holds read data between
 * fs_bufstart and fs_bufread, or write data between fs_bufstart and
 * fs_bufpos (then fs_bufread == fs_bufstart).
 */

static bool stdio_ext_reading(FAR FILE *stream)
{
#if CONFIG_NUNGET_CHARS > 0
  if (stream->fs_nungotten > 0)
    {
      return true;
    }
#endif

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  return stream->fs_bufstart != NULL &&
         stream->fs_bufread != stream->fs_bufstart;
#else
  return false;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __fbufsize
 *
 * Description:
 *   Return the size of the buffer of the stream.
 *
 ****************************************************************************/

size_t __fbufsize(FAR FILE *stream)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  if (stream->fs_bufstart != NULL)
    {
      return stream->fs_bufend - stream->fs_bufstart;
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: __fpending
 *
 * Description:
 *   Return the number of bytes in the output buffer of the stream.
 *
 ****************************************************************************/

size_t __fpending(FAR FILE *stream)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  if (stream->fs_bufstart != NULL &&
      stream->fs_bufread == stream->fs_bufstart)
    {
      return stream->fs_bufpos - stream->fs_bufstart;
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: __freading
 *
 * Description:
 *   Return non-zero if the stream is read-only, or if the last operation on
 *   it was a read.
 *
 ****************************************************************************/

int __freading(FAR FILE *stream)
{
  return (stream->fs_oflags & O_ACCMODE) == O_RDONLY ||
         stdio_ext_reading(stream);
}

/****************************************************************************
 * Name: __fwriting
 *
 * Description:
 *   Return non-zero if the stream is write-only, or if the last operation
 *   on it was a write.
 *
 ****************************************************************************/

int __fwriting(FAR FILE *stream)
{
  return (stream->fs_oflags & O_ACCMODE) == O_WRONLY ||
         __fpending(stream) > 0;
}

/****************************************************************************
 * Name: __freadable
 *
 * Description:
 *   Return non-zero if the stream may be read.
 *
 ****************************************************************************/

int __freadable(FAR FILE *stream)
{
  return (stream->fs_oflags & O_ACCMODE) != O_WRONLY;
}

/****************************************************************************
 * Name: __fwritable
 *
 * Description:
 *   Return non-zero if the stream may be written.
 *
 ****************************************************************************/

int __fwritable(FAR FILE *stream)
{
  return (stream->fs_oflags & O_ACCMODE) != O_RDONLY;
}

/****************************************************************************
 * Name: __flbf
 *
 * Description:
 *   Return non-zero if the stream is line buffered.
 *
 ****************************************************************************/

int __flbf(FAR FILE *stream)
{
  return (stream->fs_flags & __FS_FLAG_LBF) != 0;
}

/****************************************************************************
 * Name: __fpurge
 *
 * Description:
 *   Discard the contents of the buffer of the stream: unread input,
 *   pushed-back characters and unwritten output.
 *
 ****************************************************************************/

void __fpurge(FAR FILE *stream)
{
  flockfile(stream);

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  stream->fs_bufpos  = stream->fs_bufstart;
  stream->fs_bufread = stream->fs_bufstart;
#endif

#if CONFIG_NUNGET_CHARS > 0
  stream->fs_nungotten = 0;
#endif

  funlockfile(stream);
}

/****************************************************************************
 * Name: __freadahead
 *
 * Description:
 *   Return the number of bytes that have been read into the buffer, or
 *   pushed back, and not yet consumed.
 *
 ****************************************************************************/

size_t __freadahead(FAR FILE *stream)
{
  size_t n = 0;

  if (!stdio_ext_reading(stream))
    {
      return 0;
    }

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  if (stream->fs_bufstart != NULL)
    {
      n = stream->fs_bufread - stream->fs_bufpos;
    }
#endif

#if CONFIG_NUNGET_CHARS > 0
  n += stream->fs_nungotten;
#endif

  return n;
}

/****************************************************************************
 * Name: __freadptr
 *
 * Description:
 *   Return a pointer to the bytes in the read buffer that have not been
 *   consumed, and their number in *sizep.  Return NULL if there are none,
 *   or if characters have been pushed back.
 *
 ****************************************************************************/

FAR const char *__freadptr(FAR FILE *stream, FAR size_t *sizep)
{
#if CONFIG_NUNGET_CHARS > 0
  if (stream->fs_nungotten > 0)
    {
      return NULL;
    }
#endif

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  if (stdio_ext_reading(stream) && stream->fs_bufread > stream->fs_bufpos)
    {
      *sizep = stream->fs_bufread - stream->fs_bufpos;
      return stream->fs_bufpos;
    }
#endif

  return NULL;
}

/****************************************************************************
 * Name: __freadptrinc
 *
 * Description:
 *   Consume inc bytes of the read buffer (see __freadptr()).
 *
 ****************************************************************************/

void __freadptrinc(FAR FILE *stream, size_t inc)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  stream->fs_bufpos += inc;
#endif
}

/****************************************************************************
 * Name: __fseterr
 *
 * Description:
 *   Set the error indicator of the stream.
 *
 ****************************************************************************/

void __fseterr(FAR FILE *stream)
{
  stream->fs_flags |= __FS_FLAG_ERROR;
}

/****************************************************************************
 * Name: __fsetlocking
 *
 * Description:
 *   The library always locks the stream; the type is ignored.
 *
 ****************************************************************************/

int __fsetlocking(FAR FILE *stream, int type)
{
  UNUSED(stream);
  UNUSED(type);

  return FSETLOCKING_INTERNAL;
}

/****************************************************************************
 * Name: _flushlbf
 *
 * Description:
 *   Flush all line-buffered streams.  NuttX flushes all streams.
 *
 ****************************************************************************/

void _flushlbf(void)
{
  fflush(NULL);
}
