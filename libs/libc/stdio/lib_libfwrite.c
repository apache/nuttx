/****************************************************************************
 * libs/libc/stdio/lib_libfwrite.c
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
 * Name: lib_fwrite
 ****************************************************************************/

ssize_t lib_fwrite(FAR const void *ptr, size_t count, FAR FILE *stream)
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
{
  FAR const unsigned char *start = ptr;
  FAR const unsigned char *src   = ptr;
  ssize_t ret = ERROR;
  size_t gulp_size;

  /* Make sure that writing to this stream is allowed */

  if (stream == NULL)
    {
      set_errno(EBADF);
      return ret;
    }

  /* Check if write access is permitted */

  if ((stream->fs_oflags & O_WROK) == 0)
    {
      set_errno(EBADF);
      goto errout;
    }

  /* If there is no I/O buffer, then output data immediately */

  if (stream->fs_bufstart == NULL)
    {
      ret = _NX_WRITE(stream->fs_fd, ptr, count);
      if (ret < 0)
        {
          _NX_SETERRNO(ret);
          ret = ERROR;
        }

      goto errout;
    }

  /* Get exclusive access to the stream */

  lib_take_lock(stream);

  /* If the buffer is currently being used for read access, then
   * discard all of the read-ahead data.  We do not support concurrent
   * buffered read/write access.
   */

  if (lib_rdflush(stream) < 0)
    {
      goto errout_with_lock;
    }

  /* Determine the number of bytes left in the buffer */

  gulp_size = stream->fs_bufend - stream->fs_bufpos;
  if (gulp_size != CONFIG_STDIO_BUFFER_SIZE || count < gulp_size)
    {
      if (gulp_size > count)
        {
          /* Yes, clip the gulp to the size of the user data */

          gulp_size = count;
        }

      /* Adjust the number of bytes remaining to be transferred
       * on the next pass through the loop (might be zero).
       */

      count -= gulp_size;

      /* Transfer the data into the buffer */

      memcpy(stream->fs_bufpos, src, gulp_size);
      stream->fs_bufpos += gulp_size;
      src += gulp_size;

      /* Is the buffer full? */

      if (stream->fs_bufpos >= stream->fs_bufend)
        {
          /* Flush the buffered data to the IO stream */

          int bytes_buffered = lib_fflush(stream, false);
          if (bytes_buffered < 0)
            {
              goto errout_with_lock;
            }
        }
    }

  if (count >= CONFIG_STDIO_BUFFER_SIZE)
    {
      ret = _NX_WRITE(stream->fs_fd, src, count);
      if (ret < 0)
        {
          _NX_SETERRNO(ret);
          ret = ERROR;
          goto errout_with_lock;
        }

      src += ret;
    }
  else if (count > 0)
    {
      memcpy(stream->fs_bufpos, src, count);
      stream->fs_bufpos += count;
      src += count;
    }

  /* Return the number of bytes written */

  ret = (uintptr_t)src - (uintptr_t)start;

errout_with_lock:
  lib_give_lock(stream);

errout:
  if (ret < 0)
    {
      stream->fs_flags |= __FS_FLAG_ERROR;
    }

  return ret;
}
#else
{
  ssize_t ret = _NX_WRITE(stream->fs_fd, ptr, count);
  if (ret < 0)
    {
      stream->fs_flags |= __FS_FLAG_ERROR;
      _NX_SETERRNO(ret);
    }

  return ret;
}
#endif /* CONFIG_STDIO_DISABLE_BUFFERING */
