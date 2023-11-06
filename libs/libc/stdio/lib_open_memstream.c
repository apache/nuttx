/****************************************************************************
 * libs/libc/stdio/lib_open_memstream.c
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct memstream_cookie_s
{
  FAR char **buf;   /* Memory buffer */
  char saved;       /* char at sizep before '\0' */
  size_t *sizep;
  off_t size;       /* Allocated buffer size */
  off_t end;        /* Maximum position we have written to */
  off_t pos;        /* Current position in the buffer */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memstream_write
 ****************************************************************************/

static ssize_t memstream_write(FAR void *c, FAR const char *buf,
                               size_t size)
{
  FAR struct memstream_cookie_s *memstream_cookie =
    (FAR struct memstream_cookie_s *)c;
  FAR char *buf_grow;
  size_t new_size;

  if (memstream_cookie->pos + size + 1 > memstream_cookie->size)
    {
      /* We have to reallocate the buffer. */

      new_size = memstream_cookie->pos + size + 1;
      buf_grow = lib_realloc(*memstream_cookie->buf, new_size);
      if (buf_grow == NULL)
        {
          return -ENOMEM;
        }

      memset(buf_grow + memstream_cookie->end, 0,
            new_size - memstream_cookie->size);

      *memstream_cookie->buf = buf_grow;
      memstream_cookie->size = new_size;
    }

  memcpy(*memstream_cookie->buf + memstream_cookie->pos, buf, size);

  /* POSIX: If a write moves the position to a value larger than
   * the current length, the current length shall be set to this position.
   * In this case a null character shall be appended to the current buffer.
   */

  memstream_cookie->pos += size;
  if (memstream_cookie->pos > memstream_cookie->end)
    {
      memstream_cookie->end = memstream_cookie->pos;
    }
  else
    {
      memstream_cookie->saved = *(*memstream_cookie->buf +
        memstream_cookie->pos);
    }

  *memstream_cookie->sizep = memstream_cookie->pos;
  *(*memstream_cookie->buf + memstream_cookie->pos) = '\0';

  return size;
}

/****************************************************************************
 * Name: memstream_seek
 ****************************************************************************/

static off_t memstream_seek(FAR void *c, FAR off_t *offset, int whence)
{
  FAR struct memstream_cookie_s *memstream_cookie =
    (FAR struct memstream_cookie_s *)c;
  off_t new_offset;

  switch (whence)
    {
      case SEEK_SET:
        new_offset = *offset;
        break;
      case SEEK_END:
        new_offset = memstream_cookie->end + *offset;
        break;
      case SEEK_CUR:
        new_offset = memstream_cookie->pos + *offset;
        break;
      default:
        set_errno(ENOTSUP);
        return -1;
    }

  /* Seek to negative value or value larger than maximum size shall fail. */

  if (new_offset < 0 || new_offset > memstream_cookie->end)
    {
      set_errno(EINVAL);
      return -1;
    }

  if (memstream_cookie->pos < memstream_cookie->end)
    {
      /* Retrieve saved character if we painted in already written area. */

      *(*memstream_cookie->buf + memstream_cookie->pos) =
        memstream_cookie->saved;
    }

  memstream_cookie->pos = new_offset;
  if (memstream_cookie->pos < memstream_cookie->end)
    {
      /* We go backwards, therefore we have to write null character
       * at memstream_cookie->pos. But we might want to keep this
       * character for future seeks, so keep it in memstream_cookie->saved.
       */

      memstream_cookie->saved = *(*memstream_cookie->buf +
        memstream_cookie->pos);
      *(*memstream_cookie->buf + memstream_cookie->pos) = '\0';
      *memstream_cookie->sizep = memstream_cookie->pos;
    }
  else
    {
      *memstream_cookie->sizep = memstream_cookie->end;
    }

  *offset = new_offset;
  return new_offset;
}

/****************************************************************************
 * Name: memstream_close
 ****************************************************************************/

static int memstream_close(FAR void *c)
{
  FAR struct memstream_cookie_s *memstream_cookie =
    (FAR struct memstream_cookie_s *)c;

  lib_free(memstream_cookie);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR FILE *open_memstream(FAR char **bufp, FAR size_t *sizep)
{
  cookie_io_functions_t memstream_io;
  FAR struct memstream_cookie_s *memstream_cookie;
  FAR FILE *filep;

  if (bufp == NULL || sizep == NULL)
    {
      set_errno(EINVAL);
      return NULL;
    }

  memstream_cookie = lib_zalloc(sizeof(struct memstream_cookie_s));
  if (memstream_cookie == NULL)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  *bufp = NULL;
  *sizep = 0;
  memstream_cookie->buf   = bufp;
  memstream_cookie->sizep = sizep;

  memstream_io.read   = NULL;
  memstream_io.write  = memstream_write;
  memstream_io.seek   = memstream_seek;
  memstream_io.close  = memstream_close;

  filep = fopencookie(memstream_cookie, "w", memstream_io);
  if (filep == NULL)
    {
      lib_free(memstream_cookie);
      return NULL;
    }

  return filep;
}
