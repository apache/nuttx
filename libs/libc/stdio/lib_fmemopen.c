/****************************************************************************
 * libs/libc/stdio/lib_fmemopen.c
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

struct fmemopen_cookie_s
{
  FAR char *buf;    /* Memory buffer */
  off_t pos;        /* Current position in the buffer */
  off_t end;        /* End buffer position */
  size_t size;      /* Buffer size */
  bool custom;      /* True if custom buffer is used */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fmemopen_read
 ****************************************************************************/

static ssize_t fmemopen_read(FAR void *c, FAR char *buf, size_t size)
{
  FAR struct fmemopen_cookie_s *fmemopen_cookie =
    (FAR struct fmemopen_cookie_s *)c;
  if (fmemopen_cookie->pos + size > fmemopen_cookie->end)
    {
      size = fmemopen_cookie->end - fmemopen_cookie->pos;
    }

  memcpy(buf, fmemopen_cookie->buf + fmemopen_cookie->pos, size);

  fmemopen_cookie->pos += size;
  return size;
}

/****************************************************************************
 * Name: fmemopen_write
 ****************************************************************************/

static ssize_t fmemopen_write(FAR void *c, FAR const char *buf, size_t size)
{
  FAR struct fmemopen_cookie_s *fmemopen_cookie =
    (FAR struct fmemopen_cookie_s *)c;
  if (size + fmemopen_cookie->pos > fmemopen_cookie->size)
    {
      size = fmemopen_cookie->size - fmemopen_cookie->pos;
    }

  memcpy(fmemopen_cookie->buf + fmemopen_cookie->pos, buf, size);

  fmemopen_cookie->pos += size;
  if (fmemopen_cookie->pos > fmemopen_cookie->end)
    {
      fmemopen_cookie->end = fmemopen_cookie->pos;
    }

  /* POSIX states that NULL byte shall be written at the current position
   * or end of the buffer.
   */

  if (fmemopen_cookie->pos < fmemopen_cookie->size &&
      fmemopen_cookie->buf[fmemopen_cookie->pos - 1] != '\0')
    {
      fmemopen_cookie->buf[fmemopen_cookie->pos] = '\0';
    }

  return size;
}

/****************************************************************************
 * Name: fmemopen_seek
 ****************************************************************************/

static off_t fmemopen_seek(FAR void *c, FAR off_t *offset, int whence)
{
  FAR struct fmemopen_cookie_s *fmemopen_cookie =
    (FAR struct fmemopen_cookie_s *)c;
  off_t new_offset;

  switch (whence)
    {
      case SEEK_SET:
        new_offset = *offset;
        break;
      case SEEK_END:
        new_offset = fmemopen_cookie->end + *offset;
        break;
      case SEEK_CUR:
        new_offset = fmemopen_cookie->pos + *offset;
        break;
      default:
        set_errno(ENOTSUP);
        return -1;
    }

  /* Seek to negative value or value larger than maximum size shall fail. */

  if (new_offset < 0 || new_offset > fmemopen_cookie->end)
    {
      set_errno(EINVAL);
      return -1;
    }

  fmemopen_cookie->pos = new_offset;
  *offset = new_offset;
  return new_offset;
}

/****************************************************************************
 * Name: fmemopen_close
 ****************************************************************************/

static int fmemopen_close(FAR void *c)
{
  FAR struct fmemopen_cookie_s *fmemopen_cookie =
    (FAR struct fmemopen_cookie_s *)c;
  if (fmemopen_cookie->custom)
    {
      lib_free(fmemopen_cookie->buf);
    }

  lib_free(fmemopen_cookie);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fmemopen
 ****************************************************************************/

FAR FILE *fmemopen(FAR void *buf, size_t size, FAR const char *mode)
{
  cookie_io_functions_t fmemopen_io;
  FAR struct fmemopen_cookie_s *fmemopen_cookie;
  FAR FILE *filep;
  int oflags;

  fmemopen_cookie = lib_zalloc(sizeof(struct fmemopen_cookie_s));
  if (fmemopen_cookie == NULL)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  oflags = lib_mode2oflags(mode);

  if (buf == NULL)
    {
      /* POSIX standard states:
       * Because this feature is only useful when the stream is opened for
       * updating (because there is no way to get a pointer to the buffer)
       * the fmemopen() call may fail if the mode argument does not
       * include a '+'.
       */

      if ((oflags & O_RDWR) != O_RDWR)
        {
          lib_free(fmemopen_cookie);
          set_errno(EINVAL);
          return NULL;
        }

      /* Buf argument is NULL pointer and mode is correct. This buffer
       * will be freed when stream is closed so we have to keep the
       * information in fmemopen_cookie->custom.
       */

      fmemopen_cookie->custom = true;
      fmemopen_cookie->buf    = lib_zalloc(size);
      if (fmemopen_cookie->buf == NULL)
        {
          lib_free(fmemopen_cookie);
          set_errno(ENOMEM);
          return NULL;
        }

      /* If buf is a null pointer, the initial position shall always be set
       * to the beginning of the buffer.
       */

      fmemopen_cookie->buf[0] = '\0';
    }
  else
    {
      /* Buffer was already allocated by the user. */

      fmemopen_cookie->custom = false;
      fmemopen_cookie->buf    = buf;
    }

  fmemopen_cookie->size = size;
  fmemopen_cookie->pos  = 0;

  /* For modes w and w+ the initial size shall be zero. */

  if ((oflags & O_TRUNC) != 0)
    {
      fmemopen_cookie->end = 0;
      fmemopen_cookie->buf[0] = '\0';
    }

  /* For modes r and r+ the size shall be set to the value given
   * by the size argument.
   */

  if ((oflags & O_RDWR) == O_RDOK)
    {
      fmemopen_cookie->end = size;
    }

  /* For modes a and a+ the initial size shall be:
   *  - Zero, if buf is a null pointer
   *  - The position of the first null byte in the buffer, if one is foun
   *  - The value of the size argument, if buf is not a null pointer and
   *    no null byte is found
   */

  if ((oflags & O_APPEND) != 0)
    {
      fmemopen_cookie->pos = fmemopen_cookie->end =
          strnlen(fmemopen_cookie->buf, fmemopen_cookie->size);
    }

  /* Assign fmemopen callbacks. */

  fmemopen_io.read  = fmemopen_read;
  fmemopen_io.write = fmemopen_write;
  fmemopen_io.seek  = fmemopen_seek;
  fmemopen_io.close = fmemopen_close;

  /* Let fopencookie do the rest. */

  filep = fopencookie(fmemopen_cookie, mode, fmemopen_io);
  if (filep == NULL)
    {
      if (fmemopen_cookie->custom)
        {
          lib_free(fmemopen_cookie->buf);
        }

      lib_free(fmemopen_cookie);
      return NULL;
    }

  return filep;
}
