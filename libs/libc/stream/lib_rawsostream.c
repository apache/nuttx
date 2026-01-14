/****************************************************************************
 * libs/libc/stream/lib_rawsostream.c
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rawsostream_putc
 ****************************************************************************/

static void rawsostream_putc(FAR struct lib_sostream_s *self, int ch)
{
  FAR struct lib_rawsostream_s *stream =
                                       (FAR struct lib_rawsostream_s *)self;
  char buffer = ch;
  ssize_t nwritten;

  DEBUGASSERT(self && stream->fd >= 0);

  /* Loop until the character is successfully transferred or until an
   * irrecoverable error occurs.
   */

  do
    {
      nwritten = _NX_WRITE(stream->fd, &buffer, 1);
      if (nwritten == 1)
        {
          self->nput++;
          return;
        }

      /* The only expected error is EINTR, meaning that the write operation
       * was awakened by a signal.  Zero would not be a valid return value
       * from _NX_WRITE().
       */

      nwritten = _NX_GETERRVAL(nwritten);
      DEBUGASSERT(nwritten < 0);
    }
  while (nwritten == -EINTR);
}

/****************************************************************************
 * Name: rawsostream_puts
 ****************************************************************************/

static ssize_t rawsostream_puts(FAR struct lib_sostream_s *self,
                                FAR const void *buffer, size_t len)
{
  FAR struct lib_rawsostream_s *stream =
                                       (FAR struct lib_rawsostream_s *)self;
  ssize_t nwritten;

  DEBUGASSERT(self && stream->fd >= 0);

  /* Loop until the buffer is successfully transferred or until an
   * irrecoverable error occurs.
   */

  do
    {
      nwritten = _NX_WRITE(stream->fd, buffer, len);
      if (nwritten >= 0)
        {
          self->nput += nwritten;
          return nwritten;
        }

      /* The only expected error is EINTR, meaning that the write operation
       * was awakened by a signal.  Zero would not be a valid return value
       * from _NX_WRITE().
       */

      nwritten = _NX_GETERRVAL(nwritten);
      DEBUGASSERT(nwritten < 0);
    }
  while (nwritten == -EINTR);

  return nwritten;
}

/****************************************************************************
 * Name: rawsostream_seek
 ****************************************************************************/

static off_t rawsostream_seek(FAR struct lib_sostream_s *self, off_t offset,
                              int whence)
{
  FAR struct lib_rawsostream_s *stream =
                                       (FAR struct lib_rawsostream_s *)self;

  DEBUGASSERT(self);
  offset = _NX_SEEK(stream->fd, offset, whence);
  if (offset < 0)
    {
      offset = _NX_GETERRVAL(offset);
    }

  return offset;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rawsostream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_rawsostream_s to be initialized.
 *   fd     - User provided file/socket descriptor (must have been opened
 *            for write access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawsostream(FAR struct lib_rawsostream_s *stream, int fd)
{
  stream->common.putc  = rawsostream_putc;
  stream->common.puts  = rawsostream_puts;
  stream->common.flush = lib_snoflush;
  stream->common.seek  = rawsostream_seek;
  stream->common.nput  = 0;
  stream->fd           = fd;
}
