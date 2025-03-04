/****************************************************************************
 * libs/libc/stream/lib_rawsistream.c
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
 * Name: rawsistream_getc
 ****************************************************************************/

static int rawsistream_getc(FAR struct lib_sistream_s *self)
{
  FAR struct lib_rawsistream_s *stream =
                                       (FAR struct lib_rawsistream_s *)self;
  int nread;
  char ch;

  DEBUGASSERT(self && stream->fd >= 0);

  /* Attempt to read one character */

  nread = _NX_READ(stream->fd, &ch, 1);
  if (nread == 1)
    {
      self->nget++;
      return ch;
    }
  else
    {
      return _NX_GETERRVAL(nread);
    }
}

/****************************************************************************
 * Name: rawsistream_gets
 ****************************************************************************/

static ssize_t rawsistream_gets(FAR struct lib_instream_s *self,
                                FAR void *buffer, size_t len)
{
  FAR struct lib_rawsistream_s *stream =
                                       (FAR struct lib_rawsistream_s *)self;
  ssize_t nread;

  DEBUGASSERT(self && stream->fd >= 0);

  /* Attempt to read a buffer */

  nread = _NX_READ(stream->fd, buffer, len);
  if (nread >= 0)
    {
      self->nget += nread;
    }
  else
    {
      nread = _NX_GETERRVAL(nread);
    }

  return nread;
}

/****************************************************************************
 * Name: rawsistream_seek
 ****************************************************************************/

static off_t rawsistream_seek(FAR struct lib_sistream_s *self, off_t offset,
                              int whence)
{
  FAR struct lib_rawsistream_s *stream =
                                       (FAR struct lib_rawsistream_s *)self;

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
 * Name: lib_rawsistream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_rawsistream_s to be initialized.
 *   fd     - User provided file/socket descriptor (must have been opened
 *            for the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawsistream(FAR struct lib_rawsistream_s *stream, int fd)
{
  stream->common.getc = rawsistream_getc;
  stream->common.gets = rawsistream_gets;
  stream->common.seek = rawsistream_seek;
  stream->common.nget = 0;
  stream->fd          = fd;
}
