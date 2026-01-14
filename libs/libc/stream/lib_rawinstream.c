/****************************************************************************
 * libs/libc/stream/lib_rawinstream.c
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
 * Name: rawinstream_getc
 ****************************************************************************/

static int rawinstream_getc(FAR struct lib_instream_s *self)
{
  FAR struct lib_rawinstream_s *stream =
                                       (FAR struct lib_rawinstream_s *)self;
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
 * Name: rawinstream_getc
 ****************************************************************************/

static ssize_t rawinstream_gets(FAR struct lib_instream_s *self,
                                FAR void *buffer, size_t len)
{
  FAR struct lib_rawinstream_s *stream =
                                       (FAR struct lib_rawinstream_s *)self;
  ssize_t nread;

  DEBUGASSERT(self && stream->fd >= 0);

  /* Attempt to read one character */

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rawinstream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *              lib_rawinstream_s to be initialized.
 *   fd       - User provided file/socket descriptor (must have been opened
 *              for the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawinstream(FAR struct lib_rawinstream_s *stream, int fd)
{
  stream->common.getc = rawinstream_getc;
  stream->common.gets = rawinstream_gets;
  stream->common.nget = 0;
  stream->fd          = fd;
}
