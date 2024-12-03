/****************************************************************************
 * libs/libc/stream/lib_meminstream.c
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

#include <assert.h>
#include <errno.h>
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: meminstream_getc
 ****************************************************************************/

static int meminstream_getc(FAR struct lib_instream_s *self)
{
  FAR struct lib_meminstream_s *stream =
                                       (FAR struct lib_meminstream_s *)self;
  int ret = -EINVAL;

  DEBUGASSERT(self);

  /* Get the next character (if any) from the buffer */

  if (self->nget < stream->buflen)
    {
      ret = stream->buffer[self->nget];
      self->nget++;
    }

  return ret;
}

/****************************************************************************
 * Name: meminstream_gets
 ****************************************************************************/

static ssize_t meminstream_gets(FAR struct lib_instream_s *self,
                                FAR void *buffer, size_t len)
{
  FAR struct lib_meminstream_s *stream =
                                       (FAR struct lib_meminstream_s *)self;
  ssize_t ret = -EINVAL;

  DEBUGASSERT(self);

  /* Get the buffer (if any) from the stream */

  if (self->nget < stream->buflen)
    {
      ret = stream->buflen - self->nget < len ?
            stream->buflen - self->nget : len;
      self->nget += ret;
      memcpy(buffer, stream->buffer, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_meminstream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *              lib_meminstream_s to be initialized.
 *   bufstart - Address of the beginning of the fixed-size memory buffer
 *   buflen   - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (stream initialized).
 *
 ****************************************************************************/

void lib_meminstream(FAR struct lib_meminstream_s *stream,
                     FAR const char *bufstart, size_t buflen)
{
  stream->common.getc = meminstream_getc;
  stream->common.gets = meminstream_gets;
  stream->common.nget = 0;          /* Will be buffer index */
  stream->buffer      = bufstart;   /* Start of buffer */
  stream->buflen      = buflen;     /* Length of the buffer */
}
