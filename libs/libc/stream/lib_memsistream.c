/****************************************************************************
 * libs/libc/stream/lib_memsistream.c
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

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memsistream_getc
 ****************************************************************************/

static int memsistream_getc(FAR struct lib_sistream_s *self)
{
  FAR struct lib_memsistream_s *stream =
                                       (FAR struct lib_memsistream_s *)self;
  int ret = -EINVAL;

  DEBUGASSERT(self);

  /* Get the next character (if any) from the buffer */

  if (stream->offset < stream->buflen)
    {
      ret = stream->buffer[stream->offset];
      stream->offset++;
      self->nget++;
    }

  return ret;
}

/****************************************************************************
 * Name: meminstream_gets
 ****************************************************************************/

static ssize_t memsistream_gets(FAR struct lib_instream_s *self,
                                FAR void *buffer, size_t len)
{
  FAR struct lib_memsistream_s *stream =
                                       (FAR struct lib_memsistream_s *)self;
  ssize_t ret = -EINVAL;

  DEBUGASSERT(self);

  /* Get the buffer (if any) from the stream */

  if (self->nget < stream->buflen)
    {
      ret = stream->buflen - self->nget;
      ret = ret < len ? ret : len;
      self->nget += ret;
      memcpy(buffer, stream->buffer, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: memsistream_seek
 ****************************************************************************/

static off_t memsistream_seek(FAR struct lib_sistream_s *self, off_t offset,
                              int whence)
{
  FAR struct lib_memsistream_s *stream =
                                       (FAR struct lib_memsistream_s *)self;
  off_t newpos;

  DEBUGASSERT(self);

  switch (whence)
    {
      case SEEK_CUR:
        newpos = stream->offset + offset;
        break;

      case SEEK_SET:
        newpos = offset;
        break;

      case SEEK_END:
        newpos = stream->buflen + offset;
        break;

      default:
        return -EINVAL;
    }

  /* Make sure that the new position is within range */

  if (newpos < 0 || newpos >= stream->buflen)
    {
      return -EINVAL;
    }

  /* Return the new position */

  stream->offset = newpos;
  return newpos;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_memsistream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input Parameters:
 *   instream    - User allocated, uninitialized instance of struct
 *                 lib_memsistream_s to be initialized.
 *   bufstart    - Address of the beginning of the fixed-size memory buffer
 *   buflen      - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (instream initialized).
 *
 ****************************************************************************/

void lib_memsistream(FAR struct lib_memsistream_s *instream,
                     FAR const char *bufstart, size_t buflen)
{
  instream->common.getc = memsistream_getc;
  instream->common.gets = memsistream_gets;
  instream->common.seek = memsistream_seek;
  instream->common.nget = 0;          /* Total number of characters read */
  instream->buffer      = bufstart;   /* Start of buffer */
  instream->offset      = 0;          /* Will be the buffer index */
  instream->buflen      = buflen;     /* Length of the buffer */
}
