/****************************************************************************
 * libs/libc/stream/lib_bufferedoutstream.c
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
 * Name: bufferedoutstream_flush
 ****************************************************************************/

static int bufferedoutstream_flush(FAR struct lib_outstream_s *self)
{
  FAR struct lib_bufferedoutstream_s *stream =
    (FAR struct lib_bufferedoutstream_s *)self;
  int ret = OK;

  ret = lib_stream_puts(stream->backend, stream->buffer,
                        stream->pending);

  if (ret >= 0)
    {
      stream->pending = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: bufferedoutstream_puts
 ****************************************************************************/

static int bufferedoutstream_puts(FAR struct lib_outstream_s *self,
                                 FAR const void *buf, int len)
{
  FAR struct lib_bufferedoutstream_s *stream =
    (FAR struct lib_bufferedoutstream_s *)self;
  int ret = len;

  if (stream->pending + len <= CONFIG_STREAM_OUT_BUFFER_SIZE)
    {
      /* If buffer is enough to save incoming data, cache it */

      memcpy(stream->buffer + stream->pending, buf, len);
      stream->pending += len;
    }
  else
    {
      /* Or, for long data flush buffer and write it directly */

      ret = lib_stream_flush(self);
      if (ret >= 0)
        {
          ret = lib_stream_puts(stream->backend, buf, len);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bufferedoutstream_putc
 ****************************************************************************/

static void bufferedoutstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  char c = ch;

  bufferedoutstream_puts(self, &c, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_bufferedoutstream
 ****************************************************************************/

void lib_bufferedoutstream(FAR struct lib_bufferedoutstream_s *stream,
                           FAR struct lib_outstream_s *backend)
{
  stream->common.putc  = bufferedoutstream_putc;
  stream->common.puts  = bufferedoutstream_puts;
  stream->common.flush = bufferedoutstream_flush;
  stream->common.nput  = 0;
  stream->backend      = backend;
  stream->pending      = 0;
}
