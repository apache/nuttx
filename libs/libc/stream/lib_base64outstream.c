/****************************************************************************
 * libs/libc/stream/lib_base64outstream.c
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
#include <nuttx/compiler.h>
#include <nuttx/streams.h>

#include <resolv.h>

/* Limit the output stream buffer size to multiple of 4 bytes. */

#define STREAM_BASE64_BUFFER_SIZE (CONFIG_STREAM_BASE64_BUFFER_SIZE / 4 * 4)

/****************************************************************************
 * Name: base64stream_flush
 ****************************************************************************/

static int base64stream_flush(FAR struct lib_outstream_s *self)
{
  FAR struct lib_base64outstream_s *stream = (FAR void *)self;

  if (stream->nbytes > 0)
    {
      b64_ntop(stream->bytes, stream->nbytes,
               stream->buffer + stream->pending, 4 + 1);

      stream->pending += 4;
      stream->nbytes = 0;
    }

  lib_stream_puts(stream->backend, stream->buffer, stream->pending);
  stream->pending = 0;
  return OK;
}

/****************************************************************************
 * Name: base64stream_putc
 ****************************************************************************/

static void base64stream_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct lib_base64outstream_s *stream = (FAR void *)self;

  stream->bytes[stream->nbytes++] = (char)ch;
  if (stream->nbytes == 3)
    {
      b64_ntop(stream->bytes, 3,
               stream->buffer + stream->pending, 4 + 1);
      stream->pending += 4;
      stream->nbytes = 0;
      if (stream->pending == STREAM_BASE64_BUFFER_SIZE)
        {
          base64stream_flush(self);
        }
    }

  self->nput++;
}

/****************************************************************************
 * Name: base64stream_puts
 ****************************************************************************/

static ssize_t base64stream_puts(FAR struct lib_outstream_s *self,
                                 FAR const void *buf, size_t len)
{
  FAR struct lib_base64outstream_s *stream = (FAR void *)self;
  FAR const unsigned char *input = (FAR const unsigned char *)buf;
  size_t remaining = len;

  if (stream->nbytes)
    {
      /* Flush the first three bytes */

      size_t n = 3 - stream->nbytes;
      if (n > remaining)
        {
          n = remaining;
        }

      memcpy(stream->bytes + stream->nbytes, input, n);
      stream->nbytes += n;
      remaining -= n;
      input += n;
      if (stream->nbytes == 3)
        {
          b64_ntop(stream->bytes, 3,
                   stream->buffer + stream->pending, 4 + 1);
          stream->pending += 4;
          stream->nbytes = 0;
          if (stream->pending == STREAM_BASE64_BUFFER_SIZE)
            {
              base64stream_flush(self);
            }
        }
    }

  while (remaining >= 3)
    {
      size_t outlen;
      size_t n = (STREAM_BASE64_BUFFER_SIZE - stream->pending) / 4 * 3;

      if (n > remaining)
        {
          n = remaining / 3 * 3;
        }

      outlen = n / 3 * 4;
      b64_ntop(input, n, stream->buffer + stream->pending,
               outlen + 1);
      stream->pending += outlen;
      input += n;
      remaining -= n;
      if (stream->pending == STREAM_BASE64_BUFFER_SIZE)
        {
          base64stream_flush(self);
        }
    }

  if (remaining > 0)
    {
      memcpy(stream->bytes, input, remaining);
      stream->nbytes = remaining;
    }

  self->nput += len;
  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_base64stream
 *
 * Description:
 *   Convert binary stream to base64 and redirect to output stream
 *
 * Input Parameters:
 *   stream    - User allocated, uninitialized instance of struct
 *               lib_base64outstream_s to be initialized.
 *   backend   - Stream backend port.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_base64outstream(FAR struct lib_base64outstream_s *stream,
                         FAR struct lib_outstream_s *backend)
{
  struct lib_outstream_s *public = &stream->common;

  public->putc    = base64stream_putc;
  public->puts    = base64stream_puts;
  public->flush   = base64stream_flush;
  public->nput    = 0;

  stream->pending = 0;
  stream->backend = backend;
}
