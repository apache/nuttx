/****************************************************************************
 * libs/libc/stream/lib_hexdumpstream.c
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
#include <nuttx/compiler.h>
#include <nuttx/streams.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nibble2hex
 *
 * Description:
 *  Convert a binary nibble to a hexadecimal character.
 *
 ****************************************************************************/

static char nibble2hex(unsigned char nibble)
{
  if (nibble < 10)
    {
      return '0' + nibble;
    }
  else
    {
      return 'A' + nibble - 10;
    }
}

/****************************************************************************
 * Name: bin2hex
 ****************************************************************************/

static size_t bin2hex(FAR const uint8_t *buf, size_t buflen,
                      FAR char *hex, size_t hexlen)
{
  size_t i;

  if (buflen > hexlen)
    {
      buflen = hexlen;
    }

  for (i = 0; i < buflen; i++)
    {
      hex[2 * i]     = nibble2hex(buf[i] >> 4);
      hex[2 * i + 1] = nibble2hex(buf[i] & 0xf);
    }

  return buflen;
}

/****************************************************************************
 * Name: hexdumpstream_flush
 ****************************************************************************/

static int hexdumpstream_flush(FAR struct lib_outstream_s *self)
{
  FAR struct lib_hexdumpstream_s *stream = (FAR void *)self;

  if (stream->pending > 0)
    {
      stream->buffer[stream->pending] = '\n';
      lib_stream_puts(stream->backend, stream->buffer, stream->pending + 1);
      stream->pending = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: hexdumpstream_putc
 ****************************************************************************/

static void hexdumpstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct lib_hexdumpstream_s *stream = (FAR void *)self;
  size_t outlen = CONFIG_STREAM_HEXDUMP_BUFFER_SIZE;
  const uint8_t byte = ch;

  bin2hex(&byte, 1, stream->buffer + stream->pending,
          (outlen - stream->pending) / 2);

  stream->pending += 2;

  if (stream->pending == outlen)
    {
      hexdumpstream_flush(self);
    }

  self->nput++;
}

/****************************************************************************
 * Name: hexdumpstream_puts
 ****************************************************************************/

static ssize_t hexdumpstream_puts(FAR struct lib_outstream_s *self,
                                  FAR const void *buf, size_t len)
{
  FAR struct lib_hexdumpstream_s *stream = (FAR void *)self;
  const unsigned char *p = buf;
  size_t outlen = CONFIG_STREAM_HEXDUMP_BUFFER_SIZE;
  size_t line = outlen / 2;
  size_t remain = len;
  ssize_t ret;

  while (remain > 0)
    {
      ret = remain > line ? line : remain;
      ret = bin2hex(p, ret, stream->buffer + stream->pending,
                    (outlen - stream->pending) / 2);

      p               += ret;
      remain          -= ret;
      stream->pending += ret * 2;

      if (stream->pending == outlen)
        {
          hexdumpstream_flush(self);
        }
    }

  self->nput += len;

  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_hexdumpstream
 *
 * Description:
 *   Convert binary stream to hex and redirect to syslog
 *
 * Input Parameters:
 *   stream    - User allocated, uninitialized instance of struct
 *               lib_bufferedoutstream_s to be initialized.
 *   backend   - Stream backend port.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_hexdumpstream(FAR struct lib_hexdumpstream_s *stream,
                       FAR struct lib_outstream_s *backend)
{
  struct lib_outstream_s *public = &stream->common;

  public->putc    = hexdumpstream_putc;
  public->puts    = hexdumpstream_puts;
  public->flush   = hexdumpstream_flush;
  public->nput    = 0;

  stream->pending = 0;
  stream->backend = backend;
}
