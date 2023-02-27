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

static int bufferedoutstream_flush(FAR struct lib_outstream_s *this)
{
  FAR struct lib_bufferedoutstream_s *rthis =
    (FAR struct lib_bufferedoutstream_s *)this;
  int ret = OK;

  ret = lib_stream_puts(rthis->backend, rthis->buffer,
                        rthis->pending);

  if (ret >= 0)
    {
      rthis->pending = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: bufferedoutstream_puts
 ****************************************************************************/

static int bufferedoutstream_puts(FAR struct lib_outstream_s *this,
                                 FAR const void *buf, int len)
{
  FAR struct lib_bufferedoutstream_s *rthis =
    (FAR struct lib_bufferedoutstream_s *)this;
  int ret = len;

  if (rthis->pending + len <= CONFIG_STREAM_OUT_BUFFER_SIZE)
    {
      /* If buffer is enough to save incoming data, cache it */

      memcpy(rthis->buffer + rthis->pending, buf, len);
      rthis->pending += len;
    }
  else
    {
      /* Or, for long data flush buffer and write it directly */

      ret = lib_stream_flush(this);
      if (ret >= 0)
        {
          ret = lib_stream_puts(rthis->backend, buf, len);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bufferedoutstream_putc
 ****************************************************************************/

static void bufferedoutstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  char c = ch;

  bufferedoutstream_puts(this, &c, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_bufferedoutstream
 ****************************************************************************/

void lib_bufferedoutstream(FAR struct lib_bufferedoutstream_s *outstream,
                           FAR struct lib_outstream_s *backend)
{
  outstream->public.putc  = bufferedoutstream_putc;
  outstream->public.puts  = bufferedoutstream_puts;
  outstream->public.flush = bufferedoutstream_flush;
  outstream->public.nput  = 0;
  outstream->backend      = backend;
  outstream->pending      = 0;
}
