/****************************************************************************
 * drivers/segger/stream_rtt.c
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
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/streams.h>
#include <nuttx/segger/rtt.h>

#include <SEGGER_RTT.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rttstream_putc
 ****************************************************************************/

static void rttstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  FAR struct lib_rttoutstream_s *stream =
                                 (FAR struct lib_rttoutstream_s *)this;
  stream->public.nput += SEGGER_RTT_PutChar(stream->channel, ch);
}

/****************************************************************************
 * Name: rttstream_puts
 ****************************************************************************/

static int rttstream_puts(FAR struct lib_outstream_s *this,
                          FAR const void *buf, int len)
{
  FAR struct lib_rttoutstream_s *stream =
                                (FAR struct lib_rttoutstream_s *)this;
  int ret = SEGGER_RTT_Write(stream->channel, buf, len);
  stream->public.nput += ret;
  return ret;
}

/****************************************************************************
 * Name: rttstream_getc
 ****************************************************************************/

static int rttstream_getc(FAR struct lib_instream_s *this)
{
  FAR struct lib_rttinstream_s *stream =
                                (FAR struct lib_rttinstream_s *)this;
  int ch = -1;

  DEBUGASSERT(stream);
  stream->public.nget += SEGGER_RTT_Read(stream->channel, &ch, 1);
  return ch;
}

/****************************************************************************
 * Name: rttstream_gets
 ****************************************************************************/

static int rttstream_gets(FAR struct lib_instream_s *this,
                          FAR void * buffer, int size)
{
  FAR struct lib_rttinstream_s *stream =
                                (FAR struct lib_rttinstream_s *)this;
  int ret;

  DEBUGASSERT(stream);
  ret = SEGGER_RTT_Read(stream->channel, buffer, size);
  stream->public.nget += ret;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rttoutstream_open
 *
 * Description:
 *   Initializes a stream for use with the configured RTT interface.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_rttoutstream_s to be initialized.
 *   channel - SEGGER RTT channel number
 *   bufsize - Size of the RTT buffer
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rttoutstream_open(FAR struct lib_rttoutstream_s *stream,
                           int channel, size_t bufsize)
{
  if (channel)
    {
      bufsize = bufsize ? bufsize : BUFFER_SIZE_UP;
      stream->buffer = (FAR char *)kmm_malloc(bufsize);
      DEBUGASSERT(stream->buffer);
      snprintf(stream->name, sizeof(stream->name), "rtt%d", channel);
      SEGGER_RTT_ConfigUpBuffer(channel, stream->name, stream->buffer,
                                bufsize, SEGGER_RTT_MODE_DEFAULT);
    }

  stream->public.putc = rttstream_putc;
  stream->public.puts = rttstream_puts;
  stream->public.flush = lib_noflush;
  stream->public.nput = 0;
  stream->channel = channel;
}

/****************************************************************************
 * Name: lib_rttoutstream_close
 ****************************************************************************/

void lib_rttoutstream_close(FAR struct lib_rttoutstream_s *stream)
{
  if (stream->channel)
    {
      kmm_free(stream->buffer);
      SEGGER_RTT_ConfigUpBuffer(stream->channel, NULL, NULL,
                                0, SEGGER_RTT_MODE_DEFAULT);
    }
}

/****************************************************************************
 * Name: lib_rttinstream
 *
 * Description:
 *   Initializes input stream for use with the configured RTT interface.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_rttinstream_s to be initialized.
 *   channel - SEGGER RTT channel number
 *   bufsize - Size of the RTT input buffer
 *
 ****************************************************************************/

void lib_rttinstream_open(FAR struct lib_rttinstream_s *stream,
                          int channel, size_t bufsize)
{
  if (channel)
    {
      bufsize = bufsize ? bufsize : BUFFER_SIZE_DOWN;
      stream->buffer = (FAR char *)kmm_malloc(bufsize);
      DEBUGASSERT(stream->buffer);
      snprintf(stream->name, sizeof(stream->name), "rtt%d", channel);
      SEGGER_RTT_ConfigDownBuffer(channel, stream->name, stream->buffer,
                                  bufsize, SEGGER_RTT_MODE_DEFAULT);
    }

  stream->public.getc = rttstream_getc;
  stream->public.gets = rttstream_gets;
  stream->public.nget = 0;
  stream->channel = channel;
}

/****************************************************************************
 * Name: lib_rttinstream_close
 ****************************************************************************/

void lib_rttinstream_close(FAR struct lib_rttinstream_s *stream)
{
  if (stream->channel)
    {
      kmm_free(stream->buffer);
      SEGGER_RTT_ConfigDownBuffer(stream->channel, NULL, NULL,
                                  0, SEGGER_RTT_MODE_DEFAULT);
    }
}
