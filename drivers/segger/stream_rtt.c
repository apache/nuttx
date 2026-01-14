/****************************************************************************
 * drivers/segger/stream_rtt.c
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

static void rttstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct lib_rttoutstream_s *stream =
                                 (FAR struct lib_rttoutstream_s *)self;

  SEGGER_RTT_BLOCK_IF_FIFO_FULL(0);
  stream->common.nput += SEGGER_RTT_PutChar(stream->channel, ch);
}

/****************************************************************************
 * Name: rttstream_puts
 ****************************************************************************/

static ssize_t rttstream_puts(FAR struct lib_outstream_s *self,
                              FAR const void *buf, size_t len)
{
  FAR struct lib_rttoutstream_s *stream =
                                (FAR struct lib_rttoutstream_s *)self;
  ssize_t ret;

  SEGGER_RTT_BLOCK_IF_FIFO_FULL(0);
  ret = SEGGER_RTT_Write(stream->channel, buf, len);
  stream->common.nput += ret;
  return ret;
}

/****************************************************************************
 * Name: rttstream_getc
 ****************************************************************************/

static int rttstream_getc(FAR struct lib_instream_s *self)
{
  FAR struct lib_rttinstream_s *stream =
                                (FAR struct lib_rttinstream_s *)self;
  int ch = -1;

  DEBUGASSERT(stream);
  stream->common.nget += SEGGER_RTT_Read(stream->channel, &ch, 1);
  return ch;
}

/****************************************************************************
 * Name: rttstream_gets
 ****************************************************************************/

static ssize_t rttstream_gets(FAR struct lib_instream_s *self,
                              FAR void * buffer, size_t size)
{
  FAR struct lib_rttinstream_s *stream =
                                (FAR struct lib_rttinstream_s *)self;
  ssize_t ret;

  DEBUGASSERT(stream);
  ret = SEGGER_RTT_Read(stream->channel, buffer, size);
  stream->common.nget += ret;
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
      stream->buffer = kmm_malloc(bufsize);
      DEBUGASSERT(stream->buffer);
      snprintf(stream->name, sizeof(stream->name), "rtt%d", channel);
      SEGGER_RTT_ConfigUpBuffer(channel, stream->name, stream->buffer,
                                bufsize, SEGGER_RTT_MODE_DEFAULT);
    }

  stream->common.putc = rttstream_putc;
  stream->common.puts = rttstream_puts;
  stream->common.flush = lib_noflush;
  stream->common.nput = 0;
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
      stream->buffer = kmm_malloc(bufsize);
      DEBUGASSERT(stream->buffer);
      snprintf(stream->name, sizeof(stream->name), "rtt%d", channel);
      SEGGER_RTT_ConfigDownBuffer(channel, stream->name, stream->buffer,
                                  bufsize, SEGGER_RTT_MODE_DEFAULT);
    }

  stream->common.getc = rttstream_getc;
  stream->common.gets = rttstream_gets;
  stream->common.nget = 0;
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
