/****************************************************************************
 * drivers/syslog/syslog_stream.c
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

#include <errno.h>
#include <sys/types.h>

#include <nuttx/kmalloc.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_SYSLOG_STREAM

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains all SYSLOGing state information */

struct syslog_stream_s
{
  struct syslog_channel_s     channel;
  FAR struct lib_outstream_s *stream;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t syslog_stream_write(FAR struct syslog_channel_s *channel,
                                   FAR const char *buffer, size_t buflen);
static int syslog_stream_putc(FAR struct syslog_channel_s *channel, int ch);
static int syslog_stream_force(FAR struct syslog_channel_s *channel, int ch);
static int syslog_stream_flush(FAR struct syslog_channel_s *channel);
void syslog_stream_uninit(FAR struct syslog_channel_s *channel);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct syslog_channel_ops_s g_syslog_stream_ops =
{
  syslog_stream_putc,
  syslog_stream_force,
  syslog_stream_flush,
  syslog_stream_write,
  syslog_stream_uninit
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_stream_write
 *
 * Description:
 *   This is the low-level, multiple byte, system logging interface provided
 *   for the driver interface.
 *
 * Input Parameters:
 *   channel    - Handle to syslog channel to be used.
 *   buffer     - The buffer containing the data to be output.
 *   buflen     - The number of bytes in the buffer.
 *
 * Returned Value:
 *   On success, the number of characters written is returned. A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t syslog_stream_write(FAR struct syslog_channel_s *channel,
                                   FAR const char *buffer, size_t buflen)
{
  FAR struct syslog_stream_s *chan =
    (FAR struct syslog_stream_s *)channel;
  ssize_t nwritten;
  irqstate_t flags;

  flags = enter_critical_section();
  nwritten = lib_stream_puts(chan->stream, buffer, buflen);
  leave_critical_section(flags);
  return nwritten;
}

/****************************************************************************
 * Name: syslog_stream_putc
 *
 * Description:
 *   This is the low-level, single character, system logging interface
 *   provided for the driver interface.
 *
 * Input Parameters:
 *   channel    - Handle to syslog channel to be used.
 *   ch         - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller. A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int syslog_stream_putc(FAR struct syslog_channel_s *channel, int ch)
{
  FAR struct syslog_stream_s *chan =
    (FAR struct syslog_stream_s *)channel;
  irqstate_t flags;

  flags = enter_critical_section();
  lib_stream_putc(chan->stream, ch);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: syslog_stream_force
 *
 * Description:
 *   Force output in interrupt context.
 *
 * Input Parameters:
 *   channel    - Handle to syslog channel to be used.
 *   ch         - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int syslog_stream_force(FAR struct syslog_channel_s *channel, int ch)
{
  FAR struct syslog_stream_s *chan =
    (FAR struct syslog_stream_s *)channel;

  lib_stream_putc(chan->stream, ch);
  return OK;
}

/****************************************************************************
 * Name: syslog_stream_flush
 *
 * Description:
 *   Flush any buffer data in the file system to media.
 *
 * Input Parameters:
 *   channel    - Handle to syslog channel to be used.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int syslog_stream_flush(FAR struct syslog_channel_s *channel)
{
  FAR struct syslog_stream_s *chan =
    (FAR struct syslog_stream_s *)channel;
  irqstate_t flags;

  flags = enter_critical_section();
  lib_stream_flush(chan->stream);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_stream_uninit
 *
 * Description:
 *   Disable the channel in preparation to use a different
 *   SYSLOG device.
 *
 * Input Parameters:
 *   channel    - Handle to syslog channel to be used.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   The caller has already switched the SYSLOG source to some safe channel
 *   (the default channel).
 *
 ****************************************************************************/

void syslog_stream_uninit(FAR struct syslog_channel_s *channel)
{
  FAR struct syslog_stream_s *chan =
    (FAR struct syslog_stream_s *)channel;

  /* Attempt to flush any buffered data. */

  syslog_stream_flush(channel);

  /* Free the channel structure */

  kmm_free(chan);
}

/****************************************************************************
 * Name: syslog_stream_channel
 *
 * Description:
 *   Initialize to use the device stream as the SYSLOG sink.
 *
 *   On power up, the SYSLOG facility is non-existent or limited to very
 *   low-level output.  This function may be called later in the
 *   initialization sequence after full driver support has been initialized.
 *   (via syslog_initialize())  It installs the configured SYSLOG drivers
 *   and enables full SYSLOGing capability.
 *
 * Input Parameters:
 *   stream - The stream device to be used.
 *
 * Returned Value:
 *   Returns a newly created SYSLOG channel, or NULL in case of any failure.
 *
 ****************************************************************************/

FAR struct syslog_channel_s *
syslog_stream_channel(FAR struct lib_outstream_s *stream)
{
  FAR struct syslog_stream_s *chan;

  DEBUGASSERT(stream != NULL);

  chan = (FAR struct syslog_stream_s *)
    kmm_zalloc(sizeof(struct syslog_stream_s));

  if (chan == NULL)
    {
      return NULL;
    }

  chan->stream = stream;
  chan->channel.sc_ops = &g_syslog_stream_ops;
  return (FAR struct syslog_channel_s *)chan;
}

#endif /* CONFIG_SYSLOG_STREAM */
