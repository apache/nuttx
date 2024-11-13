/****************************************************************************
 * drivers/syslog/syslog_channel.c
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/syslog/syslog.h>
#include <nuttx/compiler.h>
#include <nuttx/mutex.h>

#ifdef CONFIG_RAMLOG_SYSLOG
#  include <nuttx/syslog/ramlog.h>
#endif

#ifdef CONFIG_SYSLOG_RPMSG
#  include <nuttx/syslog/syslog_rpmsg.h>
#endif

#ifdef CONFIG_SYSLOG_RTT
#  include <nuttx/segger/rtt.h>
#endif

#ifdef CONFIG_ARCH_LOWPUTC
#  include <nuttx/arch.h>
#endif

#include "syslog.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_SYSLOG_DEFAULT)
static int syslog_default_putc(FAR syslog_channel_t *channel,
                               int ch);
static ssize_t syslog_default_write(FAR syslog_channel_t *channel,
                                    FAR const char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_SYSLOG_DEFAULT) && defined(CONFIG_ARCH_LOWPUTC)
static mutex_t g_lowputs_lock = NXMUTEX_INITIALIZER;
#endif

#ifdef CONFIG_RAMLOG_SYSLOG
static const struct syslog_channel_ops_s g_ramlog_channel_ops =
{
  ramlog_putc,
  ramlog_putc,
  NULL,
  ramlog_write
};

static syslog_channel_t g_ramlog_channel =
{
  &g_ramlog_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "ram"
#  endif
#  ifdef CONFIG_SYSLOG_CRLF
  , SYSLOG_CHANNEL_DISABLE_CRLF
#  endif
};
#endif

#ifdef CONFIG_SYSLOG_RPMSG
static const struct syslog_channel_ops_s g_rpmsg_channel_ops =
{
  syslog_rpmsg_putc,
  syslog_rpmsg_putc,
  syslog_rpmsg_flush,
  syslog_rpmsg_write,
  syslog_rpmsg_write
};

static syslog_channel_t g_rpmsg_channel =
{
  &g_rpmsg_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "rpmsg"
#  endif
#  ifdef CONFIG_SYSLOG_CRLF
  , SYSLOG_CHANNEL_DISABLE_CRLF
#  endif
};
#endif

#ifdef CONFIG_SYSLOG_RTT
static const struct syslog_channel_ops_s g_rtt_channel_ops =
{
  syslog_rtt_putc,
  syslog_rtt_putc,
  NULL,
  syslog_rtt_write,
  syslog_rtt_write
};

static syslog_channel_t g_rtt_channel =
{
  &g_rtt_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "rtt"
#  endif
#  ifdef CONFIG_SYSLOG_CRLF
  , SYSLOG_CHANNEL_DISABLE_CRLF
#  endif
};
#endif

#ifdef CONFIG_SYSLOG_DEFAULT
static const struct syslog_channel_ops_s g_default_channel_ops =
{
  syslog_default_putc,
  syslog_default_putc,
  NULL,
  syslog_default_write
};

static syslog_channel_t g_default_channel =
{
  &g_default_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "default"
#  endif
};
#endif

/* This is a simply sanity check to avoid we have more elements than the
 * `g_syslog_channel` array can hold
 */

#ifdef CONFIG_SYSLOG_DEFAULT
#  define SYSLOG_DEFAULT_AVAILABLE 1
#else
#  define SYSLOG_DEFAULT_AVAILABLE 0
#endif

#ifdef CONFIG_RAMLOG_SYSLOG
#  define RAMLOG_SYSLOG_AVAILABLE 1
#else
#  define RAMLOG_SYSLOG_AVAILABLE 0
#endif

#ifdef CONFIG_SYSLOG_RPMSG
#  define SYSLOG_RPMSG_AVAILABLE 1
#else
#  define SYSLOG_RPMSG_AVAILABLE 0
#endif

#ifdef CONFIG_SYSLOG_RTT
#  define SYSLOG_RTT_AVAILABLE 1
#else
#  define SYSLOG_RTT_AVAILABLE 0
#endif

#define SYSLOG_NCHANNELS (SYSLOG_DEFAULT_AVAILABLE + \
                          RAMLOG_SYSLOG_AVAILABLE + \
                          SYSLOG_RPMSG_AVAILABLE + \
                          SYSLOG_RTT_AVAILABLE)

#if SYSLOG_NCHANNELS > CONFIG_SYSLOG_MAX_CHANNELS
#  error "Maximum channel number exceeds."
#endif

/* This is the current syslog channel in use */

FAR syslog_channel_t *
#ifndef CONFIG_SYSLOG_REGISTER
const
#endif
g_syslog_channel[CONFIG_SYSLOG_MAX_CHANNELS] =
{
#ifdef CONFIG_SYSLOG_DEFAULT
  &g_default_channel,
#endif
#ifdef CONFIG_RAMLOG_SYSLOG
  &g_ramlog_channel,
#endif
#ifdef CONFIG_SYSLOG_RPMSG
  &g_rpmsg_channel,
#endif
#ifdef CONFIG_SYSLOG_RTT
  &g_rtt_channel
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_default_putc
 *
 * Description:
 *   If the arch supports a low-level putc function, output will be
 *   redirected there. Else acts as a dummy, no-nothing channel.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_DEFAULT
static int syslog_default_putc(FAR syslog_channel_t *channel, int ch)
{
  UNUSED(channel);

#  ifdef CONFIG_ARCH_LOWPUTC
  up_putc(ch);
#  endif
  return ch;
}

static ssize_t syslog_default_write(FAR syslog_channel_t *channel,
                                    FAR const char *buffer, size_t buflen)
{
#  ifdef CONFIG_ARCH_LOWPUTC
  nxmutex_lock(&g_lowputs_lock);

  up_nputs(buffer, buflen);

  nxmutex_unlock(&g_lowputs_lock);
#  endif

  UNUSED(channel);
  return buflen;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_channel
 *
 * Description:
 *   Configure the SYSLOGging function to use the provided channel to
 *   generate SYSLOG output.
 *
 * Input Parameters:
 *   channel - Provides the interface to the channel to be used.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_REGISTER
int syslog_channel_register(FAR syslog_channel_t *channel)
{
  DEBUGASSERT(channel != NULL);

  if (channel != NULL)
    {
#if CONFIG_SYSLOG_MAX_CHANNELS == 1
      g_syslog_channel[0] = channel;
      return OK;
#else
      int i;

      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          if (g_syslog_channel[i] == NULL)
            {
#  ifdef CONFIG_SYSLOG_IOCTL
              if (channel->sc_name[0] == '\0')
                {
                  snprintf(channel->sc_name, sizeof(channel->sc_name),
                           "channel-%p", channel->sc_ops);
                }
#  endif

              g_syslog_channel[i] = channel;
              return OK;
            }
          else if (g_syslog_channel[i] == channel)
            {
              return OK;
            }
        }
#endif
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: syslog_channel_unregister
 *
 * Description:
 *   Removes an already configured SYSLOG channel from the list of used
 *   channels.
 *
 * Input Parameters:
 *   channel - Provides the interface to the channel to be removed.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int syslog_channel_unregister(FAR syslog_channel_t *channel)
{
  int i;

  DEBUGASSERT(channel != NULL);

  if (channel != NULL)
    {
      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          if (g_syslog_channel[i] == channel)
            {
              /* Get the rest of the channels one position back
               * to ensure that there are no holes in the list.
               */

              while (i < (CONFIG_SYSLOG_MAX_CHANNELS - 1) &&
                     g_syslog_channel[i + 1] != NULL)
                {
                  g_syslog_channel[i] = g_syslog_channel[i + 1];
                  i++;
                }

              g_syslog_channel[i] = NULL;

              /* The channel is now removed from the list and its driver
               * can be safely uninitialized.
               */

              if (channel->sc_ops->sc_close)
                {
                  channel->sc_ops->sc_close(channel);
                }

              return OK;
            }
        }
    }

  return -EINVAL;
}
#endif
