/****************************************************************************
 * drivers/syslog/syslog_channel.c
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

#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_RAMLOG_SYSLOG
#  include <nuttx/syslog/ramlog.h>
#elif defined(CONFIG_SYSLOG_RPMSG)
#  include <nuttx/syslog/syslog_rpmsg.h>
#elif defined(CONFIG_ARCH_LOWPUTC)
#  include <nuttx/arch.h>
#endif

#include "syslog.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ARCH_LOWPUTC)
#  define HAVE_LOWPUTC
#elif !defined(CONFIG_RAMLOG_SYSLOG) && !defined(CONFIG_SYSLOG_RPMSG)
#  define NEED_LOWPUTC
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef NEED_LOWPUTC
static int syslog_default_putc(int ch);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_RAMLOG_SYSLOG)
static const struct syslog_channel_s g_default_channel =
{
  ramlog_putc,
  ramlog_putc,
};
#elif defined(CONFIG_SYSLOG_RPMSG)
static const struct syslog_channel_s g_default_channel =
{
  syslog_rpmsg_putc,
  syslog_rpmsg_putc,
  syslog_rpmsg_flush,
  syslog_rpmsg_write
};
#elif defined(HAVE_LOWPUTC)
static const struct syslog_channel_s g_default_channel =
{
  up_putc,
  up_putc,
};
#else
static const struct syslog_channel_s g_default_channel =
{
  syslog_default_putc,
  syslog_default_putc,
};
#endif

/* This is the current syslog channel in use */

FAR const struct syslog_channel_s
*g_syslog_channel[CONFIG_SYSLOG_MAX_CHANNELS] =
{
  &g_default_channel
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_default_putc and syslog_default_flush
 *
 * Description:
 *   Dummy, no-nothing channel interface methods
 *
 ****************************************************************************/

#ifdef NEED_LOWPUTC
static int syslog_default_putc(int ch)
{
  return ch;
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

int syslog_channel(FAR const struct syslog_channel_s *channel)
{
#if (CONFIG_SYSLOG_MAX_CHANNELS != 1)
  int i;
#endif

  DEBUGASSERT(channel != NULL);

  if (channel != NULL)
    {
      DEBUGASSERT(channel->sc_putc != NULL && channel->sc_force != NULL);

#if (CONFIG_SYSLOG_MAX_CHANNELS == 1)
      g_syslog_channel[0] = channel;
      return OK;
#else
      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          if (g_syslog_channel[i] == NULL)
            {
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
 * Name: syslog_channel_remove
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

int syslog_channel_remove(FAR const struct syslog_channel_s *channel)
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

              return OK;
            }
        }
    }

  return -EINVAL;
}
