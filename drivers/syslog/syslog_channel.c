/****************************************************************************
 * drivers/syslog/syslog_channel.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

FAR const struct syslog_channel_s *g_syslog_channel = &g_default_channel;

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
  DEBUGASSERT(channel != NULL);

  if (channel != NULL)
    {
      DEBUGASSERT(channel->sc_putc != NULL && channel->sc_force != NULL);

      g_syslog_channel = channel;
      return OK;
    }

  return -EINVAL;
}
