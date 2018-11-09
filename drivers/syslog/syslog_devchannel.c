/****************************************************************************
 * drivers/syslog/syslog_devchannel.c
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

#include <sys/stat.h>
#include <fcntl.h>

#include <nuttx/syslog/syslog.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_CHAR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPEN_FLAGS (O_WRONLY)
#define OPEN_MODE  (S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SYSLOG channel methods */

#ifdef CONFIG_SYSLOG_CHAR_CRLF
static int syslog_devchan_putc(int ch);
#endif
static int syslog_devchan_force(int ch);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the SYSLOG channel */

static const struct syslog_channel_s g_syslog_dev_channel =
{
#ifdef CONFIG_SYSLOG_CHAR_CRLF
  syslog_devchan_putc,
#else
  syslog_dev_putc,
#endif
  syslog_devchan_force,
  syslog_dev_flush,
#ifdef CONFIG_SYSLOG_WRITE
  syslog_dev_write,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_devchan_putc
 *
 * Description:
 *   A front-end to syslog_dev_putc that does LF -> CR-LF expansion
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CHAR_CRLF
static int syslog_devchan_putc(int ch)
{
  int ret;

  /* Check for a linefeed */

  if (ch == '\n')
    {
      /* Pre-pend a carriage return */

      ret = syslog_dev_putc('\r');
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Output the provided character */

  return syslog_dev_putc(ch);
}
#endif

/****************************************************************************
 * Name: syslog_devchan_force
 *
 * Description:
 *   A dummy FORCE method
 *
 ****************************************************************************/

static int syslog_devchan_force(int ch)
{
  return ch;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_dev_channel
 *
 * Description:
 *   Configure to use the character device at CONFIG_SYSLOG_DEVPATH as the
 *   SYSLOG channel.
 *
 *   This tiny function is simply a wrapper around syslog_dev_initialize()
 *   and syslog_channel().  It calls syslog_dev_initialize() to configure
 *   the character device at CONFIG_SYSLOG_DEVPATH then calls
 *   syslog_channel() to use that device as the SYSLOG output channel.
 *
 *   NOTE interrupt level SYSLOG output will be lost in this case unless
 *   the interrupt buffer is used.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int syslog_dev_channel(void)
{
  int ret;

  /* Initialize the character driver interface */

  ret = syslog_dev_initialize(CONFIG_SYSLOG_DEVPATH, OPEN_FLAGS, OPEN_MODE);
  if (ret < 0)
    {
      return ret;
    }

  /* Use the character driver as the SYSLOG channel */

  return syslog_channel(&g_syslog_dev_channel);
}

#endif /* CONFIG_SYSLOG_CHAR */
