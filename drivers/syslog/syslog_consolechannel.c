/****************************************************************************
 * drivers/syslog/syslog_consolechannel.c
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

#include <sys/stat.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/syslog/syslog.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_CONSOLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_LOWPUTC
#if defined(CONFIG_ARCH_LOWPUTC)
#  define HAVE_LOWPUTC 1
#endif

#define OPEN_FLAGS (O_WRONLY)
#define OPEN_MODE  (S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SYSLOG channel methods */

#ifndef HAVE_LOWPUTC
static int syslog_console_force(int ch);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the SYSLOG channel */

static const struct syslog_channel_s g_syslog_console_channel =
{
  syslog_dev_putc,
#ifdef HAVE_LOWPUTC
  up_putc,
#else
  syslog_console_force,
#endif
  syslog_dev_flush,
#ifdef CONFIG_SYSLOG_WRITE
  syslog_dev_write,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_console_force
 *
 * Description:
 *   A dummy FORCE method
 *
 ****************************************************************************/

#ifndef HAVE_LOWPUTC
static int syslog_console_force(int ch)
{
  return ch;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_console_channel
 *
 * Description:
 *   Configure to use the character device (or file) at /dev/console as the
 *   SYSLOG channel.
 *
 *   This tiny function is simply a wrapper around syslog_dev_initialize()
 *   and syslog_channel().  It calls syslog_dev_initialize() to configure
 *   the character device at /dev/console then calls syslog_channel() to
 *   use that device as the SYSLOG output channel.
 *
 *   NOTE interrupt level SYSLOG output will be lost in the general case
 *   unless the interrupt buffer is used.  As a special case:  If the serial
 *   console is used and the architecture provides up_putc(), the interrupt
 *   level output will be directed to up_putc() is the interrupt buffer is
 *   disabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int syslog_console_channel(void)
{
  int ret;

  /* Initialize the character driver interface */

  ret = syslog_dev_initialize("/dev/console", OPEN_FLAGS, OPEN_MODE);
  if (ret < 0)
    {
      return ret;
    }

  /* Use the character driver as the SYSLOG channel */

  return syslog_channel(&g_syslog_console_channel);
}

#endif /* CONFIG_SYSLOG_CONSOLE */
