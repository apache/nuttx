/****************************************************************************
 * drivers/syslog/syslog_devchannel.c
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
#include <errno.h>

#include <nuttx/syslog/syslog.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_CHAR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPEN_FLAGS (O_WRONLY)
#define OPEN_MODE  (S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Handle to the SYSLOG channel */

FAR static struct syslog_channel_s *g_syslog_dev_channel;

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
  /* Initialize the character driver interface */

  g_syslog_dev_channel = syslog_dev_initialize(CONFIG_SYSLOG_DEVPATH,
                                               OPEN_FLAGS, OPEN_MODE);
  if (g_syslog_dev_channel == NULL)
    {
      return -ENOMEM;
    }

  /* Use the character driver as the SYSLOG channel */

  return syslog_channel(g_syslog_dev_channel);
}

#endif /* CONFIG_SYSLOG_CHAR */
