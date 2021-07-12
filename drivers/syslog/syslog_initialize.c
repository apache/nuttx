/****************************************************************************
 * drivers/syslog/syslog_initialize.c
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

#include <stdio.h>

#include <nuttx/syslog/ramlog.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_rpmsg.h>

#include "syslog.h"

#ifndef CONFIG_ARCH_SYSLOG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_initialize
 *
 * Description:
 *   One power up, the SYSLOG facility is non-existent or limited to very
 *   low-level output.  This function is called later in the initialization
 *   sequence after full driver support has been initialized.  It installs
 *   the configured SYSLOG drivers and enables full SYSLOGing capability.
 *
 *   This function performs these basic operations:
 *
 *   - Initialize the SYSLOG device
 *   - Call syslog_channel() to begin using that device.
 *
 *   If CONFIG_ARCH_SYSLOG is selected, then the architecture-specifica
 *   logic will provide its own SYSLOG device initialize which must include
 *   as a minimum a call to syslog_channel() to use the device.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int syslog_initialize(void)
{
  int ret = OK;

#if defined(CONFIG_SYSLOG_CHAR)
  /* Enable use of a character device as the SYSLOG device */

  syslog_dev_channel();
#elif defined(CONFIG_SYSLOG_CONSOLE)
  /* Use the console device as the SYSLOG device */

  syslog_console_channel();
#endif

#ifdef CONFIG_RAMLOG_SYSLOG
  ramlog_syslog_register();
#endif

#ifdef CONFIG_SYSLOG_CHARDEV
  syslog_register();
#endif

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#endif

  return ret;
}

#endif /* CONFIG_ARCH_SYSLOG */
