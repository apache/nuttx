/****************************************************************************
 * drivers/syslog/syslog_initialize.c
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

  ret = syslog_dev_channel();
#elif defined(CONFIG_SYSLOG_CONSOLE)
  /* Use the console device as the SYSLOG device */

  ret = syslog_console_channel();
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
