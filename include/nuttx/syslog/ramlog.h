/****************************************************************************
 * include/nuttx/syslog/ramlog.h
 * The RAM logging driver
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

/* The RAM logging driver is a driver that was intended to support debugging
 * output (syslogging) when the normal serial output is not available.  For
 * example, if you are using a Telnet or USB serial console, the debug
 * output will get lost.
 *
 * The RAM logging  driver is similar to a pipe in that it saves the
 * debugging output in a FIFO in RAM.  It differs from a pipe in numerous
 * details as needed to support logging.
 *
 * This driver is built when CONFIG_RAMLOG is defined in the NuttX
 * configuration.
 */

#ifndef __INCLUDE_NUTTX_SYSLOG_RAMLOG_H
#define __INCLUDE_NUTTX_SYSLOG_RAMLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_RAMLOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_RAMLOG - Enables the RAM logging feature
 * CONFIG_RAMLOG_SYSLOG - Use the RAM logging device for the syslogging
 *   interface.  If this feature is enabled then all debug output (only)
 *   will be re-directed to the circular buffer in RAM.  This RAM log can
 *   be viewed from NSH using the 'dmesg' command.  NOTE:  Unlike the
 *   limited, generic character driver SYSLOG device, the RAMLOG *can* be
 *   used to generate debug output from interrupt level handlers.
 * CONFIG_RAMLOG_NPOLLWAITERS - The number of threads than can be waiting
 *   for this driver on poll().  Default: 4
 *
 * If CONFIG_RAMLOG_SYSLOG is selected, then the following may also be
 * provided:
 *
 * CONFIG_RAMLOG_BUFSIZE - Size of the console RAM log.  Default: 1024
 */

#ifndef CONFIG_RAMLOG_NPOLLWAITERS
#  define CONFIG_RAMLOG_NPOLLWAITERS 4
#endif

#ifndef CONFIG_RAMLOG_BUFSIZE
#  define CONFIG_RAMLOG_BUFSIZE 1024
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ramlog_register
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *   Mostly likely this path will be /dev/console.
 *
 *   This interface is not normally used but can be made available is
 *   someone just wants to tinker with the RAM log as a generic character
 *   device.  Normally both CONFIG_CONSOLE_SYSLOG and CONFIG_RAMLOG_SYSLOG
 *   would be set (to capture all output in the log) -OR- just
 *   CONFIG_RAMLOG_SYSLOG would be set to capture debug output only
 *   in the log.
 *
 ****************************************************************************/

int ramlog_register(FAR const char *devpath,
                    FAR char *buffer, size_t buflen);

/****************************************************************************
 * Name: ramlog_syslog_register
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *   Mostly likely this path will be CONFIG_SYSLOG_DEVPATH
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
void ramlog_syslog_register(void);
#endif

/****************************************************************************
 * Name: ramlog_putc
 *
 * Description:
 *   This is the low-level system logging interface.
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
int ramlog_putc(FAR struct syslog_channel_s *channel, int ch);
#endif

/****************************************************************************
 * Name: ramlog_write
 *
 * Description:
 *   This is the low-level system logging interface.
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
ssize_t ramlog_write(FAR struct syslog_channel_s *channel,
                     FAR const char *buffer, size_t buflen);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RAMLOG */
#endif /* __INCLUDE_NUTTX_SYSLOG_RAMLOG_H */
