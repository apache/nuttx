/****************************************************************************
 * drivers/syslog/syslog.h
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

#ifndef __DRIVERS_SYSLOG_SYSLOG_H
#define __DRIVERS_SYSLOG_SYSLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

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

/* This is the current syslog channel in use.  It initially points to
 * g_default_channel.
 */

struct syslog_channel_s; /* Forward reference */
EXTERN FAR struct syslog_channel_s *g_syslog_channel
                                                [CONFIG_SYSLOG_MAX_CHANNELS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_dev_initialize
 *
 * Description:
 *   Initialize to use the character device (or file) at
 *   CONFIG_SYSLOG_DEVPATH as the SYSLOG sink.
 *
 *   One power up, the SYSLOG facility is non-existent or limited to very
 *   low-level output.  This function may be called later in the
 *   initialization sequence after full driver support has been initialized.
 *   (via syslog_initialize())  It installs the configured SYSLOG drivers
 *   and enables full SYSLOGing capability.
 *
 *   NOTE that this implementation excludes using a network connection as
 *   SYSLOG device.  That would be a good extension.
 *
 * Input Parameters:
 *   devpath - The full path to the character device to be used.
 *   oflags  - File open flags.
 *   mode    - File open mode (only if oflags include O_CREAT).
 *
 * Returned Value:
 *   Returns a newly created SYSLOG channel, or NULL in case of any failure.
 *
 ****************************************************************************/

FAR struct syslog_channel_s *syslog_dev_initialize(FAR const char *devpath,
                                                   int oflags, int mode);

/****************************************************************************
 * Name: syslog_dev_uninitialize
 *
 * Description:
 *   Called to disable the last device/file channel in preparation to use
 *   a different SYSLOG device. Currently only used for CONFIG_SYSLOG_FILE.
 *
 * Input Parameters:
 *   channel    - Handle to syslog channel to be used.
 *
 * Assumptions:
 *   The caller has already switched the SYSLOG source to some safe channel
 *   (the default channel).
 *
 ****************************************************************************/

void syslog_dev_uninitialize(FAR struct syslog_channel_s *channel);

/****************************************************************************
 * Name: syslog_dev_channel
 *
 * Description:
 *   Configure to use the character device (or file) at
 *   CONFIG_SYSLOG_DEVPATH as the SYSLOG channel.
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
 *   A pointer to the new SYSLOG channel; NULL is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CHAR
FAR struct syslog_channel_s *syslog_dev_channel(void);
#endif

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
 *   A pointer to the new SYSLOG channel; NULL is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CONSOLE
FAR struct syslog_channel_s *syslog_console_channel(void);
#endif

/****************************************************************************
 * Name: syslog_register
 *
 * Description:
 *   Register a simple character driver at /dev/log whose write() method
 *   will transfer data to the SYSLOG device.  This can be useful if, for
 *   example, you want to redirect the output of a program to the SYSLOG.
 *
 *   NOTE that unlike other syslog output, this data is unformatted raw
 *   byte output with no time-stamping or any other SYSLOG features
 *   supported.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CHARDEV
void syslog_register(void);
#endif

/****************************************************************************
 * Name: syslog_add_intbuffer
 *
 * Description:
 *   Add one more character to the interrupt buffer.  In the event of
 *   buffer overflowed, the character will be dropped.  The indication
 *   "[truncated]\n" will be appended to the end of the interrupt buffer.
 *
 * Input Parameters:
 *   ch - The character to add to the interrupt buffer (must be positive).
 *
 * Returned Value:
 *   Zero success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 * Assumptions:
 *   Called only from interrupt handling logic; Interrupts will be disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_INTBUFFER
int syslog_add_intbuffer(int ch);
#endif

/****************************************************************************
 * Name: syslog_flush_intbuffer
 *
 * Description:
 *   Flush any characters that may have been added to the interrupt buffer
 *   to the SYSLOG device.
 *
 * Input Parameters:
 *   force   - Use the force() method of the channel vs. the putc() method.
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 * Assumptions:
 *   Interrupts may or may not be disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_INTBUFFER
int syslog_flush_intbuffer(bool force);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __DRIVERS_SYSLOG_SYSLOG_H */
