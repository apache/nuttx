/****************************************************************************
 * drivers/syslog/syslog.h
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

#ifndef __DRIVERS_SYSLOG_SYSLOG_H
#define __DRIVERS_SYSLOG_SYSLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* This is the current syslog channel in use.  It initially points to
 * g_default_channel.
 */

#ifdef CONFIG_SYSLOG
extern FAR syslog_channel_t *
#ifndef CONFIG_SYSLOG_REGISTER
const
#endif
g_syslog_channel[CONFIG_SYSLOG_MAX_CHANNELS];

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_openlog
 *
 * Description:
 *   Get the ident that the task group of the caller set with openlog(), and
 *   write the text after it to 'tail': "[pid]: " with LOG_PID, ": " for a
 *   non-empty ident, else nothing.  The messages of the OS that run in the
 *   task get the ident too.  The task information can be in user memory:
 *   print the ident with a precision of OPENLOG_IDENT_SIZE - 1.  No ident
 *   in an interrupt handler.
 *
 * Returned Value:
 *   The ident, or "" when there is nothing to print.
 *
 ****************************************************************************/

static inline FAR const char *syslog_openlog(FAR char *tail, size_t size)
{
  FAR struct task_info_s *info;
  FAR struct tcb_s *tcb;

  tail[0] = '\0';
  if (up_interrupt_context())
    {
      return "";
    }

  tcb = nxsched_self();
  if (tcb == NULL || tcb->group == NULL || tcb->group->tg_info == NULL)
    {
      return "";
    }

  info = tcb->group->tg_info;
  if ((info->ta_openlog.ol_option & LOG_PID) != 0)
    {
      snprintf(tail, size, "[%d]: ", (int)tcb->group->tg_pid);
    }
  else if (info->ta_openlog.ol_ident[0] != '\0')
    {
      snprintf(tail, size, ": ");
    }
  else
    {
      return "";
    }

  return info->ta_openlog.ol_ident;
}

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

FAR syslog_channel_t *syslog_dev_initialize(FAR const char *devpath,
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

void syslog_dev_uninitialize(FAR syslog_channel_t *channel);

/****************************************************************************
 * Name: syslog_dev_channel
 *
 * Description:
 *   Configure to use the character device (or file) at
 *   CONFIG_SYSLOG_DEVPATH as the SYSLOG channel.
 *
 *   This tiny function is simply a wrapper around syslog_dev_initialize()
 *   and syslog_channel_register().  It calls syslog_dev_initialize() to
 *   configure the character device at CONFIG_SYSLOG_DEVPATH then calls
 *   syslog_channel_register() to use that device as the SYSLOG output
 *   channel.
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
FAR syslog_channel_t *syslog_dev_channel(void);
#endif

/****************************************************************************
 * Name: syslog_console_channel
 *
 * Description:
 *   Configure to use the character device (or file) at /dev/console as the
 *   SYSLOG channel.
 *
 *   This tiny function is simply a wrapper around syslog_dev_initialize()
 *   and syslog_channel_register().  It calls syslog_dev_initialize() to
 *   configure the character device at /dev/console then calls
 *   syslog_channel_register() to use that device as the SYSLOG output
 *   channel.
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
FAR syslog_channel_t *syslog_console_channel(void);
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
 *   None
 *
 * Assumptions:
 *   Called only from interrupt handling logic; Interrupts will be disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_INTBUFFER
void syslog_add_intbuffer(FAR const char *buffer, size_t buflen);
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
 *   None
 *
 * Assumptions:
 *   Interrupts may or may not be disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_INTBUFFER
void syslog_flush_intbuffer(bool force);
#endif

/****************************************************************************
 * Name: syslog_write_foreach
 *
 * Description:
 *   This provides a default write method for syslog devices that do not
 *   support multiple byte writes  This functions simply loops, outputting
 *   one character at a time.
 *
 * Input Parameters:
 *   buffer - The buffer containing the data to be output
 *   buflen - The number of bytes in the buffer
 *   force  - Use the force() method of the channel vs. the putc() method.
 *
 * Returned Value:
 *   On success, the number of characters written is returned.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

ssize_t syslog_write_foreach(FAR const char *buffer,
                             size_t buflen, bool force);
#endif /* CONFIG_SYSLOG */

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __DRIVERS_SYSLOG_SYSLOG_H */
