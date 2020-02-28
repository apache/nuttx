/****************************************************************************
 * drivers/syslog/syslog.h
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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
EXTERN FAR const struct syslog_channel_s *g_syslog_channel;

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
 *   oflags  - File open flags
 *   mode    - File open mode (only if oflags include O_CREAT)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int syslog_dev_initialize(FAR const char *devpath, int oflags, int mode);

/****************************************************************************
 * Name: syslog_dev_uninitialize
 *
 * Description:
 *   Called to disable the last device/file channel in preparation to use
 *   a different SYSLOG device. Currently only used for CONFIG_SYSLOG_FILE.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   The caller has already switched the SYSLOG source to some safe channel
 *   (the default channel).
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_FILE
void syslog_dev_uninitialize(void);
#endif /* CONFIG_SYSLOG_FILE */

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
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CHAR
int syslog_dev_channel(void);
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
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CONSOLE
int syslog_console_channel(void);
#endif

/****************************************************************************
 * Name: syslog_register
 *
 * Description:
 *   Register a simple character driver at /dev/syslog whose write() method
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
 *   channel - The syslog channel to use in performing the flush operation.
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
int syslog_flush_intbuffer(FAR const struct syslog_channel_s *channel,
                           bool force);
#endif

/****************************************************************************
 * Name: syslog_putc
 *
 * Description:
 *   This is the low-level, single character, system logging interface.
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

int syslog_putc(int ch);

/****************************************************************************
 * Name: syslog_write
 *
 * Description:
 *   This is the low-level, multiple character, system logging interface.
 *
 * Input Parameters:
 *   buffer - The buffer containing the data to be output
 *   buflen - The number of bytes in the buffer
 *
 * Returned Value:
 *   On success, the number of characters written is returned.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

ssize_t syslog_write(FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Name: syslog_force
 *
 * Description:
 *   This is the low-level system logging interface.  This version forces
 *   the output and is only used in emergency situations (e.g., in assertion
 *   handling).
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller. A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int syslog_force(int ch);

/****************************************************************************
 * Name: syslog_dev_write
 *
 * Description:
 *   This is the low-level, multiple byte, system logging interface provided
 *   for the character driver interface.
 *
 * Input Parameters:
 *   buffer - The buffer containing the data to be output
 *   buflen - The number of bytes in the buffer
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller. A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

ssize_t syslog_dev_write(FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Name: syslog_dev_putc
 *
 * Description:
 *   This is the low-level system logging interface provided for the
 *   character driver interface.
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

int syslog_dev_putc(int ch);

/****************************************************************************
 * Name: syslog_dev_flush
 *
 * Description:
 *   Flush any buffer data in the file system to media.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int syslog_dev_flush(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __DRIVERS_SYSLOG_SYSLOG_H */
