/****************************************************************************
 * drivers/syslog/syslog.h
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

#ifndef __DRIVERS_SYSLOG_SYSLOG_H
#define __DRIVERS_SYSLOG_SYSLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Name: syslog_dev_initialize
 *
 * Description:
 *   Initialize to use the character device (or file) at
 *   CONFIG_SYSLOG_DEVPATH as the SYSLOG sink.
 *
 *   One power up, the SYSLOG facility is non-existent or limited to very
 *   low-level output.  This function may be called later in the
 *   intialization sequence after full driver support has been initialized.
 *   (via syslog_initialize())  It installs the configured SYSLOG drivers
 *   and enables full SYSLOGing capability.
 *
 *   NOTE that this implementation excludes using a network connection as
 *   SYSLOG device.  That would be a good extension.
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
int syslog_dev_initialize(void);
#endif

/****************************************************************************
 * Name: syslog_add_intbuffer
 *
 * Description:
 *   Add one more character to the interrupt buffer.  In the event of
 *   buffer overlowed, the character will be dropped.  The indication
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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __DRIVERS_SYSLOG_SYSLOG_H */
