/****************************************************************************
 * drivers/syslog/syslog_flush.c
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

#include <stdbool.h>
#include <assert.h>

#include <nuttx/syslog/syslog.h>

#include "syslog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_flush
 *
 * Description:
 *   This is called by system crash-handling logic.  It must flush any
 *   buffered data to the SYSLOG device.
 *
 *   Interrupts are disabled at the time of the crash and this logic must
 *   perform the flush using low-level, non-interrupt driven logic.
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   Zero (OK)is returned on  success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#if 0
/* REVISIT: (1) Not yet integrated into assertion handlers and (2) there is
 * an implementation problem in that if a character driver is the underlying
 * device, then there is no mechanism to flush the data buffered in the
 * driver with interrupts disabled.
 */

int syslog_flush(void)
{
  DEBUGASSERT(g_syslog_channel != NULL && g_syslog_channel->sc_flush != NULL);

#ifdef CONFIG_SYSLOG_INTBUFFER
  /* Flush any characters that may have been added to the interrupt
   * buffer.
   */

  (void)syslog_flush_intbuffer(g_syslog_channel, true);
#endif

  /* Then flush all of the buffered output to the SYSLOG device */

  return g_syslog_channel->sc_flush();
}
#endif
