/****************************************************************************
 * fs/syslog/fs_setlogmask.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <syslog.h>

#include <arch/irq.h>

#include "syslog/syslog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_ENABLE
/* The initial mask is all disabled */

#  define INITIAL_SYSLOG_MASK 0
#else
/* The initial mask is all enabled */

#  define INITIAL_SYSLOG_MASK LOG_ALL
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The currently enabled set of syslog priorities */

uint8_t g_syslog_mask = INITIAL_SYSLOG_MASK;

#ifdef CONFIG_SYSLOG_ENABLE
/* True if the syslog is enabled */

bool g_syslog_enabled;

/* The set of syslog priorities to use when the syslog is enabled */

uint8_t g_syslog_enablemask = LOG_ALL;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setlogmask
 *
 * Description:
 *  Enable or disable debug output.
 *
 ****************************************************************************/

int setlogmask(int mask)
{
  uint8_t oldmask;
  irqstate_t flags;

  /* These operations must be exclusive with respect to other threads as well
   * as interrupts.
   */

  flags = irqsave();

#ifdef CONFIG_SYSLOG_ENABLE
  /* If the syslog is disabled, use the saved enable mask */

  if (!g_syslog_enabled)
    {
      oldmask             = g_syslog_enablemask;
      g_syslog_enablemask = (uint8_t)mask;
    }
  else
#endif
    {
      oldmask             = g_syslog_mask;
      g_syslog_mask       = (uint8_t)mask;
    }

  irqrestore(flags);
  return oldmask;
}
