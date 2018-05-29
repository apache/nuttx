/****************************************************************************
 * lib/syslog/lib_setlogmask.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/irq.h>

#include "syslog/syslog.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The currently enabled set of syslog priorities */

uint8_t g_syslog_mask = LOG_ALL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setlogmask
 *
 * Description:
 *   The setlogmask() function sets the logmask and returns the previous
 *   mask. If the mask argument is 0, the current logmask is not modified.
 *
 *   The SYSLOG priorities are: LOG_EMERG, LOG_ALERT, LOG_CRIT, LOG_ERR,
 *   LOG_WARNING, LOG_NOTICE, LOG_INFO, and LOG_DEBUG.  The bit corresponding
 *   to a priority p is LOG_MASK(p); LOG_UPTO(p) provides the mask of all
 *   priorities in the above list up to and including p.
 *
 *   Per OpenGroup.org "If the maskpri argument is 0, the current log mask
 *   is not modified."  In this implementation, the value zero is permitted
 *   in order to disable all syslog levels.
 *
 * REVISIT: Per POSIX the syslog mask should be a per-process value but in
 * NuttX, the scope of the mask is dependent on the nature of the build:
 *
 *   Flat Build:  There is one, global SYSLOG mask that controls all output.
 *   Protected Build:  There are two SYSLOG masks.  One within the kernel
 *     that controls only kernel output.  And one in user-space that controls
 *     only user SYSLOG output.
 *   Kernel Build:  The kernel build is compliant with the POSIX requirement:
 *     There will be one mask for each user process, controlling the SYSLOG
 *     output only form that process.  There will be a separate mask
 *     accessible only in the kernel code to control kernel SYSLOG output.
 *
 ****************************************************************************/

int setlogmask(int mask)
{
  uint8_t oldmask;
  irqstate_t flags;

  /* These operations must be exclusive with respect to other threads as well
   * as interrupts.
   */

  flags = enter_critical_section();

  oldmask       = g_syslog_mask;
  g_syslog_mask = (uint8_t)mask;

  leave_critical_section(flags);
  return oldmask;
}
