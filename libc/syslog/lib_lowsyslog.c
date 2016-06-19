/****************************************************************************
 * lib/syslog/lib_lowsyslog.c
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

#include <syslog.h>

#include <nuttx/syslog/syslog.h>

#include "syslog/syslog.h"

#if defined(CONFIG_ARCH_LOWPUTC) || defined(CONFIG_SYSLOG)
/* The low-level SYSLOG functions can be used only if we have access to
 * either the low-level serial interface, up_putc().
 */

#if defined(CONFIG_BUILD_FLAT) || defined (__KERNEL__)
/* The low-level serial interface, up_putc(), is only available in the FLAT
 * build or during the kernel pass of the protected or kernel two pass
 * builds.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lowvsyslog
 *
 * Description:
 *   The function lowvsyslog() performs the same task as lowsyslog() with
 *   the difference that it takes a set of arguments which have been
 *   obtained using the stdarg variable argument list macros.
 *
 ****************************************************************************/

int lowvsyslog(int priority, FAR const IPTR char *fmt, va_list ap)
{
  int ret = 0;

  /* Check if this priority is enabled */

  if ((g_syslog_mask & LOG_MASK(priority)) != 0)
    {
      /* Perform the _lowvsyslog system call */

      ret = _lowvsyslog(fmt, ap);
    }

  return ret;
}

/****************************************************************************
 * Name: lowsyslog
 *
 * Description:
 *   syslog() generates a log message. The priority argument is formed by
 *   ORing the facility and the level values (see include/syslog.h). The
 *   remaining arguments are a format, as in printf and any arguments to the
 *   format.
 *
 *   This is a non-standard, low-level system logging interface.  The
 *   difference between syslog() and lowsyslog() is that the syslog()
 *   interface writes to the syslog device (usually fd=1, stdout) whereas
 *   lowsyslog() uses a lower level interface that works even from interrupt
 *   handlers.
 *
 ****************************************************************************/

int lowsyslog(int priority, FAR const IPTR char *fmt, ...)
{
  va_list ap;
  int ret;

  /* Let lowvsyslog do the work */

  va_start(ap, fmt);
  ret = lowvsyslog(priority, fmt, ap);
  va_end(ap);

  return ret;
}

#endif /* CONFIG_BUILD_FLAT) || __KERNEL__ */
#endif /* CONFIG_ARCH_LOWPUTC || CONFIG_SYSLOG */
