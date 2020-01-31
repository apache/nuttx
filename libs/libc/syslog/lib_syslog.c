/****************************************************************************
 * libs/libc/syslog/lib_syslog.c
 *
 *   Copyright (C) 2007-2009, 2011-2014, 2016, 2018 Gregory Nutt. All rights
 *     reserved.
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

#include <stdarg.h>
#include <syslog.h>

#include <nuttx/syslog/syslog.h>

#include "syslog/syslog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsyslog
 *
 * Description:
 *   The function vsyslog() performs the same task as syslog() with the
 *   difference that it takes a set of arguments which have been obtained
 *   using the stdarg variable argument list macros.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void vsyslog(int priority, FAR const IPTR char *fmt, va_list ap)
{
  /* Check if this priority is enabled */

  if ((g_syslog_mask & LOG_MASK(priority)) != 0)
    {
      /* Yes.. Perform the nx_vsyslog system call.
       *
       * NOTE:  The va_list parameter is passed by reference.  That is
       * because the va_list is a structure in some compilers and passing
       * of structures in the NuttX syscalls does not work.
       */

#ifdef va_copy
      va_list copy;

      va_copy(copy, ap);
      nx_vsyslog(priority, fmt, &copy);
      va_end(copy);
#else
      nx_vsyslog(priority, fmt, &ap);
#endif
    }
}

/****************************************************************************
 * Name: syslog
 *
 * Description:
 *   syslog() generates a log message. The priority argument is formed by
 *   ORing the facility and the level values (see include/syslog.h). The
 *   remaining arguments are a format, as in printf and any arguments to the
 *   format.
 *
 *   The NuttX implementation does not support any special formatting
 *   characters beyond those supported by printf.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void syslog(int priority, FAR const IPTR char *fmt, ...)
{
  va_list ap;

  /* Let vsyslog do the work */

  va_start(ap, fmt);
  vsyslog(priority, fmt, ap);
  va_end(ap);
}
