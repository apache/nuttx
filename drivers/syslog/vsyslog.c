/****************************************************************************
 * drivers/syslog/vsyslog.c
 *
 *   Copyright (C) 2007-2009, 2011-2014, 2016 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <syslog.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _vsyslog
 *
 * Description:
 *   _vsyslog() handles the system logging system calls. It is functionally
 *   equivalent to vsyslog() except that (1) the per-process priority
 *   filtering has already been performed and the va_list parameter is
 *   passed by reference.  That is because the va_list is a structure in
 *   some compilers and passing of structures in the NuttX sycalls does
 *   not work.
 *
 ****************************************************************************/

int _vsyslog(int priority, FAR const IPTR char *fmt, FAR va_list *ap)
{
  struct lib_outstream_s stream;
#ifdef CONFIG_SYSLOG_TIMESTAMP
  struct timespec ts;

  /* Get the current time.  Since debug output may be generated very early
   * in the start-up sequence, hardware timer support may not yet be
   * available.
   */

  if (!OSINIT_HW_READY() || clock_systimespec(&ts) < 0)
    {
      /* Timer hardware is not available, or clock_systimespec failed */

      ts.tv_sec  = 0;
      ts.tv_nsec = 0;
    }
#endif

  /* Wrap the low-level output in a stream object and let lib_vsprintf
   * do the work.  NOTE that emergency priority output is handled
   * differently.. it will use the SYSLOG emergency stream.
   */

  if (priority == LOG_EMERG)
    {
      /* Use the SYSLOG emergency stream */

      emergstream((FAR struct lib_outstream_s *)&stream);
    }
  else
    {
      /* Use the normal SYSLOG stream */

      syslogstream((FAR struct lib_outstream_s *)&stream);
    }

#if defined(CONFIG_SYSLOG_TIMESTAMP)
  /* Pre-pend the message with the current time, if available */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,
                    "[%6d.%06d]", ts.tv_sec, ts.tv_nsec/1000);
#endif

  return lib_vsprintf((FAR struct lib_outstream_s *)&stream, fmt, *ap);
}
