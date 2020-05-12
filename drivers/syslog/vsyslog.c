/****************************************************************************
 * drivers/syslog/vsyslog.c
 *
 *   Copyright (C) 2007-2009, 2011-2014, 2016-2017 Gregory Nutt.
 *   All rights reserved.
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
#include <errno.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_vsyslog
 *
 * Description:
 *   nx_vsyslog() handles the system logging system calls. It is functionally
 *   equivalent to vsyslog() except that (1) the per-process priority
 *   filtering has already been performed and the va_list parameter is
 *   passed by reference.  That is because the va_list is a structure in
 *   some compilers and passing of structures in the NuttX sycalls does
 *   not work.
 *
 ****************************************************************************/

int nx_vsyslog(int priority, FAR const IPTR char *fmt, FAR va_list *ap)
{
  struct lib_syslogstream_s stream;
  int ret;

#ifdef CONFIG_SYSLOG_TIMESTAMP
  struct timespec ts;

  /* Get the current time.  Since debug output may be generated very early
   * in the start-up sequence, hardware timer support may not yet be
   * available.
   */

  ret = -EAGAIN;
  if (OSINIT_HW_READY())
    {
#if defined(CONFIG_SYSLOG_TIMESTAMP_REALTIME)
      /* Use CLOCK_REALTIME if so configured */

      ret = clock_gettime(CLOCK_REALTIME, &ts);

#elif defined(CONFIG_CLOCK_MONOTONIC)
      /* Prefer monotonic when enabled, as it can be synchronized to
       * RTC with clock_resynchronize.
       */

      ret = clock_gettime(CLOCK_MONOTONIC, &ts);

#else
      /* Otherwise, fall back to the system timer */

      ret = clock_systime_timespec(&ts);
#endif
    }

  if (ret < 0)
    {
      /* Timer hardware is not available, or clock function failed */

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

      emergstream(&stream.public);
    }
  else
    {
      /* Use the normal SYSLOG stream */

      syslogstream_create(&stream);
    }

#if defined(CONFIG_SYSLOG_TIMESTAMP)
  /* Pre-pend the message with the current time, if available */

  ret = lib_sprintf(&stream.public, "[%5d.%06d] ",
                    ts.tv_sec, ts.tv_nsec / 1000);
#else
  ret = 0;
#endif

#if defined(CONFIG_SYSLOG_PREFIX)
  /* Pre-pend the prefix, if available */

  ret += lib_sprintf(&stream.public, "%s", CONFIG_SYSLOG_PREFIX_STRING);
#endif

  /* Generate the output */

  ret += lib_vsprintf(&stream.public, fmt, *ap);

#ifdef CONFIG_SYSLOG_BUFFER
  /* Flush and destroy the syslog stream buffer */

  if (priority != LOG_EMERG)
    {
      syslogstream_destroy(&stream);
    }
#endif

  return ret;
}
