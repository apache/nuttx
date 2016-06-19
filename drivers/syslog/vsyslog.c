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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsyslog_internal
 *
 * Description:
 *   This is the internal implementation of vsyslog (see the description of
 *   syslog and vsyslog below)
 *
 ****************************************************************************/

static inline int vsyslog_internal(FAR const IPTR char *fmt, va_list ap)
{
#if defined(CONFIG_SYSLOG)
  struct lib_outstream_s stream;
#elif CONFIG_NFILE_DESCRIPTORS > 0
  struct lib_rawoutstream_s stream;
#elif defined(CONFIG_ARCH_LOWPUTC)
  struct lib_outstream_s stream;
#endif

#if defined(CONFIG_SYSLOG_TIMESTAMP)
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

#if defined(CONFIG_SYSLOG)
  /* Wrap the low-level output in a stream object and let lib_vsprintf
   * do the work.
   */

  syslogstream((FAR struct lib_outstream_s *)&stream);

#if defined(CONFIG_SYSLOG_TIMESTAMP)
  /* Pre-pend the message with the current time, if available */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,
                    "[%6d.%06d]", ts.tv_sec, ts.tv_nsec/1000);

#endif

  return lib_vsprintf((FAR struct lib_outstream_s *)&stream, fmt, ap);

#elif CONFIG_NFILE_DESCRIPTORS > 0
  /* Wrap the stdout in a stream object and let lib_vsprintf
   * do the work.
   */

  lib_rawoutstream(&stream, 1);

#if defined(CONFIG_SYSLOG_TIMESTAMP)
  /* Pre-pend the message with the current time, if available */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,
                    "[%6d.%06d]",  ts.tv_sec, ts.tv_nsec/1000);
#endif

  return lib_vsprintf(&stream.public, fmt, ap);

#elif defined(CONFIG_ARCH_LOWPUTC)
  /* Wrap the low-level output in a stream object and let lib_vsprintf
   * do the work.
   * REVISIT: lib_lowoutstream() is only available in the FLAT build or
   * the kernel phase of other builds.   
   */

  lib_lowoutstream((FAR struct lib_outstream_s *)&stream);

#if defined(CONFIG_SYSLOG_TIMESTAMP)
  /* Pre-pend the message with the current time, if available */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,
                    "[%6d.%06d]", ts.tv_sec, ts.tv_nsec/1000);
#endif

  return lib_vsprintf((FAR struct lib_outstream_s *)&stream, fmt, ap);

#else /* CONFIG_SYSLOG */

  return 0;

#endif /* CONFIG_SYSLOG */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _vsyslog
 *
 * Description:
 *   _vsyslog() handles the system logging system calls. It is functionally
 *   equivalent to vsyslog() except that the pre-process priority filtering
 *   has already been performed and, hence, there is no priority argument.
 *
 ****************************************************************************/

int _vsyslog(FAR const IPTR char *fmt, va_list ap)
{
  int ret = 0;

#if !defined(CONFIG_SYSLOG) && CONFIG_NFILE_DESCRIPTORS > 0
  /* We are generating output on stdout.  So check if this function was
   * called from an interrupt handler.  We cannot send data to stdout from
   * an interrupt handler.
   */

  if (up_interrupt_context())
    {
#ifdef CONFIG_ARCH_LOWPUTC
      /* But the low-level serial interface up_putc() is provided so we may
       * be able to generate low-level serial output instead.
       * NOTE: The low-level serial output is not necessarily the same
       * output destination as stdout!
       */

      ret = _lowvsyslog(fmt, ap);

#endif /* CONFIG_ARCH_LOWPUTC */
    }
  else
#endif /* !CONFIG_SYSLOG && CONFIG_NFILE_DESCRIPTORS > 0 */
    {
      /* Let vsylog_internal do the deed */

      ret = vsyslog_internal(fmt, ap);
    }

  return ret;
}
