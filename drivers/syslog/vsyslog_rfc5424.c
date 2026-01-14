/****************************************************************************
 * drivers/syslog/vsyslog_rfc5424.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/init.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

#include "syslog.h"

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* RFC5424 NILVALUE encoding */

#define NILVALUE "-"
#define NILVALUE_SPACE NILVALUE " "

/* RFC5424 timestamp format string (fractional seconds and 'Z' added by
 * appending to format string)
 */

#define RFC5424_STRFTIME "%Y-%m-%dT%H:%M:%S"

/* RFC5424 structured data options were selected */

#ifdef CONFIG_SYSLOG_RFC5424_TIMEQUALITY
#  define HAVE_RFC5424_SDATA 1
#else
#  define HAVE_RFC5424_SDATA 0
#endif /* defined(CONFIG_SYSLOG_RFC5424_TIMEQUALITY) || ... */

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  struct lib_syslograwstream_s stream;
  int ret = 0;
#ifdef CONFIG_SYSLOG_PROCESS_NAME
  FAR struct tcb_s *tcb = nxsched_self();
#endif
#ifdef CONFIG_SYSLOG_TIMESTAMP
  struct timespec ts =
  {
    0
  };

  struct tm tm =
  {
    0
  };

  char date_buf[64];
#endif
#ifdef CONFIG_SYSLOG_RFC5424_HOSTNAME
  char hostname_buf[HOST_NAME_MAX + 1];
#endif

  /* Wrap the low-level output in a stream object and let lib_vsprintf
   * do the work.
   */

  lib_syslograwstream_open(&stream);

#ifdef CONFIG_SYSLOG_TIMESTAMP

  /* Get the current time.  Since debug output may be generated very early
   * in the start-up sequence, hardware timer support may not yet be
   * available.
   */

  if (OSINIT_HW_READY())
    {
      /* Use CLOCK_REALTIME if so configured */

      clock_gettime(CLOCK_REALTIME, &ts);

      /* Prepend the message with the current time, if available */

      gmtime_r(&ts.tv_sec, &tm);
    }

  /* RFC 5424 compatible syslog output, use the required format string */

  strftime(date_buf, sizeof(date_buf), RFC5424_STRFTIME, &tm);
#endif /* CONFIG_SYSLOG_TIMESTAMP */

  /* Get the host name string. NuttX configures this to be empty by default,
   * so if there is just an empty string, we'll set it to the NILVALUE to
   * stay within RFC 5424 spec.
   */

#ifdef CONFIG_SYSLOG_RFC5424_HOSTNAME
  gethostname(hostname_buf, sizeof(hostname_buf));
  if (hostname_buf[0] == '\0')
    {
      memcpy(hostname_buf, NILVALUE, sizeof(NILVALUE));
    }
#endif

  /* Output the RFC5424 header */

  ret = lib_sprintf_internal(&stream.common,

      /* Start of format string */

      "<%d>" /* PRI */
      "1 "   /* VERSION */

  /* End of format string */

#ifdef CONFIG_SYSLOG_TIMESTAMP
      "%s.%06ldZ " /* TIMESTAMP */
#else
      NILVALUE_SPACE /* NO TIMESTAMP */
#endif
#ifdef CONFIG_SYSLOG_RFC5424_HOSTNAME
      "%s " /* HOSTNAME */
#else
      NILVALUE_SPACE /* NO HOSTNAME */
#endif
#ifdef CONFIG_SYSLOG_PROCESS_NAME
      "%s " /* APPNAME */
#else
      NILVALUE_SPACE /* NO APPNAME */
#endif
#ifdef CONFIG_SYSLOG_PROCESSID
      "%d " /* PROCID */
#else
      NILVALUE_SPACE /* NO PROCID */
#endif
      NILVALUE_SPACE /* TODO: MSGID */
#if !HAVE_RFC5424_SDATA
      NILVALUE_SPACE /* Empty structured data, print the NILVALUE here to
                      * save `libsprintf` call */
#endif
#ifdef CONFIG_SYSLOG_RFC5424_TIMEQUALITY
      "[timeQuality isSynced=\"%d\" tzKnown=\"%d\"]"
#endif
#if HAVE_RFC5424_SDATA
      " " /* Space at the end of structured data before message */
#endif

      /* Beginning of formatted arguments */

      , priority /* PRIVAL */
#ifdef CONFIG_SYSLOG_TIMESTAMP
      , date_buf, ts.tv_nsec / NSEC_PER_USEC /* TIMESTAMP */
#endif
#ifdef CONFIG_SYSLOG_RFC5424_HOSTNAME
      , hostname_buf /* HOSTNAME */
#endif
#ifdef CONFIG_SYSLOG_PROCESS_NAME
      , get_task_name(tcb)
#endif
#ifdef CONFIG_SYSLOG_PROCESSID
      , nxsched_gettid() /* PROCID */
#endif
      /* TODO: MSGID */

      /* Formatted arguments for structured data */

#ifdef CONFIG_SYSLOG_RFC5424_TIMEQUALITY
      , 0 /* TODO: Not sure if synced */
      , 0 /* TODO: Not sure if time zone known */
#endif
      /* End of formatted arguments */
  );

  /* MSG string is generated below from common code for all syslog
   * calls since RFC5424 allows the MSG field to be a free-form string.
   */

  /* Generate the output */

  ret += lib_vsprintf_internal(&stream.common, fmt, *ap);

  if (stream.last_ch != '\n')
    {
      lib_stream_putc(&stream.common, '\n');
      ret++;
    }

  /* Flush and destroy the syslog stream buffer */

  lib_syslograwstream_close(&stream);
  return ret;
}
