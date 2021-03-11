/****************************************************************************
 * drivers/syslog/vsyslog.c
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

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_SYSLOG_PRIORITY)
static FAR const char * g_priority_str[] =
  {
    "EMERG", "ALERT", "CRIT", "ERROR",
    "WARN", "NOTICE", "INFO", "DEBUG"
  };
#endif

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
#if CONFIG_TASK_NAME_SIZE > 0 && defined(CONFIG_SYSLOG_PROCESS_NAME)
  struct tcb_s *tcb;
#endif
#if defined(CONFIG_SYSLOG_TIMESTAMP_FORMATTED)
  time_t time;
  struct tm tm;
  char date_buf[CONFIG_SYSLOG_TIMESTAMP_BUFFER];
#endif

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
  /* Prepend the message with the current time, if available */

#if defined(CONFIG_SYSLOG_TIMESTAMP_FORMATTED)
  time = ts.tv_sec;
#if defined(CONFIG_SYSLOG_TIMESTAMP_LOCALTIME)
  localtime_r(&time, &tm);
#else
  gmtime_r(&time, &tm);
#endif

  ret = strftime(date_buf, CONFIG_SYSLOG_TIMESTAMP_BUFFER,
                 CONFIG_SYSLOG_TIMESTAMP_FORMAT, &tm);

  if (ret > 0)
    {
      ret = lib_sprintf(&stream.public, "[%s] ", date_buf);
    }
#else
  ret = lib_sprintf(&stream.public, "[%5jd.%06ld] ",
                    (uintmax_t)ts.tv_sec, ts.tv_nsec / 1000);
#endif
#else
  ret = 0;
#endif

#if defined(CONFIG_SYSLOG_PROCESSID)
  /* Prepend the Process ID */

  ret += lib_sprintf(&stream.public, "[%2d] ", (int)getpid());
#endif

#if defined(CONFIG_SYSLOG_COLOR_OUTPUT)
  /* Set the terminal style according to message priority. */

  switch (priority)
    {
      case LOG_EMERG:   /* Red, Bold, Blinking */
        ret += lib_sprintf(&stream.public, "\e[31;1;5m");
        break;

      case LOG_ALERT:   /* Red, Bold */
        ret += lib_sprintf(&stream.public, "\e[31;1m");
        break;

      case LOG_CRIT:    /* Red, Bold */
        ret += lib_sprintf(&stream.public, "\e[31;1m");
        break;

      case LOG_ERR:     /* Red */
        ret += lib_sprintf(&stream.public, "\e[31m");
        break;

      case LOG_WARNING: /* Yellow */
        ret += lib_sprintf(&stream.public, "\e[33m");
        break;

      case LOG_NOTICE:  /* Bold */
        ret += lib_sprintf(&stream.public, "\e[1m");
        break;

      case LOG_INFO:    /* Normal */
        break;

      case LOG_DEBUG:   /* Dim */
        ret += lib_sprintf(&stream.public, "\e[2m");
        break;
    }
#endif

#if defined(CONFIG_SYSLOG_PRIORITY)
  /* Prepend the message priority. */

  ret += lib_sprintf(&stream.public, "[%6s] ", g_priority_str[priority]);
#endif

#if defined(CONFIG_SYSLOG_PREFIX)
  /* Prepend the prefix, if available */

  ret += lib_sprintf(&stream.public, "%s", CONFIG_SYSLOG_PREFIX_STRING);
#endif

#if CONFIG_TASK_NAME_SIZE > 0 && defined(CONFIG_SYSLOG_PROCESS_NAME)
  /* Prepend the process name */

  tcb = nxsched_get_tcb(getpid());
  ret += lib_sprintf(&stream.public, "%s: ", tcb->name);
#endif

  /* Generate the output */

  ret += lib_vsprintf(&stream.public, fmt, *ap);

#if defined(CONFIG_SYSLOG_COLOR_OUTPUT)
  /* Reset the terminal style back to normal. */

  ret += lib_sprintf(&stream.public, "\e[0m");
#endif

#ifdef CONFIG_SYSLOG_BUFFER
  /* Flush and destroy the syslog stream buffer */

  if (priority != LOG_EMERG)
    {
      syslogstream_destroy(&stream);
    }
#endif

  return ret;
}
