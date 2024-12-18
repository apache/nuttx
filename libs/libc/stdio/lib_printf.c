/****************************************************************************
 * libs/libc/stdio/lib_printf.c
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

#include <stdio.h>
#include <syslog.h>

#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: printf
 ****************************************************************************/

int printf(FAR const IPTR char *fmt, ...)
{
  va_list ap;
  int     ret;

  va_start(ap, fmt);

#ifdef CONFIG_SYSLOG_STDOUT_PREFIX
  ret = nx_vsyslog(LOG_NOTICE, fmt, &ap);
#elif defined(CONFIG_SYSLOG_STDOUT)
  struct lib_syslograwstream_s stream;

  /* Wrap the low-level output in a stream object and let lib_vsprintf
   * do the work.
   */

  lib_syslograwstream_open(&stream);

  ret = lib_vsprintf_internal(&stream.common, fmt, ap);

  /* Flush and destroy the syslog stream buffer */

  lib_syslograwstream_close(&stream);

#elif defined(CONFIG_FILE_STREAM)
  ret = vfprintf(stdout, fmt, ap);
#else
  ret = vdprintf(STDOUT_FILENO, fmt, ap);
#endif
  va_end(ap);

  return ret;
}
