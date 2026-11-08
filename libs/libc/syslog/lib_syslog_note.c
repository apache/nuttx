/****************************************************************************
 * libs/libc/syslog/lib_syslog_note.c
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

#include <stdarg.h>
#include <syslog.h>

#include <nuttx/sched_note.h>

/* Define the functions, not the macros of <syslog.h> */

#undef syslog
#undef vsyslog

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog/vsyslog
 *
 * Description:
 *   syslog() and vsyslog() for CONFIG_SYSLOG_TO_SCHED_NOTE.  Direct calls
 *   use the macros of <syslog.h>; these functions are for code that takes
 *   the address of syslog() or vsyslog().
 *
 ****************************************************************************/

void vsyslog(int priority, FAR const IPTR char *fmt, va_list ap)
{
  va_list copy;

  /* sched_note_vprintf() takes the address of its va_list.  Where va_list
   * is an array type (x86_64), a va_list parameter is a pointer, so give
   * it a local copy.
   */

  va_copy(copy, ap);
  sched_note_vprintf(NOTE_TAG_LOG + priority, fmt, copy);
  va_end(copy);
}

void syslog(int priority, FAR const IPTR char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vsyslog(priority, fmt, ap);
  va_end(ap);
}
