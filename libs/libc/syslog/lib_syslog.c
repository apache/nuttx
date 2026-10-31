/****************************************************************************
 * libs/libc/syslog/lib_syslog.c
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

#include <stdarg.h>
#include <stdio.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/syslog/syslog.h>
#include <nuttx/tls.h>

#include "syslog/syslog.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !defined(__KERNEL__) && defined(va_copy)
/****************************************************************************
 * Name: vsyslog_perror
 *
 * Description:
 *   With LOG_PERROR of openlog(), write the message to stderr too, with the
 *   ident and the process ID before it and a newline after it.
 *
 ****************************************************************************/

static void vsyslog_perror(FAR const IPTR char *fmt, va_list ap)
{
  FAR struct task_info_s *info = task_get_info();
  FAR const IPTR char *end;
  va_list copy;
  int option;

  if (info == NULL
#ifdef CONFIG_BUILD_FLAT
      || up_interrupt_context()
#endif
     )
    {
      return;
    }

  option = info->ta_openlog.ol_option;
  if ((option & LOG_PERROR) == 0)
    {
      return;
    }

  if ((option & LOG_PID) != 0)
    {
      dprintf(STDERR_FILENO, "%.*s[%d]: ", OPENLOG_IDENT_SIZE - 1,
              info->ta_openlog.ol_ident, (int)getpid());
    }
  else if (info->ta_openlog.ol_ident[0] != '\0')
    {
      dprintf(STDERR_FILENO, "%.*s: ", OPENLOG_IDENT_SIZE - 1,
              info->ta_openlog.ol_ident);
    }

  va_copy(copy, ap);
  vdprintf(STDERR_FILENO, fmt, copy);
  va_end(copy);

  for (end = fmt; *end != '\0'; end++)
    {
    }

  if (end == fmt || *(end - 1) != '\n')
    {
      write(STDERR_FILENO, "\n", 1);
    }
}
#endif

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

  if ((g_syslog_mask & LOG_MASK(LOG_PRI(priority))) != 0)
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

#  ifndef __KERNEL__
      vsyslog_perror(fmt, ap);
#  endif
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
