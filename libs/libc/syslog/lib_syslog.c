/****************************************************************************
 * libs/libc/syslog/lib_syslog.c
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
