/****************************************************************************
 * libs/libc/misc/lib_debug.c
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
#include <debug.h>

#include "libc.h"

#ifndef CONFIG_CPP_HAVE_VARARGS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _alert, _err, _warn, and _info
 *
 * Description:
 *  If the cross-compiler's pre-processor does not support variable
 *  length arguments, then these additional APIs will be built.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_ALERT
void _alert(const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  vsyslog(LOG_EMERG, format, ap);
  va_end(ap);
}
#endif /* CONFIG_DEBUG_ALERT */

#ifdef CONFIG_DEBUG_ERROR
void _err(const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  vsyslog(LOG_ERR, format, ap);
  va_end(ap);
}
#endif /* CONFIG_DEBUG_ERROR */

#ifdef CONFIG_DEBUG_WARN
void _warn(const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  vsyslog(LOG_WARNING, format, ap);
  va_end(ap);
}
#endif /* CONFIG_DEBUG_WARN */

#ifdef CONFIG_DEBUG_INFO
void _info(const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  vsyslog(LOG_INFO, format, ap);
  va_end(ap);
}
#endif /* CONFIG_DEBUG_INFO */

#endif /* CONFIG_CPP_HAVE_VARARGS */
