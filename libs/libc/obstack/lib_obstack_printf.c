/****************************************************************************
 * libs/libc/obstack/lib_obstack_printf.c
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

#include <obstack.h>
#include <stdarg.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_printf
 *
 * Description:
 *   This is similar to the asprintf except it uses obstack to allocate
 *   string on. The characters are written onto the end of the currently
 *   growing object and terminated by null byte.
 *
 *   This function is defined in stdio.h in GlibC. There is no definition
 *   that would be in stdio.h required for these here and thus it is easier
 *   to just keep these functions here as user has to include obstack anyway
 *   to get the full functionality.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *   fmt: format string with its format inputs followed.
 *
 * Returned Value:
 *   Number of characters added to the obstack excluding the null byte.
 *
 ****************************************************************************/

int obstack_printf(FAR struct obstack *h, FAR const char *fmt, ...)
{
  int res;
  va_list ap;

  va_start(ap, fmt);
  res = obstack_vprintf(h, fmt, ap);
  va_end(ap);

  return res;
}
