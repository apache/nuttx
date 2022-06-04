/****************************************************************************
 * libs/libc/wchar/lib_wcsftime.c
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

#include <time.h>
#include <wchar.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t wcsftime(FAR wchar_t *s, size_t maxsize, FAR const wchar_t *format,
                FAR const struct tm *tim_p)
{
  /* REVISIT: We can't just use the wide character string... We need to
   * convert it to a normal C string first.
   */

  return strftime((FAR char *)s, maxsize, (FAR char *)format, tim_p);
}
