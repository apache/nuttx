/****************************************************************************
 * libs/libc/locale/lib_newlocale.c
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

#include <locale.h>
#include <string.h>

#ifdef CONFIG_LIBC_LOCALE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: newlocale
 *
 * Description:
 *   locales are not supported by NuttX
 *
 ****************************************************************************/

locale_t newlocale(int category_mask, FAR const char *locale, locale_t base)
{
  return !locale || !strcmp(locale, "POSIX") ||
         !strcmp(locale, "C") || !strcmp(locale, "") ? localeconv() : NULL;
}
#endif
