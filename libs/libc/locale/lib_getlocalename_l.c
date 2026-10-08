/****************************************************************************
 * libs/libc/locale/lib_getlocalename_l.c
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
#include <locale.h>
#include <stddef.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getlocalename_l
 *
 * Description:
 *   Return the name of the locale of one category of a locale object
 *   (POSIX.1-2024).  LC_GLOBAL_LOCALE means the global locale.  NuttX
 *   supports only the "C" locale, so every category of every locale is
 *   "C".
 *
 * Input Parameters:
 *   category - LC_CTYPE, LC_NUMERIC, ... (not LC_ALL)
 *   locobj   - A locale object, or LC_GLOBAL_LOCALE
 *
 * Returned Value:
 *   The name of the locale, or NULL with errno set to EINVAL if the
 *   category is not valid.
 *
 ****************************************************************************/

FAR const char *getlocalename_l(int category, locale_t locobj)
{
  UNUSED(locobj);

  if (category < 0 || category >= LC_ALL)
    {
      set_errno(EINVAL);
      return NULL;
    }

  return "C";
}
