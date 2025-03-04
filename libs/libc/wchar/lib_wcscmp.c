/****************************************************************************
 * libs/libc/wchar/lib_wcscmp.c
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

#include <wchar.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcscmp
 *
 * Description:
 *   The wcscmp() function returns zero if the wide-character strings at s1
 *   and s2 are equal. It returns an integer greater than zero if at the
 *   first differing position i, the corresponding wide-character s1[i] is
 *   greater than s2[i].  It returns an integer less than zero if at the
 *   first differing position i, the corresponding wide-character s1[i] is
 *   less than s2[i]
 *
 ****************************************************************************/

int wcscmp(FAR const wchar_t *s1, FAR const wchar_t *s2)
{
  while (*s1 == *s2++)
    {
      if (*s1++ == 0)
        {
          return 0;
        }
    }

  return *s1 - *--s2;
}
