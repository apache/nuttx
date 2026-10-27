/****************************************************************************
 * libs/libc/wchar/lib_wcsncasecmp.c
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
#include <wctype.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcsncasecmp
 *
 * Description:
 *   Compare at most n wide characters of s1 and s2 as wcsncmp() does, but
 *   without the difference of case (towlower() of each wide character).
 *
 ****************************************************************************/

int wcsncasecmp(FAR const wchar_t *s1, FAR const wchar_t *s2, size_t n)
{
  wint_t c1 = 0;
  wint_t c2 = 0;

  while (n-- > 0)
    {
      c1 = towlower(*s1++);
      c2 = towlower(*s2++);
      if (c1 != c2 || c1 == L'\0')
        {
          break;
        }
    }

  return c1 < c2 ? -1 : c1 > c2;
}
