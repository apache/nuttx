/****************************************************************************
 * libs/libc/wchar/lib_wcscasecmp.c
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
 * Name: wcscasecmp
 *
 * Description:
 *   Compare the wide-character strings s1 and s2 as wcscmp() does, but
 *   without the difference of case (towlower() of each wide character).
 *
 ****************************************************************************/

int wcscasecmp(FAR const wchar_t *s1, FAR const wchar_t *s2)
{
  wint_t c1;
  wint_t c2;

  do
    {
      c1 = towlower(*s1++);
      c2 = towlower(*s2++);
    }
  while (c1 == c2 && c1 != L'\0');

  return c1 < c2 ? -1 : c1 > c2;
}
