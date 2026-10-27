/****************************************************************************
 * libs/libc/wchar/lib_wcsdup.c
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

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcsdup
 *
 * Description:
 *   Return a copy of the wide-character string s in memory from malloc(),
 *   or NULL if there is not enough memory.
 *
 ****************************************************************************/

FAR wchar_t *wcsdup(FAR const wchar_t *s)
{
  size_t size = (wcslen(s) + 1) * sizeof(wchar_t);
  FAR wchar_t *news = lib_malloc(size);

  if (news != NULL)
    {
      wmemcpy(news, s, size / sizeof(wchar_t));
    }

  return news;
}
