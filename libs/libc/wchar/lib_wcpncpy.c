/****************************************************************************
 * libs/libc/wchar/lib_wcpncpy.c
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
 * Name: wcpncpy
 *
 * Description:
 *   Copy at most n wide characters of src to dest, and fill the rest of the
 *   n wide characters of dest with null wide characters.  Return a pointer
 *   to the first null wide character written to dest, or dest + n if src
 *   has n or more wide characters (then dest is not terminated).
 *
 ****************************************************************************/

FAR wchar_t *wcpncpy(FAR wchar_t *dest, FAR const wchar_t *src, size_t n)
{
  size_t len = wcsnlen(src, n);

  wmemcpy(dest, src, len);
  wmemset(dest + len, L'\0', n - len);
  return dest + len;
}
