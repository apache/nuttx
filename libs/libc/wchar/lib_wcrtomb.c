/****************************************************************************
 * libs/libc/wchar/lib_wcrtomb.c
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
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcrtomb
 *
 * Description:
 *   Convert a wide character to a multibyte sequence
 *
 ****************************************************************************/

size_t wcrtomb(FAR char *s, wchar_t wc, FAR mbstate_t *ps)
{
  if (s == NULL)
    {
      return 0;
    }
  else if ((unsigned)wc < 0x80)
    {
      *s = wc;
      return 1;
    }
  else if ((unsigned)wc < 0x800)
    {
      *s++ = 0xc0 | (wc >> 6);
      *s = 0x80 | (wc & 0x3f);
      return 2;
    }
  else if ((unsigned)wc < 0xd800 || (unsigned)wc <= 0xffff)
    {
      *s++ = 0xe0 | (wc >> 12);
      *s++ = 0x80 | ((wc >> 6) & 0x3f);
      *s = 0x80 | (wc & 0x3f);
      return 3;
    }
  else if ((unsigned long)wc < 0x110000)
    {
      *s++ = 0xf0 | ((unsigned long)wc >> 18);
      *s++ = 0x80 | ((wc >> 12) & 0x3f);
      *s++ = 0x80 | ((wc >> 6) & 0x3f);
      *s = 0x80 | (wc & 0x3f);
      return 4;
    }

  set_errno(EILSEQ);
  return -1;
}
