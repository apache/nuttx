/****************************************************************************
 * libs/libc/string/lib_memmem.c
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

#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memmem
 *
 * Description:
 *   The memmem() function finds the start of the first occurrence of the
 *   substring needle of length needlelen in the memory area haystack of
 *   length haystacklen.
 *
 * Returned Value:
 *   The memmem() function returns a pointer to the beginning of the
 *   substring, or NULL if the substring is not found.
 *
 ****************************************************************************/

FAR void *memmem(FAR const void *haystack, size_t haystacklen,
                 FAR const void *needle, size_t needlelen)
{
  FAR const unsigned char *h = haystack;
  FAR const unsigned char *n = needle;
  size_t i;
  size_t y;

  if (needlelen > haystacklen)
    {
      return NULL;
    }

  for (i = 0; i < haystacklen - needlelen; i++)
    {
      y = 0;
      while (h[i + y] == n[y])
        {
          if (++y == needlelen)
            {
              return (void *)(h + i);
            }
        }
    }

  return NULL;
}
