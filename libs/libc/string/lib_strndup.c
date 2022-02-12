/****************************************************************************
 * libs/libc/string/lib_strndup.c
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

#include <string.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strndup
 *
 * Description:
 *   The strndup() function is equivalent to the strdup() function,
 *   duplicating the provided 's' in a new block of memory allocated as
 *   if by using malloc(), with the exception being that strndup() copies
 *   at most 'size' plus one bytes into the newly allocated memory,
 *   terminating the new string with a NUL character. If the length of 's'
 *   is larger than 'size', only 'size' bytes will be duplicated. If
 *   'size' is larger than the length of 's', all bytes in s will be
 *   copied into the new memory buffer, including the terminating NUL
 *   character. The newly created string will always be properly
 *   terminated.
 *
 ****************************************************************************/

#undef strndup /* See mm/README.txt */
FAR char *strndup(FAR const char *s, size_t size)
{
  FAR char *news = NULL;

  /* Get the size of the new string (limited to size) */

  size_t allocsize = strnlen(s, size);

  /* Allocate the new string, adding 1 for the NUL terminator */

  news = (FAR char *)lib_malloc(allocsize + 1);
  if (news)
    {
      /* Copy the string into the allocated memory and add a NUL
       * terminator in any case.
       */

      memcpy(news, s, allocsize);
      news[allocsize] = '\0';
    }

  return news;
}
