/****************************************************************************
 * libs/libc/string/lib_strpbrk.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef strpbrk /* See mm/README.txt */
FAR char *strpbrk(FAR const char *str, FAR const char *charset)
{
  /* Check each character in the string */

  while (*str)
    {
      /* Check if the character from the string matches any character in the
       * charset
       */

      if (strchr(charset, *str) != NULL)
        {
          /* Yes, then this position must be the first occurrence in string */

          return (FAR char *)str;
        }

      /* This character from the strings matches none of those in the
       * charset. Try the next character from the string.
       */

      str++;
    }

  /* We have looked at every character in the string, and none of them match
   * any of the characters in charset.
   */

  return NULL;
}
