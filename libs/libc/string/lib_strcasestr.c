/****************************************************************************
 * libs/libc/string/lib_strcasestr.c
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
#include <strings.h>
#include <ctype.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#undef strcasechr /* See mm/README.txt */
static FAR char *strcasechr(FAR const char *s, int uc)
{
  register char ch;

  for (; *s != '\0'; s++)
    {
      ch = *s;
      if (toupper(ch) == uc)
        {
          return (FAR char *)s;
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR char *strcasestr(FAR const char *str, FAR const char *substr)
{
  FAR const char *candidate; /* Candidate in str with matching start character */
  char ch;                   /* First character of the substring */
  size_t len;                /* The length of the substring */

  /* Special case the empty substring */

  len = strlen(substr);
  ch  = *substr;

  if (!ch)
    {
      /* We'll say that an empty substring matches at the beginning of
       * the string
       */

      return (FAR char *)str;
    }

  /* Search for the substring */

  candidate = str;
  ch        = toupper(ch);

  for (; ; )
    {
      /* strcasechr() will return a pointer to the next occurrence of the
       * character ch in the string (ignoring case)
       */

      candidate = strcasechr(candidate, ch);
      if (!candidate || strlen(candidate) < len)
        {
          /* First character of the substring does not appear in the string
           * or the remainder of the string is not long enough to contain the
           * substring.
           */

          return NULL;
        }

      /* Check if this is the beginning of a matching substring
       * (ignoring case)
       */

      if (strncasecmp(candidate, substr, len) == 0)
        {
          /* Yes.. return the pointer to the first occurrence of the matching
           * substring.
           */

          return (FAR char *)candidate;
        }

      /* No, find the next candidate after this one */

      candidate++;
    }

  /* Won't get here, but some compilers might complain.  Others might
   * complain about this code being unreachable too.
   */

  return NULL;
}
