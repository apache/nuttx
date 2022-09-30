/****************************************************************************
 * libs/libc/string/lib_strverscmp.c
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

#include <ctype.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strverscmp
 *
 * Description:
 *   Often one has files jan1, jan2, ..., jan9, jan10, ... and it feels
 *   wrong when ls orders them jan1, jan10, ..., jan2, ..., jan9. In
 *   order to rectify this, GNU introduced the -v option to ls, which is
 *   implemented using versionsort, which again uses strverscmp().
 *
 *   Thus, the task of strverscmp() is to compare two strings and find the
 *   "right" order, while strcmp finds only the lexicographic order. This
 *   function does not use the locale category LC_COLLATE, so is meant
 *   mostly for situations where the strings are expected to be in ASCII.
 *
 *   What this function does is the following. If both strings are
 *   equal, return 0. Otherwise, find the position between two bytes with
 *   the property that before it both strings are equal, while directly
 *   after it there is a difference. Find the largest consecutive digit
 *   strings containing (or starting at, or ending at) this position.
 *   If one or both of these is empty, then return what strcmp would have
 *   returned (numerical ordering of byte values). Otherwise, compare both
 *   digit strings numerically, where digit strings with one or more
 *   leading zeros are interpreted as if they have a decimal point
 *   in front (so that in particular digit strings with more leading zeros
 *   come before digit strings with fewer leading zeros). Thus, the
 *   ordering is 000, 00, 01, 010, 09, 0, 1, 9, 10.
 *
 * Returned Value:
 *   The strverscmp() function returns an integer less than, equal to, or
 *   greater than zero if s1 is found, respectively, to be earlier than,
 *   equal to, or later than s2.
 *
 ****************************************************************************/

int strverscmp(FAR const char *s1, FAR const char *s2)
{
  FAR const unsigned char *str1 = (FAR const void *)s1;
  FAR const unsigned char *str2 = (FAR const void *)s2;
  size_t i;
  size_t j;
  size_t dp;
  int z = 1;

  /* Find maximal matching prefix and track its maximal digit
   * suffix and whether those digits are all zeros.
   */

  for (dp = i = 0; str1[i] == str2[i]; i++)
    {
      int c = str1[i];

      if (c == 0)
        {
          return 0;
        }

      if (!isdigit(c))
        {
          dp = i + 1;
          z = 1;
        }
      else if (c != '0')
        {
          z = 0;
        }
    }

  if (str1[dp] != '0' && str2[dp] != '0')
    {
      /* If we're not looking at a digit sequence that began
       * with a zero, longest digit string is greater.
       */

      for (j = i; isdigit(str1[j]); j++)
        {
          if (!isdigit(str2[j]))
            {
              return 1;
            }
        }

      if (isdigit(str2[j]))
        {
          return -1;
        }
    }
  else if (z && dp < i && (isdigit(str1[i]) || isdigit(str2[i])))
    {
      /* Otherwise, if common prefix of digit sequence is
       * all zeros, digits order less than non-digits.
       */

      return (unsigned char)(str1[i] - '0') -
             (unsigned char)(str2[i] - '0');
    }

  return str1[i] - str2[i];
}
