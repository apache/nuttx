/****************************************************************************
 * libs/libc/string/lib_strstr.c
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2015 Tal Einat
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so, subject to the
 * following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE
 * SOFTWARE.
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

#undef strstr /* See mm/README.txt */
FAR char *strstr(FAR const char *str, FAR const char *substr)
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
  for (; ; )
    {
      /* strchr() will return a pointer to the next occurrence of the
       * character ch in the string
       */

      candidate = strchr(candidate, ch);
      if (!candidate || strlen(candidate) < len)
        {
          /* First character of the substring does not appear in the string
           * or the remainder of the string is not long enough to contain the
           * substring.
           */

          return NULL;
        }

      /* Check if this is the beginning of a matching substring */

      if (strncmp(candidate, substr, len) == 0)
        {
          return (FAR char *)candidate;
        }

      /* No, find the next candidate after this one */

      candidate++;
    }

  /* Won't get here, but some compilers might complain.  Other compilers
   * might complain about this code being unreachable too.
   */

  return NULL;
}
