/****************************************************************************
 * libs/libc/misc/lib_fnmatch.c
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * SPDX-FileCopyrightText: 1995, 2000 by Jef Poskanzer <jef@mail.acme.com>.
 * SPDX-FileContributor: with extensions by Ken Pettit
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <ctype.h>
#include <fnmatch.h>
#include <string.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fnmatch_eq
 *
 * Description:
 *   Compare one character of the pattern with one of the string.  With
 *   FNM_CASEFOLD, upper and lower case are the same.
 *
 ****************************************************************************/

static inline int fnmatch_eq(char p, char c, int flags)
{
  if (p == c)
    {
      return 1;
    }

  return (flags & FNM_CASEFOLD) != 0 &&
         tolower((unsigned char)p) == tolower((unsigned char)c);
}

/****************************************************************************
 * Name: fnmatch_range
 *
 * Description:
 *   Return non-zero if c is in the range first-last.  With FNM_CASEFOLD,
 *   the other case of c also matches.
 *
 ****************************************************************************/

static inline int fnmatch_range(char first, char last, char c,
                                int flags)
{
  char lc;
  char uc;

  if (c >= first && c <= last)
    {
      return 1;
    }

  if ((flags & FNM_CASEFOLD) == 0)
    {
      return 0;
    }

  lc = tolower((unsigned char)c);
  uc = toupper((unsigned char)c);
  return (lc >= first && lc <= last) || (uc >= first && uc <= last);
}

/****************************************************************************
 * Name: fnmatch_one
 *
 * Description:
 *   Does all of the work for one '|' delimited pattern
 *
 * Returned Value:
 *   Returns 0 (match) or 1 (no-match).
 *
 ****************************************************************************/

static int fnmatch_one(FAR const char *pattern, int patlen,
                       FAR const char *string, int flags)
{
  FAR const char *p;
  char first;
  int pl;
  int i;

  for (p = pattern; p - pattern < patlen; p++, string++)
    {
      if (*p == '?' && *string != '\0')
        {
          continue;
        }

      /* Match single character from a set:  "[a-zA-Z]" for instance */

      if (*p == '[' && *string != '\0')
        {
          /* 'i' tells if a character or a range of the set matches */

          i = 0;
          p++;
          while (*p != ']' && *p != '\0')
            {
              first = *p++;
              if (*p == '-' && p[1] != ']' && p[1] != '\0')
                {
                  if (fnmatch_range(first, p[1], *string, flags))
                    {
                      i = 1;
                    }

                  p += 2;
                }
              else if (fnmatch_eq(first, *string, flags))
                {
                  i = 1;
                }
            }

          if (i == 0 || *p == '\0')
            {
              return FNM_NOMATCH;
            }

          continue;
        }

      if (*p == '*')
        {
          p++;
          if (*p == '*')
            {
              /* Double-wildcard matches anything. */

              p++;
              i = strlen(string);
            }
          else
            {
              /* Single-wildcard matches anything but slash. */

              i = strcspn(string, "/");
            }

          pl = patlen - (p - pattern);
          for (; i >= 0; i--)
            {
              if (fnmatch_one(p, pl, &string[i], flags) == 0)
                {
                  return 0;
                }
            }

          return FNM_NOMATCH;
        }

      if (!fnmatch_eq(*p, *string, flags))
        {
          return FNM_NOMATCH;
        }
    }

  /* With FNM_LEADING_DIR, the pattern can match the string up to a '/' */

  if (*string == '\0' ||
      ((flags & FNM_LEADING_DIR) != 0 && *string == '/'))
    {
      return 0;
    }

  return FNM_NOMATCH;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fnmatch
 *
 * Description:
 *   Simple shell-style filename pattern matcher originally written by
 *   Jef Poskanzer and extended by Ken Pettit.  This pattern matcher handles
 *   '?', '*', '**', sets like [a-zA-z], and multiple  patterns separated
 *   by '|'.
 *
 *   Of the flags, it handles FNM_CASEFOLD (upper and lower case are the
 *   same) and FNM_LEADING_DIR (the pattern can match the string up to a
 *   '/').
 *
 * Returned Value:
 *   Returns 0 (match) or 1 (no-match).
 *
 ****************************************************************************/

int fnmatch(FAR const char *pattern, const char *string, int flags)
{
  FAR const char *or;

  for (; ; )
    {
      or = strchr(pattern, '|');
      if (or == NULL)
        {
          return fnmatch_one(pattern, strlen(pattern), string, flags);
        }

      if (fnmatch_one(pattern, or - pattern, string, flags) == 0)
        {
          return 0;
        }

      pattern = or + 1;
    }
}
