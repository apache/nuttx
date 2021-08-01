/****************************************************************************
 * libs/libc/misc/lib_fnmatch.c
 *
 * Simple shell-style filename pattern matcher written by Jef Poskanzer
 * This pattern matcher only handles '?', '*' and '**', and  multiple
 * patterns separated by '|'.
 *
 *   Copyright 1995, 2000 by Jef Poskanzer <jef@mail.acme.com>.
 *   All rights reserved.
 *
 * With extensions by Ken Pettit.
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

#include <fnmatch.h>
#include <string.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
          i = 0;
          while (*p != ']' && *p != '\0')
            {
              p++;

              if (*string == *p)
                {
                  /* Match found.  Advance to the ']' */

                  i = 1;
                  while (*p != ']' && *p != '\0')
                    {
                      p++;
                    }

                  break;
                }

              /* Prepare to test for range */

              if (*p != '\0')
                {
                  first = *p++;
                  if (*p == '-')
                    {
                      p++;
                      if (*string >= first && *string <= *p)
                        {
                          /* Match found.  Advance to the ']' */

                          i = 1;
                          while (*p != ']' && *p != '\0')
                            {
                              p++;
                            }

                          break;
                        }
                    }
                }
            }

          /* We reuse 'i' above to indicate match found */

          if (i)
            {
              continue;
            }

          return FNM_NOMATCH;
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

      if (*p != *string)
        {
          return FNM_NOMATCH;
        }
    }

  if (*string == '\0')
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
