/************************************************************
 * lib_strtol.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <string.h>
#include <ctype.h>

/************************************************************
 * Private Functions
 ************************************************************/

/* Skip leading spaces */

static void lib_skipspace(const char **nptr)
{
   register const char *tmp = *nptr;
   while (isspace(*tmp)) tmp++;
   *nptr = tmp;
}

static int lib_isbasedigit(int c, int base, int *value)
{
  int tmp = 0;
  int ret = 0;

  if (base <= 10)
    {
      if (c >= '0' && c <= base + '0' - 1)
        {
          tmp = c - '0';
          ret = 1;
        }
    }
  else if (base <= 36)
    {
      if (c >= '0' && c <= '9')
        {
          tmp = c - '0';
          ret = 1;
        }
      else if (c >= 'a' && c <= 'a' + base - 11)
        {
          tmp = c - 'a' + 10;
          ret = 1;
        }
      else if (c >= 'A' && c <= 'A' + base - 11)
        {
          tmp = c - 'A' + 10;
          ret = 1;
        }
    }

  if (value)
    {
      *value = tmp;
    }
  return ret;
}

/************************************************************
 * Public Functions
 ************************************************************/

/* Limited to base 1-36 */

long strtol(const char *nptr, char **endptr, int base)
{
  unsigned long accum = 0;
  int value;
  int negate = 0;

  if (nptr)
    {
      /* Skip leading spaces */

      lib_skipspace(&nptr);

      /* Check for leading + or - */

      if (*nptr == '-')
        {
          negate = 1;
          nptr++;
          lib_skipspace(&nptr);
        }
      else if (*nptr == '+')
        {
          nptr++;
          lib_skipspace(&nptr);
        }

      /* Check for unspecified base */

      if (!base)
        {
          base = 10;
          if (*nptr == '0')
            {
              base = 8;
              nptr++;
              if ((*nptr == 'X' || *nptr == 'x') && 
                  lib_isbasedigit(nptr[1], 16, NULL))
                {
                  base = 16;
                  nptr++;
                }
            }
        }
      else if (base == 16)
        {
          if (nptr[0] == '0' && (nptr[1] == 'X' || nptr[1] == 'x'))
            {
              nptr += 2;
            }
        }

      while (lib_isbasedigit(*nptr, base, &value))
        {
            accum = accum*base + value;
            nptr++;
        }

      if (endptr)
        {
          *endptr = (char *)nptr;
        }

      if (negate)
        {
          return -(long)accum;
        }
    }
  return (long)accum;
}

unsigned long strtoul(const char *nptr, char **endptr, int base)
{
   return (unsigned long)strtol(nptr, endptr, base);
}
