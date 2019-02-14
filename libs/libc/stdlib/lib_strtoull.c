/****************************************************************************
 * /libs/libc/stdlib/lib_strtoull.c
 *
 *   Copyright (C) 2009, 2010, 2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdlib.h>
#include <errno.h>

#include "libc.h"

#ifdef CONFIG_HAVE_LONG_LONG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strtoull
 *
 * Description:
 *   The strtoull() function  converts  the initial part of the string in
 *   nptr to a long unsigned integer value according to the given base, which
 *   must be between 2 and 36 inclusive, or be the special value 0.
 *
 * Returned Value:
 *   - The converted value, if the base and number are valid
 *   - 0 if an error occurs, and set errno to:
 *     * EINVAL if base < 2 or base > 36
 *   - ULLONG_MAX if an overflow occurs, and set errno to:
 *     * ERANGE if the number cannot be represented using unsigned long long
 *
 ****************************************************************************/

unsigned long long strtoull(FAR const char *nptr, FAR char **endptr, int base)
{
  unsigned long long accum = 0;
  unsigned long long prev;
  int value;
  char sign = 0;

  if (nptr)
    {
      /* Skip leading spaces */

      lib_skipspace(&nptr);

      /* Check for leading + or - already done for strtol */

      if (*nptr == '-' || *nptr == '+')
        {
          sign = *nptr;
          nptr++;
        }

      /* Check for unspecified or incorrect base */

      base = lib_checkbase(base, &nptr);

      if (base < 0)
        {
          set_errno(EINVAL);
          accum = 0;
        }
      else
        {
          /* Accumulate each "digit" */

          while (lib_isbasedigit(*nptr, base, &value))
            {
              prev = accum;
              accum = accum * base + value;
              nptr++;

              /* Check for overflow */

              if (accum < prev)
                {
                  set_errno(ERANGE);
                  accum = ULLONG_MAX;
                  break;
                }
            }

          if (sign == '-')
            {
              accum = (~accum) + 1;
            }
        }
    }

  /* Return the final pointer to the unused value */

  if (endptr)
    {
      if (sign)
        {
          if (*(nptr - 1) == sign)
            {
              nptr--;
            }
        }

      *endptr = (FAR char *)nptr;
    }

  return accum;
}

#endif
