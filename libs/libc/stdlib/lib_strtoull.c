/****************************************************************************
 * libs/libc/stdlib/lib_strtoull.c
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

unsigned long long strtoull(FAR const char *nptr,
                            FAR char **endptr, int base)
{
  unsigned long long accum = 0;
  unsigned long long limit;
  int value;
  int last_digit;
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
          limit = ULLONG_MAX / base;
          last_digit = ULLONG_MAX % base;

          /* Accumulate each "digit" */

          while (lib_isbasedigit(*nptr, base, &value))
            {
              /* Check for overflow */

              if (accum > limit || (accum == limit && value > last_digit))
                {
                  set_errno(ERANGE);
                  accum = ULLONG_MAX;
                  break;
                }

              accum = accum * base + value;
              nptr++;
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
