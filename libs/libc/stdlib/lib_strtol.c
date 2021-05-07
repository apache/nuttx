/****************************************************************************
 * libs/libc/stdlib/lib_strtol.c
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

#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strtol
 *
 * Description:
 *   The  strtol() function  converts  the initial part of the string in
 *   nptr to a long integer value according to the given base, which must be
 *   between 2 and 36 inclusive, or be the special value 0.
 *
 * Returned Value:
 *   - The converted value, if the base and number are valid
 *   - 0 if an error occurs, and set errno to:
 *     * EINVAL if base < 2 or base > 36
 *   - LONG_MIN or LONG_MAX, of correct sign, if an overflow occurs,
 *     and set errno to:
 *     * ERANGE if the number cannot be represented using long
 *
 ****************************************************************************/

long strtol(FAR const char *nptr, FAR char **endptr, int base)
{
  unsigned long accum = 0;
  long retval = 0;
  char sign = 0;

  if (nptr)
    {
      /* Skip leading spaces */

      lib_skipspace(&nptr);

      /* Check for leading + or - */

      if (*nptr == '-' || *nptr == '+')
        {
          sign = *nptr;
          nptr++;
        }

      /* Get the unsigned value */

      accum = strtoul(nptr, endptr, base);

      /* Correct the sign of the result and check for overflow */

      if (sign == '-')
        {
          const unsigned long limit = ((unsigned long)-(LONG_MIN + 1)) + 1;

          if (accum > limit)
            {
              set_errno(ERANGE);
              retval = LONG_MIN;
            }
          else
            {
              retval = (accum == limit) ? LONG_MIN : -(long)accum;
            }
        }
      else
        {
          if (accum > LONG_MAX)
            {
              set_errno(ERANGE);
              retval = LONG_MAX;
            }
          else
            {
              retval = accum;
            }
        }
    }

  /* Return the final pointer to the unused value */

  if (endptr)
    {
      if (sign)
        {
          if (*((*endptr) - 1) == sign)
            {
              (*endptr)--;
            }
        }
    }

  return retval;
}
