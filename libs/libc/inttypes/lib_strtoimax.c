/****************************************************************************
 * libs/libc/inttypes/lib_strtoimax.c
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
#include <inttypes.h>
#include <errno.h>

#include "libc.h"

/* Current implementation depends on strtoull() and, hence, is only
 * available if long long types are supported.
 */

#ifdef CONFIG_HAVE_LONG_LONG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strtoimax
 *
 * Description:
 *   The strtoimax() function  converts the initial part of the string in
 *   nptr to a intmax_t integer value according to the given base, which
 *   must be between 2 and 36 inclusive, or be the special value 0.
 *
 * Returned Value:
 *   - The converted value, if the base and number are valid
 *   - 0 if an error occurs, and set errno to:
 *     * EINVAL if base < 2 or base > 36
 *   - INTMAX_MIN or INTMAX_MAX, of correct sign, if an overflow occurs,
 *     and set errno to:
 *     * ERANGE if the number cannot be represented using intmax_t
 *
 ****************************************************************************/

intmax_t strtoimax(FAR const char *nptr, FAR char **endptr, int base)
{
  uintmax_t accum = 0;
  bool negate = false;

  if (nptr)
    {
      /* Skip leading spaces */

      lib_skipspace(&nptr);

      /* Check for leading + or - */

      if (*nptr == '-')
        {
          negate = true;
          nptr++;
        }
      else if (*nptr == '+')
        {
          nptr++;
        }

      /* Get the unsigned value */

      accum = strtoull(nptr, endptr, base);

      /* Correct the sign of the result and check for overflow */

      if (negate)
        {
          const uintmax_t limit = ((uintmax_t)-(INTMAX_MIN + 1)) + 1;

          if (accum > limit)
            {
              set_errno(ERANGE);
              return INTMAX_MIN;
            }

          return (accum == limit) ? INTMAX_MIN : -(intmax_t)accum;
        }

      if (accum > INTMAX_MAX)
        {
          set_errno(ERANGE);
          return INTMAX_MAX;
        }
    }

  return (intmax_t)accum;
}

#endif /* CONFIG_HAVE_LONG_LONG */
