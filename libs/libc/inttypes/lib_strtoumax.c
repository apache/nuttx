/****************************************************************************
 * libs/libc/inttypes/lib_strtoumax.c
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

#include <inttypes.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strtoumax
 *
 * Description:
 *   The strtoumax() function  converts  the initial part of the string in
 *   nptr to a long unsigned integer value according to the given base, which
 *   must be between 2 and 36 inclusive, or be the special value 0.
 *
 * Returned Value:
 *   - The converted value, if the base and number are valid
 *   - 0 if an error occurs, and set errno to:
 *     * EINVAL if base < 2 or base > 36
 *   - UINTMAX_MAX if an overflow occurs, and set errno to:
 *     * ERANGE if the number cannot be represented using uintmax_t
 *
 ****************************************************************************/

uintmax_t strtoumax(FAR const char *nptr, FAR char **endptr, int base)
{
  uintmax_t accum = 0;
  uintmax_t prev;
  int value;

  if (nptr)
    {
      /* Skip leading spaces */

      lib_skipspace(&nptr);

      /* Check for unspecified base */

      base = lib_checkbase(base, &nptr);

      if (base < 0)
        {
          set_errno(EINVAL);
          return 0;
        }

      /* Accumulate each "digit" */

      while (lib_isbasedigit(*nptr, base, &value))
        {
          prev  = accum;
          accum = accum*base + value;
          nptr++;

          /* Check for overflow */

          if (accum < prev)
            {
              set_errno(ERANGE);
              accum = UINTMAX_MAX;
              break;
            }
        }

      /* Return the final pointer to the unused value */

      if (endptr)
        {
          *endptr = (char *)nptr;
        }
    }

  return accum;
}
