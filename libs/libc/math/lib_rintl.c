/****************************************************************************
 * libs/libc/math/lib_rintl.c
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

#include <sys/types.h>
#include <stdlib.h>
#include <math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Constant Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

long double rintl(long double x)
{
  int64_t     llinteger;
  long double fremainder;
  long double ret;

  /* If the current rounding mode rounds toward negative
   * infinity, rint() is identical to floor().  If the current
   * rounding mode rounds toward positive infinity, rint() is
   * identical to ceil().
   */

#if defined(CONFIG_FP_ROUND_POSITIVE) && CONFIG_FP_ROUNDING_POSITIVE != 0

  ret = ceil(x);

#elif defined(CONFIG_FP_ROUND_NEGATIVE) && CONFIG_FP_ROUNDING_NEGATIVE != 0

  ret = floor(x);

#else

  /* In the default rounding mode (round to nearest), rint(x) is the
   * integer nearest x with the additional stipulation that if
   * |rint(x)-x|=1/2, then rint(x) is even.
   */

  llinteger  = (int64_t)x;
  fremainder = x - (long double)llinteger;

  if (x < 0.0)
    {
      /* fremainder should be in range 0 .. -1 */

      if (fremainder == -0.5)
        {
          llinteger = ((llinteger + 1) & ~1);
        }
      else if (fremainder < -0.5)
        {
          llinteger--;
        }
    }
  else
    {
      /* fremainder should be in range 0 .. 1 */

      if (fremainder == 0.5)
        {
          llinteger = ((llinteger + 1) & ~1);
        }
      else if (fremainder > 0.5)
        {
          llinteger++;
        }
    }

  ret = (long double)llinteger;
#endif

  return ret;
}
