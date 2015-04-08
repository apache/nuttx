/************************************************************
 * libc/fixedmath/lib_rintf.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************/

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdlib.h>
#include <math.h>

/************************************************************
 * Pre-processor Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/**********************************************************
 * Global Constant Data
 **********************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/**********************************************************
 * Private Constant Data
 **********************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

float rintf(float x)
{
  long  linteger;
  float fremainder;
  float ret;

  /* If the current rounding mode rounds toward negative
   * infinity, rintf() is identical to floorf().  If the current
   * rounding mode rounds toward positive infinity, rintf() is
   * identical to ceilf().
   */

#if defined(CONFIG_FP_ROUND_POSITIVE) && CONFIG_FP_ROUNDING_POSITIVE != 0

  ret = ceilf(x);

#elif defined(CONFIG_FP_ROUND_NEGATIVE) && CONFIG_FP_ROUNDING_NEGATIVE != 0

  ret = floorf(x);

#else

  /* In the default rounding mode (round to nearest), rint(x) is the
   * integer nearest x with the additional stipulation that if
   * |rint(x)-x|=1/2, then rint(x) is even.
   */

  linteger  = (long)x;
  fremainder = x - (float)linteger;

  if (x < 0.0)
    {
      /* fremainder should be in range 0 .. -1 */

      if (fremainder == -0.5)
        {
          linteger = ((linteger+1)&~1);
        }
      else if (fremainder < -0.5)
        {
          linteger--;
        }
    }
  else
    {
      /* fremainder should be in range 0 .. 1 */

      if (fremainder == 0.5)
        {
          linteger = ((linteger+1)&~1);
        }
      else if (fremainder > 0.5)
        {
          linteger++;
        }
    }

  ret = (float)linteger;
#endif

  return ret;
}
