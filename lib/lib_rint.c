/************************************************************
 * lib_rint.c
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
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <stdlib.h>

/************************************************************
 * Definitions
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

double rint(double x)
{
  double retValue;

/* If the current rounding mode rounds toward negative
 * infinity, rint() is identical to floor().  If the current
 * rounding mode rounds toward positive infinity, rint() is
 * identical to ceil(). */
#if ((defined(FP_ROUND_POSITIVE)) && (FP_ROUNDING_POSITIVE != 0))
  retValue = ceil(x);

#elif ((defined(FP_ROUND_NEGATIVE)) && (FP_ROUNDING_NEGATIVE != 0))
  retValue = floor(x);

#else

  /* In the default rounding mode (round to nearest), rint(x) is the
   * integer nearest x with the additional stipulation that if
   * |rint(x)-x|=1/2, then rint(x) is even. */

  long dwInteger = (long)x;
  double fRemainder = x - (double)dwInteger;

  if (x < 0.0) {

    /* fRemainder should be in range 0 .. -1 */
    if (fRemainder == -0.5)
      dwInteger = ((dwInteger+1)&~1);
    else if (fRemainder < -0.5) {
      dwInteger--;

    } /* end if */
  } /* end if */
  else {

    /* fRemainder should be in range 0 .. 1 */
    if (fRemainder == 0.5)
      dwInteger = ((dwInteger+1)&~1);
    else  if (fRemainder > 0.5) {
      dwInteger++;

    } /* end if */
  } /* end else */

  retValue = (double)dwInteger;
#endif

  return retValue;
}
