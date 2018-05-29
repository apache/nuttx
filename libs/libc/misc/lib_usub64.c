/****************************************************************************
 * libs/libc/misc/lib_usub64.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/lib/math32.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usub64
 *
 * Description:
 *   Subtract two 64-bit values and return the 64-bit difference.
 *
 * Input Parameters:
 *   minuend    - The number from which another number (the Subtrahend) is
 *     to be subtracted.
 *   subtrahend - The number that is to be subtracted.
 *   difference - The location to return the difference of the two values.
 *     difference may the same as one of minuend or subtrahend.
 *
 ****************************************************************************/

void usub64(FAR const struct uint64_s *minuend,
            FAR const struct uint64_s *subtrahend,
            FAR struct uint64_s *difference)
{
  /* Get the MS part of the difference */

  difference->ms = minuend->ms - subtrahend->ms;

  /* Check for a borrow, i.e., that is when:
   *
   * subtrahend->ls > minuend->ls
   */

  if (subtrahend->ls > minuend->ls)
    {
      difference->ms--;
    }

  /* Get the LS part of the difference */

  difference->ls = minuend->ls - subtrahend->ls;
}
