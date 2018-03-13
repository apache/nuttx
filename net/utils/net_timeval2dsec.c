/****************************************************************************
 * net/utils/net_timeval2dsec.c
 *
 *   Copyright (C) 2007, 2008, 2014-2015 Gregory Nutt. All rights reserved.
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

#include <sys/time.h>

#include <nuttx/clock.h>

#include "utils/utils.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_timeval2dsec
 *
 * Description:
 *   Convert a struct timeval to deciseconds.  Needed by setsockopt() to
 *   save new timeout values.
 *
 * Input Parameters:
 *   tv        - The struct timeval to convert
 *   remainder - Determines how to handler the microsecond remainder
 *
 * Returned Value:
 *   The converted value
 *
 * Assumptions:
 *
 ****************************************************************************/

unsigned int net_timeval2dsec(FAR struct timeval *tv,
                              enum tv2ds_remainder_e remainder)
{
  unsigned long adjust = 0;

  switch (remainder)
    {
    default:
    case TV2DS_TRUNC: /* Truncate microsecond remainder */
      break;

    case TV2DS_ROUND: /* Round to the nearest full decisecond */
      adjust = (USEC_PER_DSEC / 2);
      break;

    case TV2DS_CEIL:  /* Force to next larger full decisecond */
      adjust = (USEC_PER_DSEC - 1);
      break;
    }

  return (unsigned int)(tv->tv_sec * DSEC_PER_SEC +
                       (tv->tv_usec + adjust) / USEC_PER_DSEC);
}
