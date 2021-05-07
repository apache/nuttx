/****************************************************************************
 * net/utils/net_timeval2dsec.c
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
