/****************************************************************************
 * net/utils/net_dsec2timeval.c
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
 * Name: net_dsec2timeval
 *
 * Description:
 *   Convert a decisecond timeout value to a struct timeval.  Needed by
 *   getsockopt() to report timeout values.
 *
 * Input Parameters:
 *   dsec The decisecond value to convert
 *   tv   The struct timeval to receive the converted value
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void net_dsec2timeval(uint16_t dsec, FAR struct timeval *tv)
{
  uint16_t remainder;
  tv->tv_sec  = dsec / DSEC_PER_SEC;
  remainder   = dsec - tv->tv_sec * DSEC_PER_SEC;
  tv->tv_usec = remainder * USEC_PER_DSEC;
}
