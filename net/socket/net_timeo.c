/****************************************************************************
 * net/socket/net_timeo.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_SOCKOPTS)

#include <stdint.h>
#include <debug.h>

#include <nuttx/clock.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_timeo
 *
 * Description:
 *   Check if a timeout has elapsed.  This can be called from a socket poll
 *   function to determine if a timeout has occurred.
 *
 * Input Parameters:
 *   start_time Timeout start time in system clock ticks
 *   timeout    Timeout value in deciseconds.
 *
 * Returned Value:
 *   0 (FALSE) if not timeout; 1 (TRUE) if timeout
 *
 * Assumptions:
 *
 ****************************************************************************/

int net_timeo(clock_t start_time, socktimeo_t timeo)
{
  clock_t timeo_ticks =  DSEC2TICK(timeo);
  clock_t elapsed     =  clock_systime_ticks() - start_time;

  if (elapsed >= timeo_ticks)
    {
      return TRUE;
    }

  return FALSE;
}

#endif /* CONFIG_NET && CONFIG_NET_SOCKOPTS */
