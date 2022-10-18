/****************************************************************************
 * drivers/modem/alt1250/altmdm_timer.h
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

#ifndef __DEVICES_MODEM_ALT1250_ALTMDM_TIMER_H
#define __DEVICES_MODEM_ALT1250_ALTMDM_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>
#include <time.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_timer_start
 *
 * Description:
 *   Starts the timer with the specified timeout period. When the timeout
 *   occurs, the function specified in the handler is executed.
 *
 * Input Parameters:
 *   first_ms        - The time until next expiration.
 *   interval_ms     - The reload value of the timer.
 *   handler         - Function pointer to be called when the timer expires.
 *   ptr_param       - The value set in info->si_value.sival_ptr.
 *                     info is the second argument of the handler.
 *
 * Returned Value:
 *   If the altmdm_timer_start() succeeds, timer instance will be returned.
 *   If an error occurs, the NULL will be returned.
 *
 ****************************************************************************/

timer_t altmdm_timer_start(int first_ms, int interval_ms,
  FAR _sa_sigaction_t handler, FAR void *ptr_param);

/****************************************************************************
 * Name: altmdm_timer_restart
 *
 * Description:
 *   Restarts the timer with the specified timeout period.
 *
 * Input Parameters:
 *   timerid         - timer instatce created by altmdm_timer_start().
 *   first_ms        - The time until next expiration.
 *   interval_ms     - The reload value of the timer.
 *
 * Returned Value:
 *   If the altmdm_timer_restart() succeeds, the value 0 (OK) will be
 *   returned. If an error occurs, the value -1 (ERROR) will be returned.
 *
 ****************************************************************************/

int altmdm_timer_restart(timer_t timerid, int first_ms, int interval_ms);

/****************************************************************************
 * Name: altmdm_timer_is_running
 *
 * Description:
 *   Check whether the timer is running or not.
 *
 * Input Parameters:
 *   timerid         - timer instatce created by altmdm_timer_start().
 *
 * Returned Value:
 *   Return 1 if the timer is running, 0 otherwise.
 *
 ****************************************************************************/

int altmdm_timer_is_running(timer_t timerid);

/****************************************************************************
 * Name: altmdm_timer_stop
 *
 * Description:
 *   Stops the timer.
 *
 * Input Parameters:
 *   timerid         - timer instatce created by altmdm_timer_start().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void altmdm_timer_stop(timer_t timerid);

#endif  /* __DEVICES_MODEM_ALT1250_ALTMDM_TIMER_H */
