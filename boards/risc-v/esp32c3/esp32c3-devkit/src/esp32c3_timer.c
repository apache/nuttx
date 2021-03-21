/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/src/esp32c3_timer.c
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
#include <debug.h>

#include "esp32c3_tim_lowerhalf.h"
#include "esp32c3_tim.h"

#include "esp32c3-devkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_tim_init
 *
 * Description:
 *   Configure the timer driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_tim_init(void)
{
  int ret = OK;

#if defined(CONFIG_ESP32C3_TIMER0) && !defined(CONFIG_ESP32C3_RT_TIMER)
  ret = esp32c3_timer_initialize("/dev/timer0", ESP32C3_TIMER0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      return ret;
    }
#endif /* CONFIG_ESP32C3_TIMER0 */

#ifdef CONFIG_ESP32C3_TIMER1
  ret = esp32c3_timer_initialize("/dev/timer1", ESP32C3_TIMER1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      return ret;
    }
#endif /* CONFIG_ESP32C3_TIMER1 */

  return ret;
}

