/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_tim.c
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

#include <debug.h>
#include <sys/types.h>
#include <nuttx/timers/timer.h>

#include "esp32_tim_lowerhalf.h"
#include "esp32_board_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32_TIMER0
#  define ESP32_TIMER0 (0)
#endif

#ifdef CONFIG_ESP32_TIMER1
#  define ESP32_TIMER1 (1)
#endif

#ifdef CONFIG_ESP32_TIMER2
#  define ESP32_TIMER2 (2)
#endif

#ifdef CONFIG_ESP32_TIMER3
#  define ESP32_TIMER3 (3)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_timer_init
 *
 * Description:
 *   Configure the timer driver.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *             form /dev/timerX
 *   timer   - The timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_timer_init(void)
{
  int ret = OK;

#ifdef CONFIG_ESP32_TIMER0
  ret = esp32_timer_initialize("/dev/timer0", ESP32_TIMER0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      goto errout;
    }
#endif

#ifdef CONFIG_ESP32_TIMER1
  ret = esp32_timer_initialize("/dev/timer1", ESP32_TIMER1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      goto errout;
    }
#endif

#ifdef CONFIG_ESP32_TIMER2
  ret = esp32_timer_initialize("/dev/timer2", ESP32_TIMER2);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      goto errout;
    }
#endif

#ifdef CONFIG_ESP32_TIMER3
  ret = esp32_timer_initialize("/dev/timer3", ESP32_TIMER3);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
      goto errout;
    }
#endif

errout:
  return ret;
}

