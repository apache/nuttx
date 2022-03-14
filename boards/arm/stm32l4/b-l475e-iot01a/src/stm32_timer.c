/****************************************************************************
 * boards/arm/stm32l4/b-l475e-iot01a/src/stm32_timer.c
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
#include <nuttx/timers/timer.h>

#include <debug.h>

#include "stm32l4_tim.h"
#include "b-l475e-iot01a.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_timer_driver_setup
 *
 * Description:
 *   Configure the timer drivers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32l4_timer_driver_setup(void)
{
  int ret = OK;

#ifdef CONFIG_STM32L4_TIM1
  ret = stm32l4_timer_initialize("/dev/timer0", 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM1 at /dev/timer0: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM2
  ret = stm32l4_timer_initialize("/dev/timer1", 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM2 at /dev/timer1: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM3
  ret = stm32l4_timer_initialize("/dev/timer2", 3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM3 at /dev/timer2: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM4
  ret = stm32l4_timer_initialize("/dev/timer3", 4);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM2 at /dev/timer3: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM5
  ret = stm32l4_timer_initialize("/dev/timer4", 5);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM5 at /dev/timer4: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM6
  ret = stm32l4_timer_initialize("/dev/timer5", 6);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM6 at /dev/timer5: %d\n",
             ret);
    }

#endif
#ifdef CONFIG_STM32L4_TIM7
  ret = stm32l4_timer_initialize("/dev/timer6", 7);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM7 at /dev/timer6: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM8
  ret = stm32l4_timer_initialize("/dev/timer7", 8);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM8 at /dev/timer7: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM15
  ret = stm32l4_timer_initialize("/dev/timer8", 15);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM15 at /dev/time8: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM16
  ret = stm32l4_timer_initialize("/dev/timer9", 16);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM16 at /dev/time9: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_STM32L4_TIM17
  ret = stm32l4_timer_initialize("/dev/timer10", 17);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM17 at /dev/time10: %d\n",
             ret);
    }
#endif

  return ret;
}
#endif
