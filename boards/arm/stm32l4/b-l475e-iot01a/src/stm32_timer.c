/****************************************************************************
 * boards/arm/stm32l4/b-l475e-iot01a/src/stm32_timer.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Goden Freemans <godenfreemans@gmail.com>
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
