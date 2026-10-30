/****************************************************************************
 * boards/arm/common/stm32/src/board_timer.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdint.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/timers/oneshot.h>

#include "stm32_tim.h"
#include "stm32_oneshot.h"
#ifdef CONFIG_STM32_LPTIM
#  include "stm32_lptim.h"
#endif
#include "board_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVPATH_MAX        32
#define ONESHOT_RESOLUTION 1   /* TIM oneshot resolution in microseconds */

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum stm32_timer_dev_e
{
  DEV_TIMER,                   /* TIMn as /dev/timerN */
  DEV_ONESHOT,                 /* TIMn as /dev/oneshotN */
  DEV_LPTIMER,                 /* LPTIMn as /dev/lptimerN */
  DEV_LPONESHOT                /* LPTIMn as /dev/lponeshotN */
};

struct stm32_timer_dev_s
{
  uint8_t                n;    /* TIM or LPTIM instance */
  enum stm32_timer_dev_e dev;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stm32_timer_dev_s g_timer_devs[] =
{
#ifdef CONFIG_BOARD_STM32_COMMON_TIM1_TIMER
  { 1, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM1_ONESHOT
  { 1, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM2_TIMER
  { 2, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM2_ONESHOT
  { 2, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM3_TIMER
  { 3, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM3_ONESHOT
  { 3, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM4_TIMER
  { 4, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM4_ONESHOT
  { 4, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM5_TIMER
  { 5, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM5_ONESHOT
  { 5, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM6_TIMER
  { 6, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM6_ONESHOT
  { 6, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM7_TIMER
  { 7, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM7_ONESHOT
  { 7, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM8_TIMER
  { 8, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM8_ONESHOT
  { 8, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM9_TIMER
  { 9, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM9_ONESHOT
  { 9, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM10_TIMER
  { 10, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM10_ONESHOT
  { 10, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM11_TIMER
  { 11, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM11_ONESHOT
  { 11, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM12_TIMER
  { 12, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM12_ONESHOT
  { 12, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM13_TIMER
  { 13, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM13_ONESHOT
  { 13, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM14_TIMER
  { 14, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM14_ONESHOT
  { 14, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM15_TIMER
  { 15, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM15_ONESHOT
  { 15, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM16_TIMER
  { 16, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM16_ONESHOT
  { 16, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM17_TIMER
  { 17, DEV_TIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_TIM17_ONESHOT
  { 17, DEV_ONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM1_TIMER
  { 1, DEV_LPTIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM1_ONESHOT
  { 1, DEV_LPONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM2_TIMER
  { 2, DEV_LPTIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM2_ONESHOT
  { 2, DEV_LPONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM3_TIMER
  { 3, DEV_LPTIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM3_ONESHOT
  { 3, DEV_LPONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM4_TIMER
  { 4, DEV_LPTIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM4_ONESHOT
  { 4, DEV_LPONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM5_TIMER
  { 5, DEV_LPTIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM5_ONESHOT
  { 5, DEV_LPONESHOT },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM6_TIMER
  { 6, DEV_LPTIMER },
#endif
#ifdef CONFIG_BOARD_STM32_COMMON_LPTIM6_ONESHOT
  { 6, DEV_LPONESHOT },
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_timer_setup
 *
 * Description:
 *   Register the timer and oneshot devices selected by the
 *   BOARD_STM32_COMMON_TIMn_* and BOARD_STM32_COMMON_LPTIMn_* options.
 *   A failing device is logged and does not stop the others.
 *
 ****************************************************************************/

int stm32_timer_setup(void)
{
  static const char * const names[] =
  {
    "timer", "oneshot", "lptimer", "lponeshot"
  };

#ifdef CONFIG_ONESHOT
  struct oneshot_lowerhalf_s *lower;
#endif
  char devpath[DEVPATH_MAX];
  int ret = OK;
  int tmp;
  int i;

  for (i = 0; i < sizeof(g_timer_devs) / sizeof(g_timer_devs[0]); i++)
    {
      const struct stm32_timer_dev_s *d = &g_timer_devs[i];

      snprintf(devpath, sizeof(devpath), "/dev/%s%d", names[d->dev], d->n);
      tmp = -ENODEV;
#ifdef CONFIG_ONESHOT
      lower = NULL;
#endif

      switch (d->dev)
        {
#ifdef CONFIG_TIMER
#  ifdef CONFIG_STM32_TIM
          case DEV_TIMER:
            tmp = stm32_timer_initialize(devpath, d->n);
            break;
#  endif

#  ifdef CONFIG_STM32_LPTIM
          case DEV_LPTIMER:
            tmp = stm32_lptim_timer_initialize(devpath, d->n);
            break;
#  endif
#endif

#ifdef CONFIG_ONESHOT
#  ifdef CONFIG_STM32_ONESHOT
          case DEV_ONESHOT:
            lower = oneshot_initialize(d->n, ONESHOT_RESOLUTION);
            break;
#  endif

#  ifdef CONFIG_STM32_LPTIM
          case DEV_LPONESHOT:
            lower = stm32_lptim_oneshot_initialize(d->n);
            break;
#  endif
#endif

          default:
            break;
        }

#ifdef CONFIG_ONESHOT
      if (lower != NULL)
        {
          tmp = oneshot_register(devpath, lower);
        }
#endif

      if (tmp < 0)
        {
          syslog(LOG_ERR, "ERROR: %s failed: %d\n", devpath, tmp);
          ret = tmp;
        }
    }

  return ret;
}
