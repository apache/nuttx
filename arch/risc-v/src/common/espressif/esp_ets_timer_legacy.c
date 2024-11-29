/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_ets_timer_legacy.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this args for additional information regarding copyright ownership.  The
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

/* ets_timer module implements a set of legacy timer APIs which are
 * used by the WiFi driver. This is done on top of the esp_hr_timer APIs.
 * Applications should not use ets_timer functions, as they may change
 * without notice.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"

#include "espressif/esp_hr_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We abuse 'timer_arg' field of ETSTimer structure to hold a
 * pointer to esp_hr_timer.
 */

#define ESP_TIMER(p_ets_timer) \
    ((struct esp_hr_timer_s *) (p_ets_timer)->timer_arg)

/* Override internal name to be compliant with NuttX C Coding Standard */

#define ets_timer       ETSTimer
#define ets_timer_func  ETSTimerFunc

/* We abuse 'timer_expire' field of ETSTimer structure to hold a magic value
 * signifying that the contents of the timer was zeroed out.
 */

#define TIMER_INITIALIZED_FIELD(p_ets_timer) ((p_ets_timer)->timer_expire)
#define TIMER_INITIALIZED_VAL 0x12121212

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool timer_initialized(ets_timer *ptimer);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_initialized
 *
 * Description:
 *   This function checks if the given ETS timer is initialized.
 *
 * Input Parameters:
 *   ptimer - Pointer to the ETS timer structure.
 *
 * Returned Value:
 *   Returns true if the timer is initialized, false otherwise.
 *
 ****************************************************************************/

static bool IRAM_ATTR timer_initialized(ets_timer *ptimer)
{
  return TIMER_INITIALIZED_FIELD(ptimer) == TIMER_INITIALIZED_VAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ets_timer_setfn
 *
 * Description:
 *   Set timer callback function and argument.
 *
 * Input Parameters:
 *   ptimer    - Timer struct pointer
 *   pfunction - Timer callback
 *   parg      - Timer callback argument
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ets_timer_setfn(ets_timer *ptimer,
                     ets_timer_func *pfunction,
                     void *parg)
{
  int ret = OK;

  if (!timer_initialized(ptimer))
    {
      memset(ptimer, 0, sizeof(*ptimer));
      TIMER_INITIALIZED_FIELD(ptimer) = TIMER_INITIALIZED_VAL;
    }

  if (ESP_TIMER(ptimer) == NULL)
    {
      struct esp_hr_timer_args_s hr_timer_args;
      struct esp_hr_timer_s *hr_timers_p;

      hr_timer_args.arg      = parg;
      hr_timer_args.callback = pfunction;

      ret = esp_hr_timer_create(&hr_timer_args, &hr_timers_p);

      if (ret)
        {
          tmrerr("Failed to create ets_timer error=%d\n", ret);
        }
      else
        {
          ptimer->timer_arg = hr_timers_p;
        }
    }
}

/****************************************************************************
 * Name: ets_timer_arm_us
 *
 * Description:
 *   Arms an ETS timer. The timer range is from 640 us to 429496 ms.
 *
 * Input Parameters:
 *   ptimer      - Pointer to the timer structure.
 *   time_us     - Timer value in microseconds. The range is 1 to 429496729.
 *   repeat_flag - Specifies if the timer is periodically repeated.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR ets_timer_arm_us(ets_timer *ptimer,
                                uint32_t time_us,
                                bool repeat_flag)
{
  assert(timer_initialized(ptimer));

  esp_hr_timer_stop(ESP_TIMER(ptimer));

  if (!repeat_flag)
    {
      esp_hr_timer_start(ESP_TIMER(ptimer), time_us, false);
    }
  else
    {
      esp_hr_timer_start(ESP_TIMER(ptimer), time_us, true);
    }
}

/****************************************************************************
 * Name: ets_timer_arm
 *
 * Description:
 *   Arms an ETS timer. The timer range is from 640 us to 429496 ms.
 *
 * Input Parameters:
 *   ptimer      - Pointer to the timer structure.
 *   time_us     - Timer value in microseconds. The range is 1 to 429496.
 *   repeat_flag - Specifies if the timer is periodically repeated.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR ets_timer_arm(ets_timer *ptimer,
                             uint32_t time_ms,
                             bool repeat_flag)
{
  uint64_t time_us = 1000LL * (uint64_t) time_ms;

  assert(timer_initialized(ptimer));

  esp_hr_timer_stop(ESP_TIMER(ptimer));

  if (!repeat_flag)
    {
      esp_hr_timer_start(ESP_TIMER(ptimer), time_us, false);
    }
  else
    {
      esp_hr_timer_start(ESP_TIMER(ptimer), time_us, true);
    }
}

/****************************************************************************
 * Name: ets_timer_done
 *
 * Description:
 *   Unset timer callback and argument to NULL.
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ets_timer_done(ets_timer *ptimer)
{
  if (timer_initialized(ptimer))
    {
      esp_hr_timer_delete(ESP_TIMER(ptimer));
      ptimer->timer_arg = NULL;
      TIMER_INITIALIZED_FIELD(ptimer) = 0;
    }
}

/****************************************************************************
 * Name: ets_timer_disarm
 *
 * Description:
 *   Disarm an ets timer.
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR ets_timer_disarm(ets_timer *ptimer)
{
  if (timer_initialized(ptimer))
    {
      esp_hr_timer_stop(ESP_TIMER(ptimer));
    }
}
