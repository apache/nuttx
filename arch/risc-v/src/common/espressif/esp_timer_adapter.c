/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_timer_adapter.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>

#include "esp_timer.h"
#include "esp_timer_adapter.h"

#include "esp_private/esp_timer_private.h"
#include "esp_timer_impl.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* HR timer data structure wrapping HAL esp_timer (internal definition) */

struct esp_hr_timer_s
{
  esp_timer_handle_t hal_timer;     /* HAL timer handle */
  uint64_t timeout;                 /* Timeout value */
  uint64_t alarm;                   /* Timeout period */
  void (*callback)(void *arg);      /* Callback function */
  void *arg;                        /* Private data */
  uint16_t flags;                   /* Supported features */
  enum esp_hr_timer_state_e state;  /* Timer state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_hr_timer_callback_wrapper(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_hr_timer_lock = SP_UNLOCKED;
static bool g_hr_timer_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_hr_timer_callback_wrapper
 *
 * Description:
 *   Adapter callback used by esp_timer to dispatch to the upper-layer timer
 *   callback and update the timer state.
 *
 * Input Parameters:
 *   arg - Pointer to struct esp_hr_timer_s.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_hr_timer_callback_wrapper(void *arg)
{
  struct esp_hr_timer_s *timer = (struct esp_hr_timer_s *)arg;

  if (timer && timer->callback)
    {
      timer->state = HR_TIMER_TIMEOUT;
      timer->callback(timer->arg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_hr_timer_create
 *
 * Description:
 *   Create a high-resolution timer instance and initialize it from the
 *   provided arguments.
 *
 * Input Parameters:
 *   args         - Timer creation arguments.
 *   timer_handle - Location where the created timer handle is returned.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_hr_timer_create(const struct esp_hr_timer_args_s *args,
                        struct esp_hr_timer_s **timer_handle)
{
  struct esp_hr_timer_s *timer;
  esp_timer_create_args_t hal_args;
  esp_err_t ret;

  if (args == NULL || timer_handle == NULL || args->callback == NULL)
    {
      return -EINVAL;
    }

  timer = kmm_zalloc(sizeof(struct esp_hr_timer_s));
  if (timer == NULL)
    {
      return -ENOMEM;
    }

  timer->callback = args->callback;
  timer->arg = args->arg;
  timer->flags = HR_TIMER_NOFLAGS;
  timer->state = HR_TIMER_IDLE;

  hal_args.callback = esp_hr_timer_callback_wrapper;
  hal_args.arg = timer;
  hal_args.dispatch_method = ESP_TIMER_TASK;
  hal_args.name = args->name ? args->name : "nuttx_hr";
  hal_args.skip_unhandled_events = args->skip_unhandled_events;

  ret = esp_timer_create(&hal_args, &timer->hal_timer);
  if (ret != ESP_OK)
    {
      kmm_free(timer);
      return -EINVAL;
    }

  *timer_handle = timer;
  return OK;
}

/****************************************************************************
 * Name: esp_hr_timer_start
 *
 * Description:
 *   Start a high-resolution timer in one-shot or periodic mode.
 *
 * Input Parameters:
 *   timer   - Timer instance to start.
 *   timeout - Timeout period in microseconds.
 *   repeat  - True for periodic mode, false for one-shot mode.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_start(struct esp_hr_timer_s *timer,
                       uint64_t timeout,
                       bool repeat)
{
  esp_err_t ret;

  if (timer == NULL)
    {
      return ERROR;
    }

  timer->timeout = timeout;

  if (repeat)
    {
      timer->flags |= HR_TIMER_REPEAT;
      ret = esp_timer_start_periodic(timer->hal_timer, timeout);
    }
  else
    {
      timer->flags &= ~HR_TIMER_REPEAT;
      ret = esp_timer_start_once(timer->hal_timer, timeout);
    }

  if (ret == ESP_OK)
    {
      timer->state = HR_TIMER_READY;
      return OK;
    }

  return ERROR;
}

/****************************************************************************
 * Name: esp_hr_timer_start_once
 *
 * Description:
 *   Start a high-resolution timer in one-shot mode.
 *
 * Input Parameters:
 *   timer   - Timer instance to start.
 *   timeout - One-shot timeout in microseconds.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_start_once(struct esp_hr_timer_s *timer, uint64_t timeout)
{
  return esp_hr_timer_start(timer, timeout, false);
}

/****************************************************************************
 * Name: esp_hr_timer_start_periodic
 *
 * Description:
 *   Start a high-resolution timer in periodic mode.
 *
 * Input Parameters:
 *   timer   - Timer instance to start.
 *   timeout - Period interval in microseconds.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_start_periodic(struct esp_hr_timer_s *timer,
                                uint64_t timeout)
{
  return esp_hr_timer_start(timer, timeout, true);
}

/****************************************************************************
 * Name: esp_hr_timer_stop
 *
 * Description:
 *   Stop a running high-resolution timer.
 *
 * Input Parameters:
 *   timer - Timer instance to stop.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_stop(struct esp_hr_timer_s *timer)
{
  esp_err_t ret;

  if (timer == NULL)
    {
      return ERROR;
    }

  ret = esp_timer_stop(timer->hal_timer);
  if (ret == ESP_OK)
    {
      timer->state = HR_TIMER_IDLE;
      return OK;
    }

  return ERROR;
}

/****************************************************************************
 * Name: esp_hr_timer_stop_nolock
 *
 * Description:
 *   Stop a running high-resolution timer without taking any adapter lock.
 *
 * Input Parameters:
 *   timer - Timer instance to stop.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_stop_nolock(struct esp_hr_timer_s *timer)
{
  /* esp_timer already handles its own locking */

  return esp_hr_timer_stop(timer);
}

/****************************************************************************
 * Name: esp_hr_timer_delete
 *
 * Description:
 *   Delete a timer instance and release associated resources.
 *
 * Input Parameters:
 *   timer - Timer instance to delete.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_delete(struct esp_hr_timer_s *timer)
{
  esp_err_t ret;

  if (timer == NULL)
    {
      return ERROR;
    }

  timer->state = HR_TIMER_DELETE;

  ret = esp_timer_delete(timer->hal_timer);
  if (ret == ESP_OK)
    {
      kmm_free(timer);
      return OK;
    }

  return ERROR;
}

/****************************************************************************
 * Name: esp_hr_timer_time_us
 *
 * Description:
 *   Return the current high-resolution timer time in microseconds.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Current timer time in microseconds.
 *
 ****************************************************************************/

uint64_t esp_hr_timer_time_us(void)
{
  return esp_timer_impl_get_time();
}

/****************************************************************************
 * Name: esp_hr_timer_get_alarm
 *
 * Description:
 *   Return the timestamp of the next scheduled timer alarm.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Absolute time in microseconds for the next alarm.
 *
 ****************************************************************************/

uint64_t esp_hr_timer_get_alarm(void)
{
  return esp_timer_get_next_alarm();
}

/****************************************************************************
 * Name: esp_hr_timer_calibration
 *
 * Description:
 *   Apply a timer calibration adjustment.
 *   The HAL esp_timer backend does not support calibration, so this is a
 *   no-op.
 *
 * Input Parameters:
 *   time_us - Calibration adjustment in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_calibration(uint64_t time_us)
{
  /* HAL esp_timer does not support calibration */

  UNUSED(time_us);
}

/****************************************************************************
 * Name: esp_hr_timer_set
 *
 * Description:
 *   Set the high-resolution timer counter to a specific timestamp.
 *
 * Input Parameters:
 *   new_us - New timer value in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_set(uint64_t new_us)
{
  /* Use HAL function to set timer */

  esp_timer_private_set(new_us);
}

/****************************************************************************
 * Name: esp_hr_timer_lock
 *
 * Description:
 *   Acquire the adapter lock used to serialize timer operations.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_lock(void)
{
  spin_lock(&g_hr_timer_lock);
}

/****************************************************************************
 * Name: esp_hr_timer_unlock
 *
 * Description:
 *   Release the adapter lock used to serialize timer operations.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_unlock(void)
{
  spin_unlock(&g_hr_timer_lock);
}

/****************************************************************************
 * Name: esp_hr_timer_init
 *
 * Description:
 *   Initialize the timer adapter and the underlying ESP timer subsystem.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_init(void)
{
  esp_err_t ret;

  if (g_hr_timer_initialized)
    {
      return OK;
    }

  /* Initialize the timer subsystem */

  ret = esp_timer_init();
  if (ret == ESP_OK)
    {
      g_hr_timer_initialized = true;
      return OK;
    }

  return ERROR;
}
