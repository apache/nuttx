/****************************************************************************
 * arch/xtensa/src/esp32/esp32_oneshot.c
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
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/clock.h>

#include "hardware/esp32_soc.h"

#include "esp32_tim.h"
#include "esp32_clockconfig.h"
#include "esp32_oneshot.h"

#ifdef CONFIG_ESP32_ONESHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_TIMER_COUNTER UINT64_MAX

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32_oneshot_handler(int irq, void * context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_oneshot_handler
 *
 * Description:
 *   Oneshot interrupt Handler. When any oneshot timer interrupt
 *   expires, this function will be triggered. It will forward the call to
 *   the next level up.
 *
 * Input Parameters:
 *   irq   - IRQ associated to that interrupt
 *   arg -   A pointer to the argument provided when the interrupt was
 *           registered.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp32_oneshot_handler(int irq, void * context, void *arg)
{
  int ret = OK;
  struct esp32_oneshot_s *oneshot = (struct esp32_oneshot_s *)arg;

  DEBUGASSERT(oneshot != NULL && oneshot->handler != NULL);

  tmrinfo("Oneshot handler triggered\n");

  /* Stop timer
   * Note: It's not necessary to disable the alarm because
   * it automatically disables each time it expires.
   */

  ESP32_TIM_STOP(oneshot->tim);

  /* Disable int */

  ESP32_TIM_DISABLEINT(oneshot->tim);

  /* Detach handler */

  ret = ESP32_TIM_SETISR(oneshot->tim, NULL, NULL);

  /* Call the callback */

  oneshot->handler((void *)oneshot->arg);

  /* Restore state */

  oneshot->running = false;
  oneshot->handler = NULL;
  oneshot->arg = NULL;

  /* Clear the Interrupt */

  ESP32_TIM_ACKINT(oneshot->tim);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper.
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure.
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32_oneshot_initialize(struct esp32_oneshot_s *oneshot, int chan,
                             uint16_t resolution)
{
  uint16_t pre;
  int ret = OK;
  tmrinfo("chan=%d resolution=%d usecs\n", chan, resolution);
  DEBUGASSERT(oneshot != NULL);
  DEBUGASSERT(resolution > 0);

  oneshot->chan = chan;

  oneshot->tim = esp32_tim_init(chan);
  if (oneshot->tim == NULL)
    {
      tmrerr("ERROR: Failed to allocate TIM %d\n", chan);
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the remaining fields in the state structure. */

      oneshot->running    = false;
      oneshot->handler    = NULL;
      oneshot->arg        = NULL;
      oneshot->resolution = resolution;

      /* Ensure timer is disabled.
       * Change the prescaler divider with the timer enabled can lead to
       * unpredictable results.
       */

      ESP32_TIM_STOP(oneshot->tim);

      /* Calculate the suitable prescaler according to the current apb
       * frequency to generate a period equals to resolution.
       */

      pre = (esp_clk_apb_freq() * resolution) / USEC_PER_SEC;

      /* Configure TIMER prescaler */

      ESP32_TIM_SETPRE(oneshot->tim, pre);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_oneshot_max_delay
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure. This
 *           structure must have been previously initialized via a call to
 *           esp32_oneshot_initialize();
 *   usec    The maximum delay in us.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

int esp32_oneshot_max_delay(struct esp32_oneshot_s *oneshot, uint64_t *usec)
{
  DEBUGASSERT(oneshot != NULL && usec != NULL);

  /* In theory, Maximum delay (us) = resolution (us) * MAX_TIMER_COUNTER
   * But if the resolution is bigger than 1 us, the value will not fit
   * in a uint64_t. So, this function assumes the max delay using a
   *  resolution of 1 us.
   */

  *usec = MAX_TIMER_COUNTER;

  return OK;
}

/****************************************************************************
 * Name: esp32_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           esp32_oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32_oneshot_start(struct esp32_oneshot_s *oneshot,
                        oneshot_handler_t handler, void *arg,
                        const struct timespec *ts)
{
  uint64_t timeout_us;
  int ret = OK;

  tmrinfo("handler=%p arg=%p, ts=(%lu, %lu)\n",
          handler, arg, (unsigned long)ts->tv_sec,
          (unsigned long)ts->tv_nsec);
  DEBUGASSERT(oneshot != NULL);
  DEBUGASSERT(handler != NULL);
  DEBUGASSERT(ts != NULL);

  if (oneshot->running)
    {
      tmrinfo("One shot timer already in use. Cancelling it ...\n");

      /* If the oneshot timer was already started, cancel it and then
       * restart.
       */

      esp32_oneshot_cancel(oneshot, NULL);
    }

  /* Save the new callback and its argument */

  oneshot->handler = handler;
  oneshot->arg     = arg;

  /* Retrieve the duration from timespec in microsecond */

  timeout_us = (uint64_t)ts->tv_sec * USEC_PER_SEC +
               (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  /* Verify if it is a multiple of the configured resolution.
   * In case it isn't, warn the user.
   */

  if ((timeout_us % oneshot->resolution) != 0)
    {
      tmrwarn("Warning: The interval is not multiple of the resolution.\n"
               "Adjust the resolution in your bringup file.\n");
    }

  /* Set the timer */

  /* Ensure timer is stopped */

  ESP32_TIM_STOP(oneshot->tim);

  /* Configure TIMER mode */

  ESP32_TIM_SETMODE(oneshot->tim, ESP32_TIM_MODE_UP);

  /* Clear TIMER counter value */

  ESP32_TIM_CLEAR(oneshot->tim);

  /* Disable autoreload */

  ESP32_TIM_SETARLD(oneshot->tim, false);

  /* Set the timeout */

  ESP32_TIM_SETALRVL(oneshot->tim, timeout_us / oneshot->resolution);

  /* Enable TIMER alarm */

  ESP32_TIM_SETALRM(oneshot->tim, true);

  /* Clear Interrupt Bits Status */

  ESP32_TIM_ACKINT(oneshot->tim);

  /* Set the interrupt */

  /* Register the handler that calls the callback */

  ret = ESP32_TIM_SETISR(oneshot->tim, esp32_oneshot_handler, oneshot);
  if (ret == OK)
    {
      ESP32_TIM_ENABLEINT(oneshot->tim);

      /* Finally, start the TIMER */

      ESP32_TIM_START(oneshot->tim);

      oneshot->running = true;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           esp32_oneshot_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.  ts may be zero in which case the time remaining
 *           is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int esp32_oneshot_cancel(struct esp32_oneshot_s *oneshot,
                         struct timespec *ts)
{
  int ret = OK;
  uint64_t current_us;
  uint64_t remaining_us;
  uint64_t timeout_us;
  uint64_t counter_value;
  uint64_t alarm_value;

  DEBUGASSERT(oneshot);

  if (oneshot->running == false)
    {
      tmrinfo("Trying to cancel a non started oneshot timer.\n");
      ts->tv_sec  = 0;
      ts->tv_nsec = 0;
    }
  else
    {
      /* Stop timer */

      ESP32_TIM_STOP(oneshot->tim);

      /* Disable int */

      ESP32_TIM_DISABLEINT(oneshot->tim);

      /* Detach handler */

      ret = ESP32_TIM_SETISR(oneshot->tim, NULL, NULL);

      if (ts != NULL)
        {
          /* Get the current counter value */

          ESP32_TIM_GETCTR(oneshot->tim, &counter_value);

          /* Get the current configured timeout */

          ESP32_TIM_GETALRVL(oneshot->tim, &alarm_value);

          current_us = counter_value * oneshot->resolution;
          timeout_us = alarm_value   * oneshot->resolution;

          /* Remaining time (us) = timeout (us) - current (us) */

          remaining_us = timeout_us - current_us;
          ts->tv_sec   = remaining_us / USEC_PER_SEC;
          remaining_us = remaining_us - ts->tv_sec * USEC_PER_SEC;
          ts->tv_nsec  = remaining_us * NSEC_PER_USEC;
        }

        oneshot->running  = false;
        oneshot->handler  = NULL;
        oneshot->arg      = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_oneshot_current
 *
 * Description:
 *   Get the current time.
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           esp32_oneshot_initialize();
 *   usec    The maximum delay in us.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

int esp32_oneshot_current(struct esp32_oneshot_s *oneshot, uint64_t *usec)
{
  /* Get the current counter value */

  ESP32_TIM_GETCTR(oneshot->tim, usec);

  *usec = *usec * (uint64_t)oneshot->resolution;

  return OK;
}

#endif /* CONFIG_ESP32_ONESHOT */
