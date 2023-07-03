/****************************************************************************
 * arch/risc-v/src/espressif/esp_oneshot.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>

#include <nuttx/compiler.h>
#include <nuttx/timers/oneshot.h>

#include "esp_irq.h"
#include "esp_oneshot.h"

#include "esp_clk_tree.h"
#include "esp_attr.h"
#include "hal/timer_hal.h"
#include "hal/timer_ll.h"
#include "periph_ctrl.h"
#include "soc/clk_tree_defs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Oneshot Timer is assigned to the Timer 0 of TimerGroup 1 */

#define ONESHOT_TIMERGROUP_ID 1
#define ONESHOT_TIMER_ID      0

/* Resolution of 1 microsecond */

#define ONESHOT_RESOLUTION    1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure. This structure must be cast-compatible with the
 * well-known oneshot_lowerhalf_s structure.
 */

struct esp_oneshot_lowerhalf_s
{
  struct oneshot_lowerhalf_s lh;          /* Lower half instance */
  timer_hal_context_t        hal;         /* HAL context */
  oneshot_callback_t         callback;    /* Current user interrupt callback */
  void                      *arg;         /* Argument passed to upper half callback */
  uint16_t                   resolution;  /* Timer resolution in microseconds */
  bool                       running;     /* True: the timer is running */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int esp_oneshot_isr(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int esp_oneshot_maxdelay(struct oneshot_lowerhalf_s *lower,
                                struct timespec *ts);
static int esp_oneshot_start(struct oneshot_lowerhalf_s *lower,
                             oneshot_callback_t callback,
                             void *arg,
                             const struct timespec *ts);
static int esp_oneshot_cancel(struct oneshot_lowerhalf_s *lower,
                              struct timespec *ts);
static int esp_oneshot_current(struct oneshot_lowerhalf_s *lower,
                               struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct oneshot_operations_s g_oneshot_ops =
{
  .max_delay = esp_oneshot_maxdelay,
  .start     = esp_oneshot_start,
  .cancel    = esp_oneshot_cancel,
  .current   = esp_oneshot_current
};

/* Oneshot Timer lower-half */

static struct esp_oneshot_lowerhalf_s g_oneshot_lowerhalf =
{
  .lh =
    {
      .ops = &g_oneshot_ops,
    },
  .callback = NULL,
  .arg = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_oneshot_maxdelay
 *
 * Description:
 *   Determine the maximum delay of the oneshot timer (in microseconds).
 *
 * Input Parameters:
 *   lower         - An instance of the lower-half oneshot state structure.
 *                   This structure must have been previously initialized via
 *                   a call to oneshot_initialize().
 *   ts            - The location in which to return the maximum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int esp_oneshot_maxdelay(struct oneshot_lowerhalf_s *lower,
                                struct timespec *ts)
{
  DEBUGASSERT(ts != NULL);

  /* The real maximum delay surpass the limit that timespec can represent.
   * Even if considering the best case scenario of 1us resolution.
   * Therefore, here, fill the timespec with the maximum value supported
   * value.
   */

  ts->tv_sec  = UINT32_MAX;
  ts->tv_nsec = NSEC_PER_SEC - 1;

  tmrinfo("max sec=%" PRIu32 "\n", ts->tv_sec);
  tmrinfo("max nsec=%ld\n", ts->tv_nsec);

  return OK;
}

/****************************************************************************
 * Name: esp_oneshot_start
 *
 * Description:
 *   Start the oneshot timer.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   callback      - Function to call when when the oneshot timer expires.
 *                   Inside the handler scope.
 *   arg           - A pointer to the argument that will accompany the
 *                   callback.
 *   ts            - Provides the duration of the oneshot timer.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int esp_oneshot_start(struct oneshot_lowerhalf_s *lower,
                             oneshot_callback_t callback,
                             void *arg,
                             const struct timespec *ts)
{
  struct esp_oneshot_lowerhalf_s *priv =
    (struct esp_oneshot_lowerhalf_s *)lower;
  uint64_t timeout_us;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(callback != NULL);
  DEBUGASSERT(ts != NULL);

  tmrinfo("callback=%p arg=%p, ts=(%lu, %ld)\n",
          callback, arg, (unsigned long)ts->tv_sec, ts->tv_nsec);

  if (priv->running)
    {
      tmrinfo("Oneshot timer already in use. Cancelling it...\n");

      /* If the oneshot timer was already started, cancel it and then
       * restart.
       */

      esp_oneshot_cancel(lower, NULL);
    }

  /* Save the new callback and its argument */

  priv->callback = callback;
  priv->arg      = arg;

  /* Retrieve the duration from timespec in microsecond */

  timeout_us = (uint64_t)ts->tv_sec * USEC_PER_SEC +
               (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  /* Verify if it is a multiple of the configured resolution.
   * In case it isn't, warn the user.
   */

  if ((timeout_us % priv->resolution) != 0)
    {
      tmrwarn("The interval is not multiple of the resolution.\n"
              "Adjust the resolution in your bringup file.\n");
    }

  timer_hal_context_t *hal = &(priv->hal);

  /* Make sure the timer is stopped to avoid unpredictable behavior */

  timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                       false);
  timer_ll_enable_counter(hal->dev, hal->timer_id, false);

  /* Configure timer mode */

  timer_ll_set_count_direction(hal->dev, hal->timer_id, GPTIMER_COUNT_UP);

  /* Clear timer counter value */

  timer_hal_set_counter_value(hal, 0);

  /* Enable autoreload */

  timer_ll_enable_auto_reload(hal->dev, hal->timer_id, true);

  /* Set the timeout */

  timer_ll_set_alarm_value(hal->dev, hal->timer_id,
                           timeout_us / priv->resolution);

  /* Enable timer alarm */

  timer_ll_enable_alarm(hal->dev, hal->timer_id, true);

  /* Clear Interrupt Bits Status */

  timer_ll_clear_intr_status(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id));

  /* Configure callback, in case a handler was provided before */

  if (priv->callback != NULL)
    {
      timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                           true);
    }

  /* Finally, start the timer */

  timer_ll_enable_counter(hal->dev, hal->timer_id, true);

  priv->running = true;

  return OK;
}

/****************************************************************************
 * Name: esp_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   ts            - The location in which to return the time remaining on
 *                   the oneshot timer. A time of zero is returned if the
 *                   timer is not running. ts may be zero in which case the
 *                   time remaining is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int esp_oneshot_cancel(struct oneshot_lowerhalf_s *lower,
                              struct timespec *ts)
{
  struct esp_oneshot_lowerhalf_s *priv =
    (struct esp_oneshot_lowerhalf_s *)lower;
  uint64_t current_us;
  uint64_t remaining_us;
  uint64_t timeout_us;
  uint64_t counter_value;
  uint64_t alarm_value;

  DEBUGASSERT(priv != NULL);

  if (!priv->running)
    {
      tmrinfo("Trying to cancel a non started oneshot timer.\n");
      ts->tv_sec  = 0;
      ts->tv_nsec = 0;

      return -EFAULT;
    }

  timer_hal_context_t *hal = &(priv->hal);
  timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                       false);
  timer_ll_enable_counter(hal->dev, hal->timer_id, false);

  if (ts != NULL)
    {
      /* Get the current counter value */

      counter_value = timer_hal_capture_and_get_counter_value(hal);

      /* Get the current configured timeout */

      volatile timg_hwtimer_reg_t *hw_timer =
        &(hal->dev->hw_timer[hal->timer_id]);
      alarm_value = ((uint64_t)hw_timer->alarmhi.tx_alarm_hi << 32) |
                    (hw_timer->alarmlo.tx_alarm_lo);

      current_us = counter_value * priv->resolution;
      timeout_us = alarm_value   * priv->resolution;

      /* Remaining time (us) = timeout (us) - current (us) */

      remaining_us = timeout_us - current_us;
      ts->tv_sec   = remaining_us / USEC_PER_SEC;
      remaining_us = remaining_us - ts->tv_sec * USEC_PER_SEC;
      ts->tv_nsec  = remaining_us * NSEC_PER_USEC;
    }

  priv->running  = false;
  priv->callback = NULL;
  priv->arg      = NULL;

  return OK;
}

/****************************************************************************
 * Name: esp_oneshot_current
 *
 * Description:
 *  Get the current time.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   ts            - The location in which to return the current time. A time
 *                   of zero is returned for the initialization moment.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int esp_oneshot_current(struct oneshot_lowerhalf_s *lower,
                               struct timespec *ts)
{
  struct esp_oneshot_lowerhalf_s *priv =
    (struct esp_oneshot_lowerhalf_s *)lower;
  uint64_t current_us;
  uint64_t current_counter_value;
  uint64_t alarm_value;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(ts != NULL);

  timer_hal_context_t *hal = &(priv->hal);

  /* Get the current counter value */

  current_counter_value = timer_hal_capture_and_get_counter_value(hal);
  current_us = current_counter_value * priv->resolution;

  ts->tv_sec  = current_us / USEC_PER_SEC;
  current_us  = current_us - ts->tv_sec * USEC_PER_SEC;
  ts->tv_nsec = current_us * NSEC_PER_USEC;

  return OK;
}

/****************************************************************************
 * Name: esp_oneshot_isr
 *
 * Description:
 *   This is the oneshot timer interrupt handler. It will be invoked when an
 *   interrupt is received on the device.
 *
 * Input Parameters:
 *   irq           - IRQ associated to that interrupt.
 *   context       - Interrupt register state save info.
 *   arg           - A pointer to the argument provided when the interrupt
 *                   was registered.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

IRAM_ATTR static int esp_oneshot_isr(int irq, void *context, void *arg)
{
  struct esp_oneshot_lowerhalf_s *priv =
    (struct esp_oneshot_lowerhalf_s *)arg;
  oneshot_callback_t callback;
  void *callback_arg;

  timer_hal_context_t *hal = &(priv->hal);
  uint32_t intr_status = timer_ll_get_intr_status(hal->dev);

  if ((intr_status & TIMER_LL_EVENT_ALARM(hal->timer_id)) == 0)
    {
      return OK;
    }

  /* Clear the Interrupt */

  timer_ll_clear_intr_status(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id));

  /* The timer is no longer running */

  priv->running = false;

  /* Forward the event, clearing out any vestiges */

  callback       = priv->callback;
  callback_arg   = priv->arg;
  priv->callback = NULL;
  priv->arg      = NULL;

  /* Call the callback */

  callback(&priv->lh, callback_arg);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower half driver
 *   instance.
 *
 * Input Parameters:
 *   chan          - Timer counter channel to be used.
 *   resolution    - The required resolution of the timer in units of
 *                   microseconds. NOTE that the range is restricted to the
 *                   range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned. NULL is returned on any failure.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *oneshot_initialize(int chan, uint16_t resolution)
{
  struct esp_oneshot_lowerhalf_s *lower = &g_oneshot_lowerhalf;
  uint32_t counter_src_hz = 0;
  uint32_t prescale;

  UNUSED(chan);

  periph_module_enable(PERIPH_TIMG1_MODULE);

  /* Initialize the elements of lower half state structure */

  lower->callback   = NULL;
  lower->arg        = NULL;
  lower->resolution = resolution;
  lower->running    = false;
  timer_hal_init(&lower->hal, ONESHOT_TIMERGROUP_ID, ONESHOT_TIMER_ID);

  /* Configure clock source */

  timer_ll_set_clock_source(lower->hal.dev, lower->hal.timer_id,
                            GPTIMER_CLK_SRC_DEFAULT);

  /* Calculate the suitable prescaler according to the current APB
   * frequency to generate a period of 1 us.
   */

  esp_clk_tree_src_get_freq_hz((soc_module_clk_t)GPTIMER_CLK_SRC_DEFAULT,
                           ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                           &counter_src_hz);
  prescale = (counter_src_hz * resolution) / USEC_PER_SEC;

  /* Configure timer prescaler */

  timer_ll_set_clock_prescale(lower->hal.dev, lower->hal.timer_id, prescale);

  esp_setup_irq(TG1_T0_LEVEL_INTR_SOURCE,
                ESP_IRQ_PRIORITY_DEFAULT,
                ESP_IRQ_TRIGGER_LEVEL);

  /* Attach the handler for the timer IRQ */

  irq_attach(ESP_IRQ_TG1_T0_LEVEL, (xcpt_t)esp_oneshot_isr, lower);

  /* Enable the allocated CPU interrupt */

  up_enable_irq(ESP_IRQ_TG1_T0_LEVEL);

  return (struct oneshot_lowerhalf_s *)lower;
}

/****************************************************************************
 * Name: esp_oneshot_initialize
 *
 * Description:
 *   Initialize a timer device.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_oneshot_initialize(void)
{
  struct oneshot_lowerhalf_s *lower = oneshot_initialize(0,
                                                         ONESHOT_RESOLUTION);
  if (lower == NULL)
    {
      tmrerr("Failed to initialize oneshot timer\n");

      return -EBUSY;
    }

#ifdef CONFIG_CPULOAD_ONESHOT
  /* Configure the oneshot timer to support CPU load measurement */

  nxsched_oneshot_extclk(os_lower);

#else
  int ret = oneshot_register("/dev/oneshot", lower);
  if (ret < 0)
    {
      tmrerr("Failed to register oneshot: %d\n", ret);

      return ret;
    }
#endif /* CONFIG_CPULOAD_ONESHOT */

  return OK;
}
