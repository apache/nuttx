/****************************************************************************
 * arch/risc-v/src/espressif/esp_timer.c
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
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>

#include <nuttx/timers/timer.h>

#include "esp_irq.h"
#include "esp_timer.h"

#include "esp_clk_tree.h"
#include "esp_attr.h"
#include "hal/timer_hal.h"
#include "hal/timer_ll.h"
#include "periph_ctrl.h"
#include "soc/clk_tree_defs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure. This structure must be cast-compatible with the
 * well-known timer_lowerhalf_s structure.
 */

struct esp_timer_lowerhalf_s
{
  const struct timer_ops_s *ops;       /* Lower half operations */
  timer_hal_context_t       hal;       /* HAL context */
  int                       source;    /* Timer interrupt source */
  int                       irq;       /* IRQ associated with this Timer */
  tccb_t                    callback;  /* Current user interrupt callback */
  void                     *arg;       /* Argument passed to upper half callback */
  bool                      started;   /* True: Timer has been started */
  void                     *upper;     /* Pointer to timer_upperhalf_s */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int esp_timer_isr(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int esp_timer_start(struct timer_lowerhalf_s *lower);
static int esp_timer_stop(struct timer_lowerhalf_s *lower);
static int esp_timer_getstatus(struct timer_lowerhalf_s *lower,
                               struct timer_status_s *status);
static int esp_timer_settimeout(struct timer_lowerhalf_s *lower,
                                uint32_t timeout);
static void esp_timer_setcallback(struct timer_lowerhalf_s *lower,
                                  tccb_t callback, void *arg);
static int esp_timer_maxtimeout(struct timer_lowerhalf_s *lower,
                                uint32_t *timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = esp_timer_start,
  .stop        = esp_timer_stop,
  .getstatus   = esp_timer_getstatus,
  .settimeout  = esp_timer_settimeout,
  .setcallback = esp_timer_setcallback,
  .maxtimeout  = esp_timer_maxtimeout,
  .ioctl       = NULL
};

/* TIMER0 lower-half */

static struct esp_timer_lowerhalf_s g_timer0_lowerhalf =
{
  .ops = &g_timer_ops,
  .source = TG0_T0_LEVEL_INTR_SOURCE,
  .irq = ESP_IRQ_TG0_T0_LEVEL
};

/* TIMER1 lower-half */

static struct esp_timer_lowerhalf_s g_timer1_lowerhalf =
{
  .ops = &g_timer_ops,
  .source = TG1_T0_LEVEL_INTR_SOURCE,
  .irq = ESP_IRQ_TG1_T0_LEVEL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_timer_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int esp_timer_start(struct timer_lowerhalf_s *lower)
{
  struct esp_timer_lowerhalf_s *priv = (struct esp_timer_lowerhalf_s *)lower;
  uint32_t counter_src_hz = 0;
  uint32_t prescale;

  DEBUGASSERT(priv != NULL);

  if (priv->started)
    {
      /* Return EBUSY to indicate that the timer is already running */

      return -EBUSY;
    }

  timer_hal_context_t *hal = &(priv->hal);

  /* Make sure the timer is stopped to avoid unpredictable behavior */

  timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                       false);
  timer_ll_enable_counter(hal->dev, hal->timer_id, false);

  /* Configure clock source */

  timer_ll_set_clock_source(hal->dev, hal->timer_id,
                            GPTIMER_CLK_SRC_DEFAULT);

  /* Calculate the suitable prescaler according to the current APB
   * frequency to generate a period of 1 us.
   */

  esp_clk_tree_src_get_freq_hz((soc_module_clk_t)GPTIMER_CLK_SRC_DEFAULT,
                           ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                           &counter_src_hz);
  prescale = counter_src_hz / USEC_PER_SEC;

  /* Configure TIMER prescaler */

  timer_ll_set_clock_prescale(hal->dev, hal->timer_id, prescale);

  /* Configure TIMER mode */

  timer_ll_set_count_direction(hal->dev, hal->timer_id, GPTIMER_COUNT_UP);

  /* Clear TIMER counter value */

  timer_hal_set_counter_value(hal, 0);

  /* Enable autoreload */

  timer_ll_enable_auto_reload(hal->dev, hal->timer_id, true);

  /* Enable TIMER alarm */

  timer_ll_enable_alarm(hal->dev, hal->timer_id, true);

  /* Clear Interrupt Bits Status */

  timer_ll_clear_intr_status(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id));

  /* Configure callback, in case a handler was provided before */

  if (priv->callback != NULL)
    {
      timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                           true);
    }

  /* Finally, start the TIMER */

  timer_ll_enable_counter(hal->dev, hal->timer_id, true);

  priv->started = true;

  return OK;
}

/****************************************************************************
 * Name: esp_timer_stop
 *
 * Description:
 *   Stop the timer. In case a callback was previously configured,
 *   unregister and deallocate it.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int esp_timer_stop(struct timer_lowerhalf_s *lower)
{
  struct esp_timer_lowerhalf_s *priv = (struct esp_timer_lowerhalf_s *)lower;
  int ret = OK;

  DEBUGASSERT(priv != NULL);

  if (!priv->started)
    {
      /* Return ENODEV to indicate that the timer was not running */

      return -ENODEV;
    }

  timer_hal_context_t *hal = &(priv->hal);
  timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                       false);
  timer_ll_enable_counter(hal->dev, hal->timer_id, false);

  priv->started = false;
  priv->callback = NULL;

  return ret;
}

/****************************************************************************
 * Name: timer_getstatus
 *
 * Description:
 *   Get the current timer status
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   status        - The location to return the timer status information.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int esp_timer_getstatus(struct timer_lowerhalf_s *lower,
                               struct timer_status_s *status)
{
  struct esp_timer_lowerhalf_s *priv = (struct esp_timer_lowerhalf_s *)lower;
  uint64_t current_counter_value;
  uint64_t alarm_value;

  DEBUGASSERT(priv != NULL);

  timer_hal_context_t *hal = &(priv->hal);

  /* Return the status bit */

  status->flags = 0;

  if (priv->started)
    {
      /* TIMER is running */

      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      /* TIMER has a user callback function to be called when
       * expiration happens
       */

      status->flags |= TCFLAGS_HANDLER;
    }

  /* Get the current counter value */

  current_counter_value = timer_hal_capture_and_get_counter_value(hal);

  /* Get the current configured timeout */

  volatile timg_hwtimer_reg_t *hw_timer =
    &(hal->dev->hw_timer[hal->timer_id]);
  alarm_value = ((uint64_t)hw_timer->alarmhi.tx_alarm_hi << 32) |
                 (hw_timer->alarmlo.tx_alarm_lo);

  status->timeout  = (uint32_t)alarm_value;
  status->timeleft = (uint32_t)(alarm_value - current_counter_value);

  return OK;
}

/****************************************************************************
 * Name: esp_timer_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   timeout       - The new timeout value in microseconds.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int esp_timer_settimeout(struct timer_lowerhalf_s *lower,
                                uint32_t timeout)
{
  struct esp_timer_lowerhalf_s *priv = (struct esp_timer_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  timer_hal_context_t *hal = &(priv->hal);

  /* Set the timeout */

  timer_ll_set_alarm_value(hal->dev, hal->timer_id, (uint64_t)timeout);

  return OK;
}

/****************************************************************************
 * Name: esp_timer_setcallback
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of
 *                   the "lower-half" driver state structure.
 *   callback      - The new timer expiration function pointer. If this
 *                   function pointer is NULL, then the timer will be
 *                   disabled.
 *   arg           - Argument that will be provided in the callback
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void esp_timer_setcallback(struct timer_lowerhalf_s *lower,
                                  tccb_t callback, void *arg)
{
  struct esp_timer_lowerhalf_s *priv = (struct esp_timer_lowerhalf_s *)lower;
  timer_hal_context_t *hal = &(priv->hal);

  DEBUGASSERT(priv != NULL);

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  /* There is a user callback and the timer has already been started */

  bool enable = callback != NULL && priv->started;

  timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                       enable);
}

/****************************************************************************
 * Name: esp_timer_maxtimeout
 *
 * Description:
 *   Get the maximum timeout value.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of
 *                   the "lower-half" driver state structure.
 *   maxtimeout    - A pointer to the variable that will store the maximum
 *                   timeout value.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int esp_timer_maxtimeout(struct timer_lowerhalf_s *lower,
                                uint32_t *max_timeout)
{
  DEBUGASSERT(max_timeout != NULL);

  *max_timeout = UINT32_MAX;

  return OK;
}

/****************************************************************************
 * Name: esp_timer_handler
 *
 * Description:
 *   This is the Timer interrupt handler. It will be invoked when an
 *   interrupt is received on the device.
 *
 * Input Parameters:
 *   irq           - IRQ associated to that interrupt.
 *   context       - Interrupt register state save info.
 *   arg           - A pointer to the argument provided when the interrupt
 *                   was registered.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

IRAM_ATTR static int esp_timer_isr(int irq, void *context, void *arg)
{
  struct esp_timer_lowerhalf_s *priv = (struct esp_timer_lowerhalf_s *)arg;
  timer_hal_context_t *hal = &(priv->hal);
  uint32_t next_interval_us = 0;

  uint32_t intr_status = timer_ll_get_intr_status(hal->dev);

  if ((intr_status & TIMER_LL_EVENT_ALARM(hal->timer_id)) == 0)
    {
      return OK;
    }

  /* Clear the Interrupt */

  timer_ll_clear_intr_status(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id));

  if (priv->callback(&next_interval_us, priv->arg))
    {
      if (next_interval_us > 0)
        {
          /* Set a value to the alarm */

          timer_ll_set_alarm_value(hal->dev, hal->timer_id,
                                   (uint64_t)next_interval_us);
        }
    }
  else
    {
      esp_timer_stop((struct timer_lowerhalf_s *)priv);
    }

  timer_ll_enable_alarm(hal->dev, hal->timer_id, true);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_timer_initialize
 *
 * Description:
 *   Initialize a timer device.
 *
 * Input Parameters:
 *   timer_id      - ID of the hardware timer to be initialized.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_timer_initialize(uint32_t timer_id)
{
  struct esp_timer_lowerhalf_s *lower = NULL;
  char devpath[PATH_MAX];
  uint32_t group_num;
  uint32_t timer_num;

  switch (timer_id)
    {
      case 0:
        {
          periph_module_enable(PERIPH_TIMG0_MODULE);
          group_num = 0;
          timer_num = 0;

          lower = &g_timer0_lowerhalf;
        }
        break;
      case 1:
        {
          periph_module_enable(PERIPH_TIMG1_MODULE);
          group_num = 1;
          timer_num = 0;

          lower = &g_timer1_lowerhalf;
        }
        break;
      default:
        {
          return -ENODEV;
        }
        break;
    }

  snprintf(devpath, sizeof(devpath), "/dev/timer%" PRIu32, timer_id);

  /* Initialize the elements of lower half state structure */

  lower->callback = NULL;
  lower->started = false;
  timer_hal_init(&lower->hal, group_num, timer_num);

  /* Register the timer driver as /dev/timerX. If the registration goes
   * right the returned value from timer_register is a pointer to
   * timer_upperhalf_s that can be either used with timer_unregister()
   * or with the handler's arg.
   */

  lower->upper = timer_register(devpath, (struct timer_lowerhalf_s *)lower);
  if (lower->upper == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'devpath' were not unique). We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      return -EEXIST;
    }

  esp_setup_irq(lower->source,
                ESP_IRQ_PRIORITY_DEFAULT,
                ESP_IRQ_TRIGGER_LEVEL);

  /* Attach the handler for the timer IRQ */

  irq_attach(lower->irq, (xcpt_t)esp_timer_isr, lower);

  /* Enable the allocated CPU interrupt */

  up_enable_irq(lower->irq);

  return OK;
}
