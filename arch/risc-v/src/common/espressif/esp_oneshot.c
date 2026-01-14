/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_oneshot.c
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
#include "soc/timer_periph.h"
#include "esp_private/esp_clk_tree_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Oneshot Timer is assigned to the Timer 0 of TimerGroup 1 */

#define GROUP_ID  1
#define TIMER_ID  0

/* Resolution of 1 microsecond */

#define ONESHOT_RESOLUTION    1

#if SOC_PERIPH_CLK_CTRL_SHARED
#  define ONESHOT_CLOCK_SRC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#  define ONESHOT_CLOCK_SRC_ATOMIC()
#endif

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
  bool                       running;     /* True: the timer is running */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int esp_oneshot_isr(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static clkcnt_t esp_oneshot_max_delay(struct oneshot_lowerhalf_s *lower);
static clkcnt_t esp_oneshot_current(struct oneshot_lowerhalf_s *lower);
static void esp_oneshot_start_absolute(struct oneshot_lowerhalf_s *lower,
                                              clkcnt_t expected);
static void esp_oneshot_start(struct oneshot_lowerhalf_s *lower,
                                  clkcnt_t delta);
static void esp_oneshot_cancel(struct oneshot_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct oneshot_operations_s g_oneshot_ops =
{
  .current        = esp_oneshot_current,
  .start          = esp_oneshot_start,
  .start_absolute = esp_oneshot_start_absolute,
  .cancel         = esp_oneshot_cancel,
  .max_delay      = esp_oneshot_max_delay
};

/* Oneshot Timer lower-half */

static struct esp_oneshot_lowerhalf_s g_oneshot_lowerhalf =
{
  .lh =
    {
      .ops = &g_oneshot_ops,
    },
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
 *
 * Returned Value:
 *   The maximum delay.
 *
 ****************************************************************************/

static clkcnt_t esp_oneshot_max_delay(struct oneshot_lowerhalf_s *lower)
{
  /* The real maximum delay surpass the limit that timespec can represent.
   * Even if considering the best case scenario of 1us resolution.
   * Therefore, here, fill the timespec with the maximum value supported
   * value.
   */

  return UINT64_MAX;
}

/****************************************************************************
 * Name: esp_oneshot_start/esp_oneshot_start_absolute
 *
 * Description:
 *   Start the oneshot timer.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   delta         - The count to start the oneshot.
 *
 * Returned Values:
 *   None.
 *
 ****************************************************************************/

static void esp_oneshot_start(struct oneshot_lowerhalf_s *lower,
                              clkcnt_t delta)
{
  struct esp_oneshot_lowerhalf_s *priv =
    (struct esp_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  tmrinfo("count=(%llu)\n", delta);

  if (priv->running)
    {
      tmrinfo("Oneshot timer already in use. Cancelling it...\n");

      /* If the oneshot timer was already started, cancel it and then
       * restart.
       */

      esp_oneshot_cancel(lower);
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

  timer_ll_set_alarm_value(hal->dev, hal->timer_id, delta);

  /* Enable timer alarm */

  timer_ll_enable_alarm(hal->dev, hal->timer_id, true);

  /* Clear Interrupt Bits Status */

  timer_ll_clear_intr_status(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id));

  /* Configure callback, in case a handler was provided before */

  timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                       true);

  /* Finally, start the timer */

  timer_ll_enable_counter(hal->dev, hal->timer_id, true);

  priv->running = true;
}

static void esp_oneshot_start_absolute(struct oneshot_lowerhalf_s *lower,
                                       clkcnt_t expected)
{
  struct esp_oneshot_lowerhalf_s *priv =
    (struct esp_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  uint64_t alarm   = expected;
  uint64_t counter = timer_hal_capture_and_get_counter_value(&priv->hal);

  /* In case of overflow. */

  counter = alarm - counter >= alarm ? 0 : alarm - counter;

  esp_oneshot_start(lower, counter);
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
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_oneshot_cancel(struct oneshot_lowerhalf_s *lower)
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
    }
  else
    {
      timer_hal_context_t *hal = &(priv->hal);
      timer_ll_enable_intr(hal->dev, TIMER_LL_EVENT_ALARM(hal->timer_id),
                           false);
      timer_ll_enable_counter(hal->dev, hal->timer_id, false);
    }

  priv->running = false;
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
 *
 * Returned Value:
 *   The current timer count.
 *
 ****************************************************************************/

static clkcnt_t esp_oneshot_current(struct oneshot_lowerhalf_s *lower)
{
  struct esp_oneshot_lowerhalf_s *priv =
    (struct esp_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Get the current counter value */

  return timer_hal_capture_and_get_counter_value(&priv->hal);
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

  /* Call the callback */

  oneshot_process_callback(&priv->lh);

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
  int ret = OK;
  periph_module_t periph;
  int irq;

  UNUSED(chan);

  /* Initialize the elements of lower half state structure */

  lower->running    = false;

  periph = timer_group_periph_signals.groups[GROUP_ID].module;

  PERIPH_RCC_ACQUIRE_ATOMIC(periph, ref_count)
    {
      if (ref_count == 0)
        {
          timer_ll_enable_bus_clock(GROUP_ID, true);
          timer_ll_reset_register(GROUP_ID);
        }
    }

  timer_hal_init(&lower->hal, GROUP_ID, TIMER_ID);

  ret = esp_clk_tree_enable_src((soc_module_clk_t)GPTIMER_CLK_SRC_DEFAULT,
                                true);
  if (ret != ESP_OK)
    {
      return NULL;
    }

  /* Configure clock source */

  timer_ll_set_clock_source(GROUP_ID, lower->hal.timer_id,
                            GPTIMER_CLK_SRC_DEFAULT);

  timer_ll_enable_clock(GROUP_ID, lower->hal.timer_id, true);

  /* Calculate the suitable prescaler according to the current APB
   * frequency to generate a period of 1 us.
   */

  esp_clk_tree_src_get_freq_hz((soc_module_clk_t)GPTIMER_CLK_SRC_DEFAULT,
                           ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                           &counter_src_hz);
  prescale = (counter_src_hz * resolution) / USEC_PER_SEC;

  /* Configure timer prescaler */

  timer_ll_set_clock_prescale(lower->hal.dev, lower->hal.timer_id, prescale);

  irq = timer_group_periph_signals.groups[GROUP_ID].timer_irq_id[TIMER_ID];

  esp_setup_irq(irq,
                ESP_IRQ_PRIORITY_DEFAULT,
                ESP_IRQ_TRIGGER_LEVEL);

  oneshot_count_init(&lower->lh, USEC_PER_SEC / resolution);

  /* Attach the handler for the timer IRQ */

  irq_attach(ESP_SOURCE2IRQ(irq), (xcpt_t)esp_oneshot_isr, lower);

  /* Enable the allocated CPU interrupt */

  up_enable_irq(ESP_SOURCE2IRQ(irq));

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
