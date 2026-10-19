/****************************************************************************
 * boards/arm/gd32w51x/gd-xd-w515-eval/src/gd32_buttons.c
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

/* The "TSI" key of the board is a capacitive electrode measured by the
 * touch sensing interface.  It is sampled periodically on the work queue
 * while a handler is attached, and the handler is called on each change,
 * as for a button interrupt.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include <arch/board/board.h>

#include "gd32w51x_gpio.h"
#include "gd32w51x_tsi.h"

#include "gd-xd-w515-eval.h"

#ifdef CONFIG_ARCH_BUTTONS

#ifndef CONFIG_GD32W51X_TSI
#  error "The touch key needs CONFIG_GD32W51X_TSI"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "The touch key needs a work queue"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCHED_LPWORK
#  define TSI_WORK                LPWORK
#else
#  define TSI_WORK                HPWORK
#endif

/* Sampling period while a handler is attached */

#define TSI_PERIOD                MSEC2TICK(50)

/* Samples of the reference count (key not touched) at initialization */

#define TSI_CALIB_SAMPLES         8

/* The count drops by about 80 % when the key is touched (about 4470
 * cycles untouched, under 1200 with a finger on the board).  The key is
 * touched below 70 % of the reference and released above 85 %.
 */

#define TSI_TOUCH_PERCENT         70
#define TSI_RELEASE_PERCENT       85

/* Weight of a new untouched sample in the reference (1 / 2^n), to follow
 * slow drifts (temperature, humidity)
 */

#define TSI_DRIFT_SHIFT           4

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_reference;          /* Untouched count, x 2^DRIFT_SHIFT */
static bool g_touched;                /* Current state of the key */
static xcpt_t g_handler;              /* Attached handler */
static void *g_handler_arg;           /* Argument of the handler */
static struct work_s g_tsi_work;      /* Periodic sampling */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_tsi_sample
 *
 * Description:
 *   Acquire the key once and update its state and reference.
 *
 ****************************************************************************/

static bool gd32_tsi_sample(void)
{
  uint32_t reference = g_reference >> TSI_DRIFT_SHIFT;
  uint16_t count;

  if (gd32_tsi_acquire(BOARD_TSI_GROUP, BOARD_TSI_KEY_PIN, &count) < 0)
    {
      return g_touched;
    }

  if (g_touched)
    {
      if (count * 100 > reference * TSI_RELEASE_PERCENT)
        {
          g_touched = false;
        }
    }
  else if (count * 100 < reference * TSI_TOUCH_PERCENT)
    {
      g_touched = true;
    }
  else if (count * 100 > reference * TSI_RELEASE_PERCENT)
    {
      /* Untouched: follow the slow drifts of the reference */

      g_reference += count - reference;
    }

  return g_touched;
}

/****************************************************************************
 * Name: gd32_tsi_worker
 ****************************************************************************/

static void gd32_tsi_worker(void *arg)
{
  bool touched = g_touched;
  xcpt_t handler;
  void *handler_arg;
  irqstate_t flags;

  if (gd32_tsi_sample() != touched)
    {
      flags       = enter_critical_section();
      handler     = g_handler;
      handler_arg = g_handler_arg;
      leave_critical_section(flags);

      if (handler != NULL)
        {
          handler(0, NULL, handler_arg);
        }
    }

  flags = enter_critical_section();
  if (g_handler != NULL && work_available(&g_tsi_work))
    {
      work_queue(TSI_WORK, &g_tsi_work, gd32_tsi_worker, NULL,
                 TSI_PERIOD);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   Configure the touch key and measure its reference count.  The key must
 *   not be touched at this time.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  uint16_t count;
  uint32_t reference = 0;
  int i;

  gd32_gpio_config(GPIO_TSI_SAMPCAP);
  gd32_gpio_config(GPIO_TSI_KEY);

  if (gd32_tsi_initialize(GD32W51X_TSI_PIN(BOARD_TSI_GROUP,
                                           BOARD_TSI_SAMPLE_PIN)) < 0)
    {
      ierr("ERROR: Failed to initialize the TSI\n");
      return 0;
    }

  /* Keep the highest count, in case a finger was near the key */

  for (i = 0; i < TSI_CALIB_SAMPLES; i++)
    {
      if (gd32_tsi_acquire(BOARD_TSI_GROUP, BOARD_TSI_KEY_PIN,
                           &count) >= 0 && count > reference)
        {
          reference = count;
        }
    }

  if (reference == 0)
    {
      ierr("ERROR: No TSI count\n");
      return 0;
    }

  iinfo("TSI reference: %" PRIu32 "\n", reference);
  g_reference = reference << TSI_DRIFT_SHIFT;
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   Return the state of the touch key.  While the key is sampled
 *   periodically, its last state is returned.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  bool touched = g_handler != NULL ? g_touched : gd32_tsi_sample();

  return touched ? BUTTON_TSI_BIT : 0;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   Attach (or detach, with a NULL handler) the handler of the touch key.
 *   The key is sampled periodically while a handler is attached.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  irqstate_t flags;

  if (id != BUTTON_TSI)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  g_handler     = irqhandler;
  g_handler_arg = arg;

  if (irqhandler != NULL)
    {
      if (work_available(&g_tsi_work))
        {
          work_queue(TSI_WORK, &g_tsi_work, gd32_tsi_worker, NULL, 0);
        }
    }
  else
    {
      work_cancel(TSI_WORK, &g_tsi_work);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
