/****************************************************************************
 * arch/risc-v/src/bl808/bl808_wdt.c
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
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>
#include <nuttx/fs/ioctl.h>

#include "hardware/bl808_timer.h"
#include "riscv_internal.h"
#include "chip.h"
#include "bl808_wdt.h"

#ifdef CONFIG_BL808_WDT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WDT_CLK_SRC_1K 2
#define WDT_CLK_SRC_NONE 5

#define BL808_UNLOCK_WDT(n) \
  ({ \
    putreg32(0xbaba, BL808_WDT_KEY1(n)); \
    putreg32(0xeb10, BL808_WDT_KEY2(n)); \
  })

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl808_wdt_s
{
  const struct watchdog_ops_s *ops;
  int idx;
  xcpt_t callback;
  bool started;
  uint32_t timeout;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

int bl808_wdt_start(struct watchdog_lowerhalf_s *lower);
int bl808_wdt_stop(struct watchdog_lowerhalf_s *lower);
int bl808_wdt_keepalive(struct watchdog_lowerhalf_s *lower);
int bl808_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                        struct watchdog_status_s *status);
int bl808_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                         uint32_t timeout);
xcpt_t bl808_wdt_capture(struct watchdog_lowerhalf_s *lower,
                         xcpt_t callback);
int bl808_wdt_ioctl(struct watchdog_lowerhalf_s *lower,
                    int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct watchdog_ops_s bl808_wdt_ops =
  {
    .start = bl808_wdt_start,
    .stop = bl808_wdt_stop,
    .keepalive = bl808_wdt_keepalive,
    .getstatus = bl808_wdt_getstatus,
    .settimeout = bl808_wdt_settimeout,
    .capture = bl808_wdt_capture,
    .ioctl = bl808_wdt_ioctl,
  };

static struct bl808_wdt_s wdt0 =
  {
    .ops = &bl808_wdt_ops,
    .idx = 0,
    .callback = NULL,
    .started = false,
    .timeout = 0
  };

static struct bl808_wdt_s wdt1 =
  {
    .ops = &bl808_wdt_ops,
    .idx = 1,
    .callback = NULL,
    .started = false,
    .timeout = 0
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wdt_interrupt
 *
 * Description:
 *   Watchdog interrupt handler. Clears the interrupt and
 *   Calls the attached callback if there is one.
 *
 ****************************************************************************/

static int __wdt_interrupt(int irq, void *context, void *arg)
{
  struct bl808_wdt_s *priv = (struct bl808_wdt_s *)arg;

  /* Clear IRQ */

  BL808_UNLOCK_WDT(priv->idx);
  modifyreg32(BL808_WDT_ICLR(priv->idx), 0, WDT_CLEAR_IRQ);

  if (priv->callback != NULL)
    {
      priv->callback(irq, context, arg);
    }

  return OK;
}

/****************************************************************************
 * Name: bl808_wdt_start
 *
 * Description:
 *   Reset the time to the current timeout and start the watchdog.
 *
 * Input parameters:
 *   lower  - A pointer to the lower-half driver of the watchdog.
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int bl808_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  struct bl808_wdt_s *priv = (struct bl808_wdt_s *)lower;

  /* Enable clock */

  modifyreg32(BL808_TIMER_TCCR(priv->idx), WDT_CLKSEL_MASK,
              WDT_CLK_SRC_1K << WDT_CLKSEL_SHIFT);

  /* Clear counter */

  BL808_UNLOCK_WDT(priv->idx);
  modifyreg32(BL808_WDT_COUNT_CLEAR(priv->idx), 0, WDT_CLEAR_COUNT);
  while (getreg32(BL808_WDT_COUNTER(priv->idx)) != 0);
  BL808_UNLOCK_WDT(priv->idx);
  modifyreg32(BL808_WDT_COUNT_CLEAR(priv->idx), WDT_CLEAR_COUNT, 0);

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: bl808_wdt_stop
 *
 * Description:
 *   Stop the watchdog.
 *
 * Input parameters:
 *   lower  - A pointer to the lower-half driver of the watchdog.
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int bl808_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  struct bl808_wdt_s *priv = (struct bl808_wdt_s *)lower;

  /* WDTs are stopped by setting the input clock to NONE.
   * This is done to allow calling watchdog_stop and then get
   * the time left to timeout afterwards. If we used the
   * watchdog enable bits, the counter would reset to 0 when stopped.
   */

  modifyreg32(BL808_TIMER_TCCR(priv->idx), WDT_CLKSEL_MASK,
              WDT_CLK_SRC_NONE << WDT_CLKSEL_SHIFT);

  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: bl808_wdt_keepalive
 *
 * Description:
 *   Reset the watchdog to keep it running.
 *
 * Input parameters:
 *   lower  - A pointer to the lower-half driver of the watchdog.
 *
 * Returned Value:
 *   Error status. Returns an IO error if the watchdog is not
 *   running, otherwise returns OK.
 *
 ****************************************************************************/

int bl808_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct bl808_wdt_s *priv = (struct bl808_wdt_s *)lower;

  /* Check that the watchdog is running */

  if (priv->started == false)
    {
      return -EIO;
    }

  BL808_UNLOCK_WDT(priv->idx);
  modifyreg32(BL808_WDT_COUNT_CLEAR(priv->idx), 0, WDT_CLEAR_COUNT);
  while (getreg32(BL808_WDT_COUNTER(priv->idx)) != 0);
  BL808_UNLOCK_WDT(priv->idx);
  modifyreg32(BL808_WDT_COUNT_CLEAR(priv->idx), WDT_CLEAR_COUNT, 0);

  return OK;
}

/****************************************************************************
 * Name: bl808_wdt_getstatus
 *
 * Description:
 *   Get current watchdog status. Returns to status parameter.
 *
 * Input parameters:
 *   lower  - A pointer to the lower-half driver of the watchdog.
 *   status - Return location for the watchdog status.
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int bl808_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                        struct watchdog_status_s *status)
{
  struct bl808_wdt_s *priv = (struct bl808_wdt_s *)lower;

  status->flags = priv->started
    | ((priv->callback == NULL) << 1)
    | ((priv->callback != NULL) << 2);

  status->timeout = priv->timeout;

  uint32_t current_count = getreg32(BL808_WDT_COUNTER(priv->idx));
  status->timeleft = priv->timeout - current_count;

  return OK;
}

/****************************************************************************
 * Name: bl808_wdt_settimeout
 *
 * Description:
 *   Set a new timeout value and reset the watchdog.
 *
 * Input parameters:
 *   lower   - A pointer to the lower-half driver of the watchdog.
 *   timeout - Watchdog timeout, in milliseconds.
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int bl808_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                         uint32_t timeout)
{
  struct bl808_wdt_s *priv = (struct bl808_wdt_s *)lower;

  BL808_UNLOCK_WDT(priv->idx);
  modifyreg32(BL808_WDT_COMP(priv->idx), 0xffff, timeout);
  priv->timeout = timeout;

  /* Clock is needed to clear counter (assuming same behavior as GP timers) */

  modifyreg32(BL808_TIMER_TCCR(priv->idx), WDT_CLKSEL_MASK,
              WDT_CLK_SRC_1K << WDT_CLKSEL_SHIFT);

  BL808_UNLOCK_WDT(priv->idx);
  modifyreg32(BL808_WDT_COUNT_CLEAR(priv->idx), 0, WDT_CLEAR_COUNT);
  while (getreg32(BL808_WDT_COUNTER(priv->idx)) != 0);

  /* Disable clock to stop timer from running after clear */

  modifyreg32(BL808_TIMER_TCCR(priv->idx), WDT_CLKSEL_MASK,
              WDT_CLK_SRC_NONE << WDT_CLKSEL_SHIFT);

  BL808_UNLOCK_WDT(priv->idx);
  modifyreg32(BL808_WDT_COUNT_CLEAR(priv->idx), WDT_CLEAR_COUNT, 0);

  return OK;
}

/****************************************************************************
 * Name: bl808_wdt_capture
 *
 * Description:
 *   Assigns a callback to be called on timeout. If
 *   the callback is null, configure the watchdog
 *   as a reset source.
 *
 * Input parameters:
 *   lower    - A pointer to the lower-half driver of the watchdog.
 *   callback - Callback to run on watchdog timeout. If null,
 *              timeout should trigger a reset.
 *
 * Returned Value:
 *   The last assigned callback, or null if there was none.
 *
 ****************************************************************************/

xcpt_t bl808_wdt_capture(struct watchdog_lowerhalf_s *lower,
                         xcpt_t callback)
{
  struct bl808_wdt_s *priv = (struct bl808_wdt_s *)lower;
  xcpt_t prev_callback = priv->callback;
  priv->callback = callback;

  /* Configure watchdog mode */

  if (callback == NULL)
    {
      BL808_UNLOCK_WDT(priv->idx);
      modifyreg32(BL808_WDT_MODE(priv->idx), 0, WDT_RESET_EN);
      if (priv->idx == 0)
        {
          up_disable_irq(BL808_IRQ_WDT0);
        }
      else
        {
          up_disable_irq(BL808_IRQ_WDT1);
        }
    }
  else
    {
      BL808_UNLOCK_WDT(priv->idx);
      modifyreg32(BL808_WDT_MODE(priv->idx), WDT_RESET_EN, 0);
      if (priv->idx == 0)
        {
          up_enable_irq(BL808_IRQ_WDT0);
        }
      else
        {
          up_enable_irq(BL808_IRQ_WDT1);
        }
    }

  return prev_callback;
}

/****************************************************************************
 * Name: bl808_wdt_ioctl
 *
 * Description:
 *   Handle ioctl commands not recognized by upper-half.
 *
 * Input parameters:
 *   lower  - A pointer to the lower-half driver of the watchdog.
 *
 * Returned Value:
 *   Error status. Always returns an IO error because no additional
 *   ioctl methods are implemented.
 *
 ****************************************************************************/

int bl808_wdt_ioctl(struct watchdog_lowerhalf_s *lower,
                    int cmd, unsigned long arg)
{
  /* No additional ioctl commands implemented */

  return -EIO;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_wdt_init
 *
 * Description:
 *   Initialize watchdog hardware and register character drivers.
 *
 ****************************************************************************/

int bl808_wdt_init(void)
{
  int ret = OK;

  /* Watchdog 0 */

  modifyreg32(BL808_TIMER_TCCR(0), WDT_CLKSEL_MASK,
              WDT_CLK_SRC_NONE << WDT_CLKSEL_SHIFT);
  BL808_UNLOCK_WDT(0);
  modifyreg32(BL808_WDT_MODE(0), 0, WDT_EN | WDT_RESET_EN);
  ret |= irq_attach(BL808_IRQ_WDT0, __wdt_interrupt,
                    (void *)&wdt0);
  watchdog_register("/dev/watchdog0",
                    (struct watchdog_lowerhalf_s *)&wdt0);

  /* Watchdog 1 */

  modifyreg32(BL808_TIMER_TCCR(1), WDT_CLKSEL_MASK,
              WDT_CLK_SRC_NONE << WDT_CLKSEL_SHIFT);
  BL808_UNLOCK_WDT(1);
  modifyreg32(BL808_WDT_MODE(1), 0, WDT_EN | WDT_RESET_EN);
  ret |= irq_attach(BL808_IRQ_WDT1, __wdt_interrupt,
                    (void *)&wdt1);
  watchdog_register("/dev/watchdog1",
                    (struct watchdog_lowerhalf_s *)&wdt0);

  return ret;
}

#endif /* CONFIG_BL808_WDT */
