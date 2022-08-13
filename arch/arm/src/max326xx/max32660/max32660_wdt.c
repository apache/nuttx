/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_wdt.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/max326_wdt.h"
#include "max326_clockconfig.h"
#include "max326_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_MAX326XX_WDOG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The watchdog is always running with source clock Fpclk.  There are two
 * timeout periods:
 *
 * INTPERIOD - Number of PCLK cycles until a watchdog timer interrupt is
 *             generated. This period must be less than the Reset Period
 *             Timeout for the watchdog timer interrupt to occur.
 * RSTPERIOD - The number of PCLK cycles until a system reset event occurs.
 *
 *  Tint = 2 ^ (31 - INTPERIOD) / Fpclk
 *  Trst = 2 ^ (31 - RSTPERIOD) / Fpclk
 */

/* If an interrupt is selected, then the reset will follow the interrupt by
 * this fixed timer period.
 */

#define WDT_RESET_DELAY (5 * MSEC_PER_SEC)  /* Five seconds in units of msec */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct max326_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops; /* Lower half operations */
  uint8_t exp;                      /* log12(Reset time period) */
  xcpt_t handler;                   /* User interrupt handler */
  clock_t lastping;                 /* Time of last WDT reset */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void
max326_wdog_reset(struct max326_wdt_lowerhalf_s *priv);
static void max326_int_enable(struct max326_wdt_lowerhalf_s *priv);
static uint32_t max326_time_left(struct max326_wdt_lowerhalf_s *priv);
static uint64_t max326_exp2msec(uint32_t pclk, uint8_t exp);
static uint8_t max326_msec2exp(uint32_t msec);

/* "Lower half" driver methods **********************************************/

static int max326_start(struct watchdog_lowerhalf_s *lower);
static int max326_stop(struct watchdog_lowerhalf_s *lower);
static int max326_keepalive(struct watchdog_lowerhalf_s *lower);
static int max326_getstatus(struct watchdog_lowerhalf_s *lower,
                            struct watchdog_status_s *status);
static int max326_settimeout(struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout);
static xcpt_t max326_capture(struct watchdog_lowerhalf_s *lower,
                             xcpt_t handler);
static int max326_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdtops =
{
  .start      = max326_start,
  .stop       = max326_stop,
  .keepalive  = max326_keepalive,
  .getstatus  = max326_getstatus,
  .settimeout = max326_settimeout,
  .capture    = max326_capture,
  .ioctl      = max326_ioctl,
};

/* "Lower half" driver state */

static struct max326_wdt_lowerhalf_s g_wdtdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_wdog_reset
 *
 * Description:
 *   Reset the watchdog timer
 *
 * Input Parameters:
 *   priv - WDT state instance.
 *
 * Returned Values:
 *   None.
 *
 ****************************************************************************/

static inline void max326_wdog_reset(struct max326_wdt_lowerhalf_s *priv)
{
  putreg32(WDT0_RST_SEQ1, MAX326_WDT0_RST);
  putreg32(WDT0_RST_SEQ2, MAX326_WDT0_RST);

  priv->lastping = clock_systime_ticks();
}

/****************************************************************************
 * Name: max326_int_enable
 *
 * Description:
 *   Enable reset or interrupts
 *
 * Input Parameters:
 *   priv - WDT state instance.
 *
 * Returned Values:
 *   None.
 *
 ****************************************************************************/

static void max326_int_enable(struct max326_wdt_lowerhalf_s *priv)
{
  uint32_t ctrl;

  /* Set clear reset/interrupt enables */

  ctrl  = getreg32(MAX326_WDT0_CTRL);
  ctrl &= ~(WDT0_CTRL_INTEN | WDT0_CTRL_RSTEN);

  /* Is there and interrupt handler attached? */

  if (priv->handler != NULL)
    {
      /* Yes.. attach handler and enable the interrupt */

      irq_attach(MAX326_IRQ_WDT0, priv->handler, priv);
      up_enable_irq(MAX326_IRQ_WDT0);

      /* Select the interrupt behavior (de-selecting the reset behavior) */

      ctrl |= WDT0_CTRL_INTEN;
    }
  else
    {
      /* No.. disable and detach the interrupt */

      up_disable_irq(MAX326_IRQ_WDT0);
      irq_detach(MAX326_IRQ_WDT0);

      /* Select the reset behavior (de-selecting the interrupt behavior) */

      ctrl |= WDT0_CTRL_RSTEN;
    }

  putreg32(ctrl, MAX326_WDT0_CTRL);
}

/****************************************************************************
 * Name: max326_time_left
 *
 * Description:
 *   Return the time left before reset or interrupt in milliseconds.
 *
 * Input Parameters:
 *   priv - WDT state instance.
 *
 * Returned Values:
 *   None.
 *
 ****************************************************************************/

static uint32_t max326_time_left(struct max326_wdt_lowerhalf_s *priv)
{
  uint64_t elapsed;
  uint64_t timeout;
  uint64_t timeleft;
  uint32_t ctrl;
  uint8_t exp;

  /* Which timeout applies?  To the interrupt or to the reset>?  Doesn't
   * matter, the interrupt time period is the right answer in either case.
   */

  ctrl = getreg32(MAX326_WDT0_CTRL);
  exp  = (ctrl & WDT0_CTRL_INTPERIOD_MASK) >> WDT0_CTRL_INTPERIOD_SHIFT;

  timeout = max326_exp2msec(max326_pclk_frequency(), exp);
  elapsed = TICK2MSEC(clock_systime_ticks() - priv->lastping);

  if (elapsed > timeout)
    {
      timeleft = 0;
    }
  else
    {
      timeleft = timeout - elapsed;
      if (timeleft > UINT32_MAX)
        {
          timeleft = UINT32_MAX;
        }
    }

  return (uint32_t)timeleft;
}

/****************************************************************************
 * Name: max326_exp2msec
 *
 * Description:
 *   Convert a max3660 time exponent to a time period in milliseconds.
 *
 * Input Parameters:
 *   pclk - Peripheral clock frequency.
 *   exp -  max32660 time exponent
 *
 * Returned Values:
 *   The time period in milliseconds.
 *
 ****************************************************************************/

static uint64_t max326_exp2msec(uint32_t pclk, uint8_t exp)
{
  return 1000LL * ((1LL << (31 - exp)) / (uint64_t)pclk);
}

/****************************************************************************
 * Name: max326_msec2exp
 *
 * Description:
 *   Convert a time period in milliseconds to the time exponent used by the
 *   max32660.
 *
 * Input Parameters:
 *   priv - WDT state instance.
 *   msec - Time period in milliseconds
 *
 * Returned Values:
 *   The max32660 time exponent.
 *
 ****************************************************************************/

static uint8_t max326_msec2exp(uint32_t msec)
{
  int64_t error;
  uint32_t timeout;
  uint32_t candidate;
  uint32_t pclk;

  wdinfo("Entry: msec=%lu\n", (unsigned long)msec);

  /* Convert the timeout in millisconds to the timeut exponent value.
   *
   * Tsec = (2 ^ (31 - timeout)) / Fpclk.
   */

  timeout = 0;
  error   = INT64_MAX;
  pclk    = max326_pclk_frequency();

  for (candidate = 0; candidate < 16; candidate++)
    {
      int64_t candmsec = max326_exp2msec(pclk, candidate);
      int64_t canderr  = msec - candmsec;

      if (canderr < 0)
        {
          canderr = -canderr;
        }

      if (canderr < error)
        {
          error   = canderr;
          timeout = candidate;
        }
    }

  return timeout;
}

/****************************************************************************
 * Name: max326_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int max326_start(struct watchdog_lowerhalf_s *lower)
{
  struct max326_wdt_lowerhalf_s *priv =
    (struct max326_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t ctrl;

  DEBUGASSERT(priv != NULL);

  wdinfo("Entry: enabled=%s timeout=%lu\n",
         (getreg32(MAX326_WDT0_CTRL) & WDT0_CTRL_WDTEN) != 0 ? "Yes" : "No",
         (unsigned long)1 << (31 - priv->exp));

  /* Perform the reset sequence */

  flags = spin_lock_irqsave(NULL);
  max326_wdog_reset(priv);

  /* Enable reset or interrupt */

  max326_int_enable(priv);

  /* Then enable the watchdog timer */

  ctrl  = getreg32(MAX326_WDT0_CTRL);
  ctrl |= WDT0_CTRL_WDTEN;
  putreg32(ctrl, MAX326_WDT0_CTRL);

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Name: max326_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int max326_stop(struct watchdog_lowerhalf_s *lower)
{
  irqstate_t flags;
  uint32_t ctrl;

  DEBUGASSERT(priv != NULL);

  wdinfo("Entry: enabled=%s timeout=%lu\n",
         (getreg32(MAX326_WDT0_CTRL) & WDT0_CTRL_WDTEN) != 0 ? "Yes" : "No",
         (unsigned long)1 << (31 - priv->exp));

  /* Disable the watchdog timer, reset, and interrupts */

  flags = spin_lock_irqsave(NULL);
  ctrl  = getreg32(MAX326_WDT0_CTRL);
  ctrl &= ~(WDT0_CTRL_WDTEN | WDT0_CTRL_INTEN | WDT0_CTRL_RSTEN);

  up_disable_irq(MAX326_IRQ_WDT0);
  irq_detach(MAX326_IRQ_WDT0);

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Name: max326_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int max326_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct max326_wdt_lowerhalf_s *priv =
    (struct max326_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Ping!\n");

  /* Reset WDT timer */

  flags = spin_lock_irqsave(NULL);
  max326_wdog_reset(priv);
  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Name: max326_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the
 *            "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int max326_getstatus(struct watchdog_lowerhalf_s *lower,
                            struct watchdog_status_s *status)
{
  struct max326_wdt_lowerhalf_s *priv =
    (struct max326_wdt_lowerhalf_s *)lower;
  uint64_t msec;

  wdinfo("Entry\n");
  DEBUGASSERT(priv != NULL && status != NULL);

  /* Return the status flags */

  status->flags = 0;
  if ((getreg32(MAX326_WDT0_CTRL) & WDT0_CTRL_WDTEN) != 0)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  status->flags |= priv->handler != NULL ? WDFLAGS_CAPTURE : WDFLAGS_RESET;

  /* Get the timeout value in milliseconds. */

  msec = max326_exp2msec(max326_pclk_frequency(), priv->exp);
  if (msec > UINT32_MAX)
    {
      msec = UINT32_MAX;
    }

  status->timeout = (uint32_t)msec;

  /* Return the time left until the expiration */

  status->timeleft = max326_time_left(priv);

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", status->flags);
  wdinfo("  timeout  : %d\n",   status->timeout);
  wdinfo("  timeleft : %d\n",   status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: max326_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int max326_settimeout(struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout)
{
  struct max326_wdt_lowerhalf_s *priv =
    (struct max326_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t ctrl;
  uint8_t exp;

  wdinfo("Entry: timeout=%lu\n", (unsigned long)timeout);
  DEBUGASSERT(priv != NULL);

  /* Reset WDT timer */

  flags = spin_lock_irqsave(NULL);
  max326_wdog_reset(priv);

  /* Convert the timeout value in milliseconds to time exponent used by the
   * max32660.
   */

  exp = max326_msec2exp(timeout);

  /* Update the selected intervals in the CTRL register.  Note that the
   * interrupt and the reset intervals are set to the same value.  The
   * User Guide requires that the reset interval be longer than the
   * interrupt interval.  This is thought to be no problem because in
   * this implementation, either the interrupt or reset behavior is
   * implemented, but not both.
   */

  ctrl  = getreg32(MAX326_WDT0_CTRL);
  ctrl &= ~(WDT0_CTRL_INTPERIOD_MASK | WDT0_CTRL_RSTPERIOD_MASK);
  ctrl |= (WDT0_CTRL_INTPERIOD(exp) | WDT0_CTRL_RSTPERIOD(exp));
  putreg32(ctrl, MAX326_WDT0_CTRL);

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Name: max326_capture
 *
 * Description:
 *   Attach the user WDT interrupt handler and enable the interrupt.
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   handler - The new WDT interrupt handler.
 *
 * Returned Values:
 *   The previous WDT interrupt handler.
 *
 ****************************************************************************/

static xcpt_t max326_capture(struct watchdog_lowerhalf_s *lower,
                             xcpt_t handler)
{
  struct max326_wdt_lowerhalf_s *priv =
    (struct max326_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  DEBUGASSERT(priv != NULL)
  wdinfo("Handler=%p\n", handler);

  /* Get the old handler */

  flags = spin_lock_irqsave(NULL);
  oldhandler = priv->handler;

  /* Save the new handler */

  priv->handler = handler;

  /* Reset the WDT timer */

  max326_wdog_reset(priv);

  /* Are we adding, removing, or changing a handler?  */

  if ((handler != NULL && oldhandler == NULL) || /* Adding a handler */
      (handler == NULL && oldhandler != NULL))   /* Removing a handler */
    {
      /* Disable/disable the interrupt and reset events appropriately. */

      max326_int_enable(priv);
    }

  spin_unlock_irqrestore(NULL, flags);
  return oldhandler;
}

/****************************************************************************
 * Name: max326_ioctl
 *
 * Description:
 *   Handle IOCTL commands forwarded from the upper half driver
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   cmd     - IOCTL command
 *   arg     - Argument associated with the IOCTL command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int max326_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                        unsigned long arg)
{
  return -ENOSYS;  /* None implemented */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_wdt_initialize
 *
 * Description:
 *   Initialize the WDT watchdog time.  The watchdog timer is initialized and
 *   registers as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *             /dev/watchdog0
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int max326_wdt_initialize(const char *devpath)
{
  struct max326_wdt_lowerhalf_s *priv = &g_wdtdev;
  void *handle;

  wdinfo("Entry: devpath=%s, mode_sleep=%d, mode_halt=%d\n",
         devpath, mode_sleep, mode_halt);

  /* Initialize the driver state structure. */

  priv->ops = &g_wdtops;

  /* Register the watchdog driver as /dev/watchdog0 */

  handle = watchdog_register(devpath,
                             (struct watchdog_lowerhalf_s *)priv);
  return (handle != NULL) ? OK : -ENODEV;
}

#endif /* CONFIG_WATCHDOG && CONFIG_MAX326XX_WDOG */
