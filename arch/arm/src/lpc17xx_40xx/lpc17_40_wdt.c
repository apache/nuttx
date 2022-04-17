/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_wdt.c
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
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>
#include <nuttx/clock.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/lpc17_40_wdt.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_LPC17_40_WDT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default WDT timeout value, in ms. */

#define LPC17_40_WDT_DEFTIMEOUT  1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct lpc17_40_lowerhalf_s
{
  const struct watchdog_ops_s *ops; /* Lower half operations */
  uint32_t timeout;                 /* The (actual) selected timeout */
  uint32_t lastreset;               /* The last reset time */
  bool     started;                 /* true: The watchdog timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int      lpc17_40_start(struct watchdog_lowerhalf_s *lower);
static int      lpc17_40_stop(struct watchdog_lowerhalf_s *lower);
static int      lpc17_40_keepalive(struct watchdog_lowerhalf_s *lower);
static int      lpc17_40_getstatus(struct watchdog_lowerhalf_s *lower,
                  struct watchdog_status_s *status);
static int      lpc17_40_settimeout(struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = lpc17_40_start,
  .stop       = lpc17_40_stop,
  .keepalive  = lpc17_40_keepalive,
  .getstatus  = lpc17_40_getstatus,
  .settimeout = lpc17_40_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct lpc17_40_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout.
 *
 * Input Parameters:
 *   lower - A pointer to the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc17_40_start(struct watchdog_lowerhalf_s *lower)
{
  struct lpc17_40_lowerhalf_s *priv =
      (struct lpc17_40_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t wdmod;

  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      flags = enter_critical_section();

      priv->started = true;
      priv->lastreset = clock_systime_ticks();

      wdmod = getreg32(LPC17_40_WDT_MOD);
      wdmod |= (WDT_MOD_WDEN | WDT_MOD_WDRESET);
      putreg32(wdmod, LPC17_40_WDT_MOD);

      putreg32(WDT_FEED_KEY_1, LPC17_40_WDT_FEED);
      putreg32(WDT_FEED_KEY_2, LPC17_40_WDT_FEED);

      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: lpc17_40_stop
 *
 * Description:
 *   Stop the watchdog timer.
 *
 * Input Parameters:
 *   lower - A pointer to the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc17_40_stop(struct watchdog_lowerhalf_s *lower)
{
  /* There is no way to disable WDT once it has been started. */

  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc17_40_keepalive
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
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc17_40_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct lpc17_40_lowerhalf_s *priv =
      (struct lpc17_40_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Reload the WDT. */

  flags = enter_critical_section();
  putreg32(WDT_FEED_KEY_1, LPC17_40_WDT_FEED);
  putreg32(WDT_FEED_KEY_2, LPC17_40_WDT_FEED);
  priv->lastreset = clock_systime_ticks();
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: lpc17_40_getstatus
 *
 * Description:
 *   Get the current watchdog timer status.
 *
 * Input Parameters:
 *   lower  - A pointer to the publicly visible representation of the
 *            "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc17_40_getstatus(struct watchdog_lowerhalf_s *lower,
                              struct watchdog_status_s *status)
{
  struct lpc17_40_lowerhalf_s *priv =
      (struct lpc17_40_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  DEBUGASSERT(priv);

  /* Return the status bit. */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout in milliseconds. */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping. */

  ticks   = clock_systime_ticks() - priv->lastreset;
  elapsed = (int32_t)TICK2MSEC(ticks);

  if (elapsed > priv->timeout)
    {
      elapsed = priv->timeout;
    }

  /* Return the approximate time until the watchdog timer expiration. */

  status->timeleft = priv->timeout - elapsed;

  return OK;
}

/****************************************************************************
 * Name: lpc17_40_settimeout
 *
 * Description:
 *   Set a new timeout value.
 *
 * Input Parameters:
 *   lower   - A pointer to the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc17_40_settimeout(struct watchdog_lowerhalf_s *lower,
                               uint32_t timeout)
{
  struct lpc17_40_lowerhalf_s *priv =
      (struct lpc17_40_lowerhalf_s *)lower;
  uint64_t wdt_clk;
  uint64_t wdtc;

  priv->timeout = timeout;

#ifdef LPC176x
  /* The internal RC oscillator will be used for the WDT. It is clocked at
   * 4MHz. This is further divided by 4, by the WDT fixed prescaller.
   */

  wdt_clk = 4000000 / 4;
  putreg32(WDT_CLKSEL_WDSEL_INTRC, LPC17_40_WDT_CLKSEL);
#else
  /* WDT has a dedicated clock, set at 500kHz. This is further divided by 4,
   * by the WDT fixed prescaller.
   */

  wdt_clk = 500000 / 4;
#endif

  wdtc = ((uint64_t)timeout * wdt_clk) / 1000;
  putreg32((uint32_t)wdtc, LPC17_40_WDT_TC);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_wdtinitialize
 *
 * Description:
 *   Initialize the watchdog timer. The watchdog timer is initialized and
 *   registers as 'devpath.  The initial state of the watchdog timer is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_40_wdtinitialize(const char *devpath)
{
  struct lpc17_40_lowerhalf_s *priv = &g_wdgdev;

  /* Initialize the driver state structure. */

  priv->ops       = &g_wdgops;
  priv->timeout   = 0;
  priv->lastreset = 0;
  priv->started   = false;

  /* Select an arbitrary initial timeout value. But don't start the watchdog
   * yet.
   */

  lpc17_40_settimeout((struct watchdog_lowerhalf_s *)priv,
                      LPC17_40_WDT_DEFTIMEOUT);

  /* Register the watchdog driver as devpath. */

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_LPC17_40_WDT */

