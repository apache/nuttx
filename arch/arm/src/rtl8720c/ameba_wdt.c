/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_wdt.c
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
#include <nuttx/timers/watchdog.h>
#include <stdint.h>
#include <sys/types.h>
#include "hal_wdt.h"
#if defined(CONFIG_WATCHDOG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/**
 * This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct ameba_lowerhalf_s
{
  const struct watchdog_ops_s *ops; /* Lower half operations */
  uint32_t timeout;                 /* The (actual) selected timeout */
  uint32_t lastreset;               /* The last reset time */
  bool     started;                 /* true: The watchdog timer has
                                     * been started
                                     */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods */

static int      ameba_start(struct watchdog_lowerhalf_s *lower);
static int      ameba_stop(struct watchdog_lowerhalf_s *lower);
static int      ameba_keepalive(struct watchdog_lowerhalf_s *lower);
static int      ameba_getstatus(struct watchdog_lowerhalf_s *lower,
                                struct watchdog_status_s *status);
static int      ameba_settimeout(struct watchdog_lowerhalf_s *lower,
                                 uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = ameba_start,
  .stop       = ameba_stop,
  .keepalive  = ameba_keepalive,
  .getstatus  = ameba_getstatus,
  .settimeout = ameba_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct ameba_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *  lower - A pointer the publicly visible representation
 *  of the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ameba_start(struct watchdog_lowerhalf_s *lower)
{
  struct ameba_lowerhalf_s *priv = (struct ameba_lowerhalf_s *)lower;
  irqstate_t flags;
  flags = enter_critical_section();
  priv->started = true;
  priv->lastreset = clock_systime_ticks();
  hal_misc_wdt_enable();
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ameba_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ameba_stop(struct watchdog_lowerhalf_s *lower)
{
  struct ameba_lowerhalf_s *priv = (struct ameba_lowerhalf_s *)lower;
  irqstate_t flags;
  flags = enter_critical_section();
  hal_misc_wdt_disable();
  priv->started = false;
  priv->timeout = 0;
  priv->lastreset = 0;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ameba_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 *   The application program must write in the SAM_WDT_CLEAR register
 *   at regular intervals during normal operation to prevent an MCU reset.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation
 *           of the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ameba_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct ameba_lowerhalf_s *priv = (struct ameba_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Reload the WDT timer */

  flags = enter_critical_section();
  priv->lastreset = clock_systime_ticks();
  hal_misc_wdt_refresh();
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ameba_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of
 *            the "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ameba_getstatus(struct watchdog_lowerhalf_s *lower,
                           struct watchdog_status_s *status)
{
  struct ameba_lowerhalf_s *priv = (struct ameba_lowerhalf_s *)lower;
  uint32_t elapsed;
  uint32_t ticks;

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks   = clock_systime_ticks() - priv->lastreset;
  elapsed = (int32_t)TICK2MSEC(ticks);
  if (elapsed > priv->timeout)
    {
      elapsed = priv->timeout;
    }

  /* Return the approximate time until the watchdog timer expiration */

  status->timeleft = priv->timeout - elapsed;
  return OK;
}

/****************************************************************************
 * Name: ameba_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of
 *             the "lower-half" driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ameba_settimeout(struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct ameba_lowerhalf_s *priv = (struct ameba_lowerhalf_s *)lower;
  irqstate_t flags;
  flags = enter_critical_section();
  priv->timeout = timeout;
  hal_misc_wdt_init(timeout * 1000);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_wdt_initialize
 *
 * Description:
 *   Initialize the WDT watchdog timer.  The watchdog timer
 *   is initialized and registers as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ameba_wdt_initialize(void)
{
  struct ameba_lowerhalf_s *priv = &g_wdgdev;

  /* Initialize the driver state structure. */

  priv->ops = &g_wdgops;
  watchdog_register(CONFIG_WATCHDOG_DEVPATH,
                    (struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG */
