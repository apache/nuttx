/****************************************************************************
 * arch/arm/src/ameba/ameba_wdt.c
 *
 *   Copyright (C) 2020 Xiaomi Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
  uint32_t timeout;                       /* The (actual) selected timeout */
  uint32_t lastreset;                     /* The last reset time */
  bool     started;                       /* true: The watchdog timer has
                                           * been started
                                           */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int      ameba_start(FAR struct watchdog_lowerhalf_s *lower);
static int      ameba_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      ameba_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      ameba_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                  FAR struct watchdog_status_s *status);
static int      ameba_settimeout(FAR struct watchdog_lowerhalf_s *lower,
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

static int ameba_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ameba_lowerhalf_s *priv = (FAR struct ameba_lowerhalf_s *)lower;
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

static int ameba_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ameba_lowerhalf_s *priv = (FAR struct ameba_lowerhalf_s *)lower;
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

static int ameba_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ameba_lowerhalf_s *priv = (FAR struct ameba_lowerhalf_s *)lower;
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

static int ameba_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status)
{
  FAR struct ameba_lowerhalf_s *priv = (FAR struct ameba_lowerhalf_s *)lower;
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

static int ameba_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct ameba_lowerhalf_s *priv = (FAR struct ameba_lowerhalf_s *)lower;
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
  FAR struct ameba_lowerhalf_s *priv = &g_wdgdev;

  /* Initialize the driver state structure. */

  priv->ops = &g_wdgops;

  (void)watchdog_register(CONFIG_WATCHDOG_DEVPATH,
                          (FAR struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG */
