/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_fwdgt.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>

#include "riscv_internal.h"
#include "hardware/gd32vw55x_fwdgt.h"
#include "hardware/gd32vw55x_rcu.h"
#include "gd32vw55x_fwdgt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_GD32VW55X_FWDGT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The FWDGT is clocked by the IRC32K internal RC oscillator.  The nominal,
 * uncalibrated frequency is 32 KHz.
 */

#ifndef GD32VW55X_IRC32K_FREQUENCY
#  define GD32VW55X_IRC32K_FREQUENCY 32000
#endif

/* The minimum frequency of the FWDGT clock is:
 *
 *   Fmin = Firc32k / 256
 *
 * So the maximum delay (in milliseconds) is then:
 *
 *   1000 * FWDGT_RLD_MAX / Fmin
 *
 * For Firc32k = 32 KHz:
 *
 *   Fmin = 125
 *   1000 * 4095 / 125 = 32760 msec
 */

#define FWDGT_FMIN       (GD32VW55X_IRC32K_FREQUENCY / 256)
#define FWDGT_MAXTIMEOUT (1000 * FWDGT_RLD_MAX / FWDGT_FMIN)

/* Configuration ************************************************************/

#ifndef CONFIG_GD32VW55X_FWDGT_DEFTIMOUT
#  define CONFIG_GD32VW55X_FWDGT_DEFTIMOUT FWDGT_MAXTIMEOUT
#endif

/* The prescaler and reload registers can only be written while the
 * corresponding update flags (PUD and RUD) are clear.  Bound the number of
 * poll iterations so that a broken IRC32K cannot hang the system.
 */

#define FWDGT_UPDATE_TIMEOUT 0x000fffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct gd32_lowerhalf_s
{
  const struct watchdog_ops_s *ops; /* Lower half operations */
  uint32_t ircfreq;                 /* Calibrated IRC32K frequency */
  uint32_t timeout;                 /* The (actual) selected timeout */
  uint32_t lastreset;               /* The last reset time */
  bool     started;                 /* true: The watchdog has been started */
  uint8_t  prescaler;               /* Clock prescaler selection */
  uint16_t reload;                  /* Timer reload value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int      gd32_start(struct watchdog_lowerhalf_s *lower);
static int      gd32_stop(struct watchdog_lowerhalf_s *lower);
static int      gd32_keepalive(struct watchdog_lowerhalf_s *lower);
static int      gd32_getstatus(struct watchdog_lowerhalf_s *lower,
                               struct watchdog_status_s *status);
static int      gd32_settimeout(struct watchdog_lowerhalf_s *lower,
                                uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = gd32_start,
  .stop       = gd32_stop,
  .keepalive  = gd32_keepalive,
  .getstatus  = gd32_getstatus,
  .settimeout = gd32_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct gd32_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_setprescaler
 *
 * Description:
 *   Write the prescaler and reload values to the FWDGT.  Write access to
 *   these registers must be explicitly enabled and each write is only
 *   accepted while the matching update flag is clear.
 *
 * Input Parameters:
 *   priv - A pointer the internal representation of the "lower-half" driver
 *          state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_setprescaler(struct gd32_lowerhalf_s *priv)
{
  uint32_t timeout;

  /* Enable write access to the FWDGT_PSC and FWDGT_RLD registers */

  putreg32(FWDGT_CTL_KEY_WRITEEN, GD32VW55X_FWDGT_CTL);

  /* Wait for the PUD flag to be cleared by hardware */

  timeout = FWDGT_UPDATE_TIMEOUT;
  while ((getreg32(GD32VW55X_FWDGT_STAT) & FWDGT_STAT_PUD) != 0)
    {
      if (--timeout == 0)
        {
          wderr("ERROR: PSC update did not complete\n");
          return -ETIMEDOUT;
        }
    }

  /* Set the prescaler */

  putreg32((uint32_t)priv->prescaler, GD32VW55X_FWDGT_PSC);

  /* Wait for the RUD flag to be cleared by hardware */

  timeout = FWDGT_UPDATE_TIMEOUT;
  while ((getreg32(GD32VW55X_FWDGT_STAT) & FWDGT_STAT_RUD) != 0)
    {
      if (--timeout == 0)
        {
          wderr("ERROR: RLD update did not complete\n");
          return -ETIMEDOUT;
        }
    }

  /* Set the reload value */

  putreg32((uint32_t)priv->reload & FWDGT_RLD_MASK, GD32VW55X_FWDGT_RLD);

  /* Reload the counter (this also disables the write access again) */

  putreg32(FWDGT_CTL_KEY_RELOAD, GD32VW55X_FWDGT_CTL);
  return OK;
}

/****************************************************************************
 * Name: gd32_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_start(struct watchdog_lowerhalf_s *lower)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  wdinfo("Entry: started=%d\n", priv->started);
  DEBUGASSERT(priv != NULL);

  /* Have we already been started? */

  if (priv->started)
    {
      return OK;
    }

  /* Set up the prescaler and the reload value for the selected timeout
   * before starting the watchdog timer.
   */

  ret = gd32_setprescaler(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the FWDGT.  The IRC32K oscillator is started by hardware if it
   * is not already running.  Once started, the FWDGT can only be stopped by
   * a reset.
   */

  flags           = enter_critical_section();
  putreg32(FWDGT_CTL_KEY_ENABLE, GD32VW55X_FWDGT_CTL);
  priv->lastreset = clock_systime_ticks();
  priv->started   = true;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: gd32_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_stop(struct watchdog_lowerhalf_s *lower)
{
  /* There is no way to disable the FWDGT once it has been started */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: gd32_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, preventing any
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

static int gd32_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");
  DEBUGASSERT(priv != NULL);

  /* Reload the FWDGT counter */

  flags = enter_critical_section();
  putreg32(FWDGT_CTL_KEY_RELOAD, GD32VW55X_FWDGT_CTL);
  priv->lastreset = clock_systime_ticks();
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: gd32_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the
 *            "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_getstatus(struct watchdog_lowerhalf_s *lower,
                          struct watchdog_status_s *status)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  uint32_t elapsed;
  uint32_t ticks;

  wdinfo("Entry\n");
  DEBUGASSERT(priv != NULL && status != NULL);

  /* Return the status flags */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks   = clock_systime_ticks() - priv->lastreset;
  elapsed = (uint32_t)TICK2MSEC(ticks);

  if (elapsed > priv->timeout)
    {
      elapsed = priv->timeout;
    }

  /* Return the approximate time until the watchdog timer expiration */

  status->timeleft = priv->timeout - elapsed;

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08" PRIx32 "\n", status->flags);
  wdinfo("  timeout  : %" PRId32 "\n", status->timeout);
  wdinfo("  timeleft : %" PRId32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: gd32_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_settimeout(struct watchdog_lowerhalf_s *lower,
                           uint32_t timeout)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  uint32_t fwdgt;
  uint64_t reload;
  int prescaler;
  int shift;

  wdinfo("Entry: timeout=%" PRId32 "\n", timeout);
  DEBUGASSERT(priv != NULL);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > FWDGT_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%" PRId32 " > %d\n",
            timeout, FWDGT_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Select the smallest prescaler that will result in a reload value that
   * is less than the maximum.
   */

  for (prescaler = 0; ; prescaler++)
    {
      /* PSC = 0 -> Divider = 4   = 1 << 2
       * PSC = 1 -> Divider = 8   = 1 << 3
       * ...
       * PSC = 6 -> Divider = 256 = 1 << 8
       * PSC = n -> Divider       = 1 << (n + 2)
       */

      shift = prescaler + 2;

      /* Get the FWDGT counter frequency in Hz.  For a nominal 32 KHz
       * IRC32K clock this is a value in the range of 8000 to 125.
       */

      fwdgt = priv->ircfreq >> shift;

      /* We want:
       *   1000 * reload / Ffwdgt = timeout
       * Or:
       *   reload = Ffwdgt * timeout / 1000
       */

      reload = (uint64_t)fwdgt * (uint64_t)timeout / 1000;

      /* If this reload value is less than the maximum, or if we have run
       * out of prescaler values, then use these settings.
       */

      if (reload <= FWDGT_RLD_MAX || prescaler == FWDGT_PSC_MAX)
        {
          break;
        }
    }

  /* Make sure that the final reload value is within range */

  if (reload > FWDGT_RLD_MAX)
    {
      reload = FWDGT_RLD_MAX;
    }

  /* Get the actual timeout value in milliseconds:
   *
   *   timeout = 1000 * reload / Ffwdgt
   */

  priv->timeout   = (1000 * (uint32_t)reload) / fwdgt;

  /* Save the setup values for later use */

  priv->prescaler = (uint8_t)prescaler;
  priv->reload    = (uint16_t)reload;

  wdinfo("prescaler=%d fwdgt=%" PRId32 " reload=%" PRId32 "\n",
         prescaler, fwdgt, (uint32_t)reload);

  /* If the watchdog is already running, the new values must be written to
   * the hardware immediately.  Otherwise they are written when the watchdog
   * is started.
   */

  if (priv->started)
    {
      return gd32_setprescaler(priv);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fwdgt_initialize
 *
 * Description:
 *   Initialize the FWDGT watchdog timer.  The watchdog timer is initialized
 *   and registered as 'devpath'.  The initial state of the watchdog timer
 *   is disabled.
 *
 * Input Parameters:
 *   devpath    - The full path to the watchdog.  This should be of the form
 *                /dev/watchdog0
 *   irc32kfreq - The calibrated IRC32K clock frequency in Hz
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_fwdgt_initialize(const char *devpath, uint32_t irc32kfreq)
{
  struct gd32_lowerhalf_s *priv = &g_wdgdev;
  uint32_t regval;

  wdinfo("Entry: devpath=%s irc32kfreq=%" PRId32 "\n", devpath, irc32kfreq);
  DEBUGASSERT(irc32kfreq > 0);

  /* Initialize the driver state structure */

  priv->ops     = &g_wdgops;
  priv->ircfreq = irc32kfreq;
  priv->started = false;

  /* Make sure that the IRC32K oscillator is enabled.  NOTE:  The oscillator
   * is enabled here but it is never disabled by this file because this file
   * has no knowledge of the global usage of the oscillator.
   */

  regval  = getreg32(GD32VW55X_RCU_RSTSCK);
  regval |= RCU_RSTSCK_IRC32KEN;
  putreg32(regval, GD32VW55X_RCU_RSTSCK);

  while ((getreg32(GD32VW55X_RCU_RSTSCK) & RCU_RSTSCK_IRC32KSTB) == 0)
    {
    }

  /* Select an arbitrary initial timeout value.  But do not start the
   * watchdog yet.  NOTE:  If the "hardware watchdog" feature is enabled via
   * the FMC option bytes (NWDG_HW), the FWDGT is automatically started at
   * power-on and this driver can only feed it.
   */

  gd32_settimeout((struct watchdog_lowerhalf_s *)priv,
                  CONFIG_GD32VW55X_FWDGT_DEFTIMOUT);

  /* Register the watchdog driver as, e.g., /dev/watchdog0 */

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_GD32VW55X_FWDGT */
