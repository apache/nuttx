/****************************************************************************
 * arch/sparc/src/bm3803/bm3803_wdg.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "sparc_internal.h"
#include "bm3803_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_BM3803_WDG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_BM3803_WDG_DEFTIMOUT
#  define CONFIG_BM3803_WDG_DEFTIMOUT 1000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct bm3803_lowerhalf_s
{
  /* Lower half operations */

  const struct watchdog_ops_s  *ops;
  uint32_t timeout;    /* The (actual) selected timeout */
  uint32_t lastreset;  /* The last reset time */
  bool     started;    /* true: The watchdog timer has been started */
  uint16_t  prescaler; /* Clock prescaler value */
  uint32_t reload;     /* Timer reload value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

# define        bm3803_getreg(addr)     getreg32(addr)
# define        bm3803_putreg(val,addr) putreg32(val,addr)

static inline void bm3803_setreload(struct bm3803_lowerhalf_s *priv);

/* "Lower half" driver methods **********************************************/

static int      bm3803_start(struct watchdog_lowerhalf_s *lower);
static int      bm3803_stop(struct watchdog_lowerhalf_s *lower);
static int      bm3803_keepalive(struct watchdog_lowerhalf_s *lower);
static int      bm3803_getstatus(struct watchdog_lowerhalf_s *lower,
                  struct watchdog_status_s *status);
static int      bm3803_settimeout(struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = bm3803_start,
  .stop       = bm3803_stop,
  .keepalive  = bm3803_keepalive,
  .getstatus  = bm3803_getstatus,
  .settimeout = bm3803_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct bm3803_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_setreload
 *
 * Description:
 *   Set up the prescaler and reload values.
 *
 * Input Parameters:
 *   priv   - A pointer the internal representation of the "lower-half"
 *             driver state structure.
 *
 ****************************************************************************/

static inline void bm3803_setreload(struct bm3803_lowerhalf_s *priv)
{
  /* Set the reload value */

  bm3803_putreg(priv->reload, BM3803_TIM1_BASE + BM3803_TIM_WDG_OFFSET);
}

/****************************************************************************
 * Name: bm3803_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_start(struct watchdog_lowerhalf_s *lower)
{
  struct bm3803_lowerhalf_s *priv =
                                      (struct bm3803_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t val = bm3803_getreg(BM3803_TIM1_BASE + BM3803_TIM_CR_OFFSET);
  val |= TIMER_WDG;

  wdinfo("Entry: started\n");
  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      /* Set up prescaler and reload value for the selected timeout before
       * starting the watchdog timer.
       */

      bm3803_setreload(priv);

      /* Enable WDG (the LSI oscillator will be enabled by hardware).  NOTE:
       * If the "Hardware watchdog" feature is enabled through the device
       * option bits, the watchdog is automatically enabled at power-on.
       */

      flags           = enter_critical_section();
      bm3803_putreg(val, BM3803_TIM1_BASE + BM3803_TIM_CR_OFFSET);
      priv->lastreset = clock_systime_ticks();
      priv->started   = true;
      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: bm3803_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower- A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_stop(struct watchdog_lowerhalf_s *lower)
{
  uint32_t val = bm3803_getreg(BM3803_TIM1_BASE + BM3803_TIM_CR_OFFSET);

  wdinfo("Entry\n");

  val &= ~TIMER_WDG;

  bm3803_putreg(val, BM3803_TIM1_BASE + BM3803_TIM_CR_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: bm3803_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower- A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct bm3803_lowerhalf_s *priv =
                                      (struct bm3803_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");

  /* Reload the WDG timer */

  flags = enter_critical_section();
  bm3803_setreload(priv);
  priv->lastreset = clock_systime_ticks();
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: bm3803_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *putreg32(value, ((struct bm3803_tim_priv_s *)dev)->base + offset);
 * Input Parameters:
 *   lower- A pointer the publicly visible representation of the "lower-half"
 *            driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_getstatus(struct watchdog_lowerhalf_s *lower,
                             struct watchdog_status_s *status)
{
  struct bm3803_lowerhalf_s *priv =
                                      (struct bm3803_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

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

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", status->flags);
  wdinfo("  timeout  : %d\n", status->timeout);
  wdinfo("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: bm3803_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower- A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_settimeout(struct watchdog_lowerhalf_s *lower,
                              uint32_t timeout)
{
  struct bm3803_lowerhalf_s *priv =
                                      (struct bm3803_lowerhalf_s *)lower;
  uint32_t reload;
  uint16_t prescaler;
  uint32_t freqin;
  uint32_t clock;

  wdinfo("Entry: timeout=%d\n", timeout);
  DEBUGASSERT(priv);

  /* Can this timeout be represented? */

  if (timeout < 1)
    {
      wderr("ERROR: Cannot represent timeout=%d \n", timeout);
      return -ERANGE;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  freqin = BOARD_TIM1_FREQUENCY;

  prescaler = 0x3ff & bm3803_getreg(BM3803_TIM12PRE_BASE +
              BM3803_TIM_PSCLOAD_OFFSET);

  clock = freqin / (prescaler + 1);

  reload = (timeout / 1000) * clock;

  priv->timeout = timeout;

  /* Save setup values for later use */

  priv->prescaler = prescaler;
  priv->reload    = reload;

  /* Write the prescaler and reload values to the WDG registers. */

  if (priv->started)
    {
      bm3803_setreload(priv);
    }

  wdinfo("reload=%d\n", reload);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_wdginitialize
 *
 * Description:
 *   Initialize the WDG watchdog timer.The watchdog timer is initialized and
 *   registers as 'devpath'.  The initial state of the watchdog timer is
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

void bm3803_wdginitialize(const char *devpath)
{
  struct bm3803_lowerhalf_s *priv = &g_wdgdev;

  wdinfo("Entry: devpath=%s \n", devpath);

  /* NOTE we assume that clocking to the WDG has already been provided by
   * the RCC initialization logic.
   */

  /* Initialize the driver state structure. */

  priv->ops     = &g_wdgops;
  priv->started = false;

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  bm3803_settimeout((struct watchdog_lowerhalf_s *)priv,
                    CONFIG_BM3803_WDG_DEFTIMOUT);

  /* Register the watchdog driver as /dev/watchdog0 */

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_BM3803_WDG */
