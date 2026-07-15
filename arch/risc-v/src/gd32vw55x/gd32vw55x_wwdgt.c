/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_wwdgt.c
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
#include <nuttx/timers/watchdog.h>

#include "riscv_internal.h"
#include "hardware/gd32vw55x_wwdgt.h"
#include "hardware/gd32vw55x_rcu.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_wwdgt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_GD32VW55X_WWDGT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The WWDGT is clocked from PCLK1 (APB1).  The counter clock is:
 *
 *   Fwwdgt = PCLK1 / 4096 / prescaler
 *
 * The minimum frequency of the WWDGT clock is therefore:
 *
 *   Fmin = PCLK1 / 4096 / 8
 *
 * and the maximum delay (in milliseconds) is:
 *
 *   1000 * (WWDGT_CTL_CNT_MAX + 1) / Fmin
 *
 * For PCLK1 = 80 MHz:
 *
 *   Fmin = 2441.4
 *   1000 * 64 / Fmin = 26.2 msec
 */

#define WWDGT_FMIN       (GD32VW55X_PCLK1_FREQ / 4096 / 8)
#define WWDGT_MAXTIMEOUT (1000 * (WWDGT_CTL_CNT_MAX + 1) / WWDGT_FMIN)

/* Configuration ************************************************************/

#ifndef CONFIG_GD32VW55X_WWDGT_DEFTIMOUT
#  define CONFIG_GD32VW55X_WWDGT_DEFTIMOUT WWDGT_MAXTIMEOUT
#endif

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
  xcpt_t   handler;                 /* Current EWI interrupt handler */
  uint32_t timeout;                 /* The actual timeout value */
  uint32_t fwwdgt;                  /* WWDGT counter clock frequency */
  bool     started;                 /* The timer has been started */
  uint8_t  reload;                  /* The 7-bit counter reset value */
  uint8_t  window;                  /* The 7-bit window value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void     gd32_setwindow(struct gd32_lowerhalf_s *priv,
                               uint8_t window);

/* Interrupt handling *******************************************************/

static int      gd32_interrupt(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int      gd32_start(struct watchdog_lowerhalf_s *lower);
static int      gd32_stop(struct watchdog_lowerhalf_s *lower);
static int      gd32_keepalive(struct watchdog_lowerhalf_s *lower);
static int      gd32_getstatus(struct watchdog_lowerhalf_s *lower,
                               struct watchdog_status_s *status);
static int      gd32_settimeout(struct watchdog_lowerhalf_s *lower,
                                uint32_t timeout);
static xcpt_t   gd32_capture(struct watchdog_lowerhalf_s *lower,
                             xcpt_t handler);
static int      gd32_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                           unsigned long arg);

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
  .capture    = gd32_capture,
  .ioctl      = gd32_ioctl,
};

/* "Lower half" driver state */

static struct gd32_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_setwindow
 *
 * Description:
 *   Set the WWDGT_CFG window value.  The window value is compared to the
 *   down counter when the counter is updated.  The counter may only be
 *   updated when its value is below this window value (and greater than
 *   0x40), otherwise a reset is generated.
 *
 * Input Parameters:
 *   priv   - A pointer the internal representation of the "lower-half"
 *            driver state structure.
 *   window - The new 7-bit window value.
 *
 ****************************************************************************/

static void gd32_setwindow(struct gd32_lowerhalf_s *priv, uint8_t window)
{
  uint32_t regval;

  /* Set the WIN[6:0] bits according to the selected window value */

  regval  = getreg32(GD32VW55X_WWDGT_CFG);
  regval &= ~WWDGT_CFG_WIN_MASK;
  regval |= ((uint32_t)window << WWDGT_CFG_WIN_SHIFT) & WWDGT_CFG_WIN_MASK;
  putreg32(regval, GD32VW55X_WWDGT_CFG);

  /* Remember the window setting */

  priv->window = window;
}

/****************************************************************************
 * Name: gd32_interrupt
 *
 * Description:
 *   WWDGT early wakeup interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int gd32_interrupt(int irq, void *context, void *arg)
{
  struct gd32_lowerhalf_s *priv = &g_wdgdev;
  uint32_t regval;

  /* Check if the EWI interrupt is really pending */

  regval = getreg32(GD32VW55X_WWDGT_STAT);
  if ((regval & WWDGT_STAT_EWIF) != 0)
    {
      /* Is there a registered handler? */

      if (priv->handler != NULL)
        {
          /* Yes... NOTE:  This interrupt service routine (ISR) must reload
           * the WWDGT counter to prevent the reset.  Otherwise, we will
           * reset upon return.
           */

          priv->handler(irq, context, arg);
        }

      /* The EWI interrupt is cleared by writing zero to the EWIF bit in the
       * WWDGT_STAT register.
       */

      putreg32(0, GD32VW55X_WWDGT_STAT);
    }

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

  wdinfo("Entry\n");
  DEBUGASSERT(priv != NULL);

  /* The watchdog is always disabled after a reset.  It is enabled by
   * setting the WDGTEN bit in the WWDGT_CTL register and then it can no
   * longer be disabled except by a reset.
   */

  putreg32(WWDGT_CTL_WDGTEN | WWDGT_CTL_CNT_RESET | priv->reload,
           GD32VW55X_WWDGT_CTL);
  priv->started = true;
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
  /* The watchdog cannot be disabled once it has been enabled */

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
 *   The application must write to the WWDGT_CTL register at regular
 *   intervals during normal operation to prevent an MCU reset.  This
 *   operation must only occur when the counter value is lower than the
 *   window value.
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

  wdinfo("Entry\n");
  DEBUGASSERT(priv != NULL);

  /* Write to the CNT[6:0] bits to configure the counter value.  There is no
   * need for a read-modify-write; writing a zero to the WDGTEN bit does
   * nothing.
   */

  putreg32(WWDGT_CTL_CNT_RESET | priv->reload, GD32VW55X_WWDGT_CTL);
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
  uint32_t counter;

  wdinfo("Entry\n");
  DEBUGASSERT(priv != NULL && status != NULL);

  /* Return the status flags */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  if (priv->handler != NULL)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds) */

  counter = (getreg32(GD32VW55X_WWDGT_CTL) & WWDGT_CTL_CNT_MASK) >>
            WWDGT_CTL_CNT_SHIFT;
  elapsed = priv->reload - (counter & WWDGT_CTL_CNT_MAX);
  status->timeleft = (priv->timeout * elapsed) / (priv->reload + 1);

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
  uint32_t fwwdgt;
  uint32_t reload;
  uint32_t regval;
  int psc;

  DEBUGASSERT(priv != NULL);
  wdinfo("Entry: timeout=%" PRId32 "\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > WWDGT_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%" PRId32 " > %d\n",
            timeout, WWDGT_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Select the smallest prescaler that will result in a reload value that
   * is less than the maximum.
   */

  for (psc = 0; ; psc++)
    {
      /* PSC = 0 -> Divider = 1 = 1 << 0
       * PSC = 1 -> Divider = 2 = 1 << 1
       * PSC = 2 -> Divider = 4 = 1 << 2
       * PSC = 3 -> Divider = 8 = 1 << 3
       */

      fwwdgt = (GD32VW55X_PCLK1_FREQ / 4096) >> psc;

      /* The timeout is given by:
       *
       *   timeout = 1000 * (reload + 1) / Fwwdgt, OR
       *   reload  = timeout * Fwwdgt / 1000 - 1
       */

      reload = timeout * fwwdgt / 1000 - 1;

      /* If this reload value is less than the maximum, or if we have run
       * out of prescaler values, then use these settings.
       */

      if (reload <= WWDGT_CTL_CNT_MAX || psc == 3)
        {
          break;
        }
    }

  /* Make sure that the final reload value is within range */

  if (reload > WWDGT_CTL_CNT_MAX)
    {
      reload = WWDGT_CTL_CNT_MAX;
    }

  /* Calculate and save the actual timeout value in milliseconds:
   *
   *   timeout = 1000 * (reload + 1) / Fwwdgt
   */

  priv->timeout = 1000 * (reload + 1) / fwwdgt;

  /* Remember the selected values */

  priv->fwwdgt = fwwdgt;
  priv->reload = (uint8_t)reload;

  wdinfo("psc=%d fwwdgt=%" PRId32 " reload=%" PRId32 "\n",
         psc, fwwdgt, reload);
  wdinfo("timeout=%" PRId32 "\n", priv->timeout);

  /* Set the PSC[1:0] bits according to the calculated value */

  regval  = getreg32(GD32VW55X_WWDGT_CFG);
  regval &= ~WWDGT_CFG_PSC_MASK;
  regval |= ((uint32_t)psc << WWDGT_CFG_PSC_SHIFT) & WWDGT_CFG_PSC_MASK;
  putreg32(regval, GD32VW55X_WWDGT_CFG);

  /* Reset the 7-bit window value to the maximum value, essentially
   * disabling the lower limit of the watchdog reset time.
   */

  gd32_setwindow(priv, 0x7f);
  return OK;
}

/****************************************************************************
 * Name: gd32_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provided
 *   timeout handler.  NOTE:  Providing handler == NULL will restore the
 *   reset behavior.
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   handler - The new watchdog expiration function pointer.  If this
 *             function pointer is NULL, then the reset-on-expiration
 *             behavior is restored.
 *
 * Returned Value:
 *   The previous watchdog expiration function pointer or NULL if there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static xcpt_t gd32_capture(struct watchdog_lowerhalf_s *lower,
                           xcpt_t handler)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);
  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler to return */

  flags      = enter_critical_section();
  oldhandler = priv->handler;

  /* Save the new handler */

  priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  regval = getreg32(GD32VW55X_WWDGT_CFG);
  if (handler != NULL)
    {
      /* Attaching... Enable the EWI interrupt */

      regval |= WWDGT_CFG_EWIE;
      putreg32(regval, GD32VW55X_WWDGT_CFG);

      up_enable_irq(GD32VW55X_IRQ_WWDGT);
    }
  else
    {
      /* Detaching... Disable the EWI interrupt */

      regval &= ~WWDGT_CFG_EWIE;
      putreg32(regval, GD32VW55X_WWDGT_CFG);

      up_disable_irq(GD32VW55X_IRQ_WWDGT);
    }

  leave_critical_section(flags);
  return oldhandler;
}

/****************************************************************************
 * Name: gd32_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                      unsigned long arg)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv != NULL);
  wdinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);

  /* WDIOC_MINTIME: Set the minimum ping time.  If two keepalive ioctls are
   * received within this time, a reset event will be generated.
   * Argument: A 32-bit time value in milliseconds.
   */

  if (cmd == WDIOC_MINTIME)
    {
      uint32_t mintime = (uint32_t)arg;

      /* The minimum time must be strictly less than the total delay */

      ret = -EINVAL;
      if (mintime < priv->timeout)
        {
          uint32_t window = (priv->timeout - mintime) * priv->fwwdgt /
                            1000 - 1;
          DEBUGASSERT(window < priv->reload);
          gd32_setwindow(priv, (uint8_t)(window | WWDGT_CTL_CNT_RESET));
          ret = OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_wwdgt_initialize
 *
 * Description:
 *   Initialize the WWDGT watchdog timer.  The watchdog timer is initialized
 *   and registered as 'devpath'.  The initial state of the watchdog timer
 *   is disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *             /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_wwdgt_initialize(const char *devpath)
{
  struct gd32_lowerhalf_s *priv = &g_wdgdev;
  uint32_t regval;

  wdinfo("Entry: devpath=%s\n", devpath);

  /* Enable the WWDGT peripheral clock (PCLK1) */

  regval  = getreg32(GD32VW55X_RCU_APB1EN);
  regval |= RCU_APB1EN_WWDGTEN;
  putreg32(regval, GD32VW55X_RCU_APB1EN);

  /* Initialize the driver state structure */

  priv->ops     = &g_wdgops;
  priv->handler = NULL;
  priv->started = false;

  /* Attach the EWI interrupt handler, but do not enable it yet */

  irq_attach(GD32VW55X_IRQ_WWDGT, gd32_interrupt, NULL);

  /* Select an arbitrary initial timeout value.  But do not start the
   * watchdog yet.
   */

  gd32_settimeout((struct watchdog_lowerhalf_s *)priv,
                  CONFIG_GD32VW55X_WWDGT_DEFTIMOUT);

  /* Register the watchdog driver as, e.g., /dev/watchdog0 */

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_GD32VW55X_WWDGT */
