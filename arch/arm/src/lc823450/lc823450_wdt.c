/****************************************************************************
 * arch/arm/src/lc823450/lc823450_wdt.c
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

#include <inttypes.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/timers/watchdog.h>

#ifdef CONFIG_SCHED_HPWORK
#  include <nuttx/wqueue.h>
#endif

#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "lc823450_syscontrol.h"
#include "lc823450_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_LC823450_WDT)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define CLKSEL_PTM 11

#define WT0SRCCLK 3
#define WT0BCGST 0

#define WDT_BASE             0x40047000  /* Plain Timer 0 */
#define WDT_PT0CTL           (WDT_BASE + 0x00)
#define WDT_PT0CTL_WT0SRCCK  0
#define WDT_PT0CTL_WT0MOD    4
#define WDT_PT0CTL_WT0ACT    8

#define WDT_PT0STS           (WDT_BASE + 0x04)
#define WDT_PT0STS_CSTS      0
#define WDT_WT0BCG           (WDT_BASE + 0x08)
#define WDT_WT0PST           (WDT_BASE + 0x0c)
#define WDT_WT0RSTS          (WDT_BASE + 0x10)
#define WDT_WT0RSTS_RSTS     0
#define WDT_BT0PST           (WDT_BASE + 0x14)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct lc823450_wdt_lowerhalf_s
{
  struct watchdog_lowerhalf_s wdt_lh;
#ifdef CONFIG_LC823450_WDT_INTERRUPT
  xcpt_t   handler;  /* Current WDT interrupt handler */
#endif
  uint32_t timeout;  /* The actual timeout value (milliseconds) */
  uint16_t reload;   /* The 16-bit watchdog reload value */
  bool     started;  /* The timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

/* Interrupt handling *******************************************************/

#ifdef CONFIG_LC823450_WDT_INTERRUPT
static int    lc823450_wdt_interrupt(int irq,
                                     void *context, void *arg);
#endif

/* "Lower half" driver methods **********************************************/

static int    lc823450_wdt_start(struct watchdog_lowerhalf_s *lower);
static int    lc823450_wdt_stop(struct watchdog_lowerhalf_s *lower);
static int    lc823450_wdt_keepalive(struct watchdog_lowerhalf_s *lower);
static int    lc823450_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                                     struct watchdog_status_s *status);
static int    lc823450_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                                      uint32_t timeout);
static xcpt_t lc823450_wdt_capture(struct watchdog_lowerhalf_s *lower,
                                   xcpt_t handler);
static int    lc823450_wdt_ioctl(struct watchdog_lowerhalf_s *lower,
                                 int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = lc823450_wdt_start,
  .stop       = lc823450_wdt_stop,
  .keepalive  = lc823450_wdt_keepalive,
  .getstatus  = lc823450_wdt_getstatus,
  .settimeout = lc823450_wdt_settimeout,
  .capture    = lc823450_wdt_capture,
  .ioctl      = lc823450_wdt_ioctl,
};

/* "Lower half" driver state */

static struct lc823450_wdt_lowerhalf_s g_wdtdev;

#ifdef CONFIG_WATCHDOG_WORK
static struct work_s    wdg_work;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_WATCHDOG_WORK
static void wdg_work_func(void *arg)
{
  lc823450_wdt_keepalive(&g_wdtdev.wdt_lh);

  work_queue(HPWORK, &wdg_work, wdg_work_func, NULL,
             MSEC2TICK(CONFIG_WATCHDOG_WORK_TIMEOUT / 2));
}

#endif

/****************************************************************************
 * Name: lc823450_wdt_interrupt
 *
 * Description:
 *   WDT early warning interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

#ifdef CONFIG_LC823450_WDT_INTERRUPT
static int lc823450_wdt_interrupt(int irq, void *context, void *arg)
{
  struct lc823450_wdt_lowerhalf_s *priv = &g_wdtdev;

  if (!(getreg32(WDT_PT0STS) & (1 << WDT_PT0STS_CSTS)))
    {
      DEBUGPANIC();
    }

  /* Is there a registered handler? */

  if (priv->handler)
    {
      /* Yes... NOTE:  This interrupt service routine (ISR) must reload
       * the WDT counter to prevent the reset.  Otherwise, we will reset
       * upon return.
       */

      priv->handler(irq, context);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: lc823450_wdt_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lc823450_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  modifyreg32(WDT_PT0CTL, 0, 1 << WDT_PT0CTL_WT0ACT);

  wdinfo("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: lc823450_wdt_stop
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

static int lc823450_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  modifyreg32(WDT_PT0CTL, 1 << WDT_PT0CTL_WT0ACT, 0);

  wdinfo("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: lc823450_wdt_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the atchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lc823450_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct lc823450_wdt_lowerhalf_s *priv =
    (struct lc823450_wdt_lowerhalf_s *)lower;

  wdinfo("Entry\n");

  putreg32(priv->reload, WDT_WT0PST);

  return OK;
}

/****************************************************************************
 * Name: lc823450_wdt_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of
 *             the "lower-half" driver state structure.
 *   stawtus - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lc823450_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                         struct watchdog_status_s *status)
{
  struct lc823450_wdt_lowerhalf_s *priv =
    (struct lc823450_wdt_lowerhalf_s *)lower;

  uint32_t wdt_freq;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

#ifdef CONFIG_LC823450_WDT_INTERRUPT
  if (priv->handler)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }
#endif

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds)
   *
   * REVISIT:  I think this that this information is available.
   */

  wdt_freq = XT1OSC_CLK / 8 /* 2 ^ WT0SRCCLK */;

  status->timeleft = (65536 - getreg32(WDT_WT0PST)) *
    (2 * (256 - WT0BCGST) * 1000) / wdt_freq;

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08" PRIx32 "\n", status->flags);
  wdinfo("  timeout  : %" PRId32 "\n", status->timeout);
  wdinfo("  timeleft : %" PRId32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: lc823450_wdt_settimeout
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

static int lc823450_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                                   uint32_t timeout)
{
  struct lc823450_wdt_lowerhalf_s *priv =
    (struct lc823450_wdt_lowerhalf_s *)lower;
  int32_t wt0pstst;
  uint32_t wdt_freq;

  DEBUGASSERT(priv);

  wdt_freq = XT1OSC_CLK / 8 /* 2 ^ WT0SRCCLK */;

  /* ProgrammersModel_PTM0v0.1.pdf:p24 */

  wt0pstst = 65536 - (uint64_t)timeout * wdt_freq /
    (2 * (256 - WT0BCGST) * 1000);

  if (wt0pstst < 1 || wt0pstst > 0xffff)
    {
      wdinfo("Error: timeout= %" PRId32 " < %" PRId32 " > %" PRId32 "\n",
            65536 - 1 * wdt_freq / (2 * (256 - WT0BCGST) * 1000),
            timeout,
            65536 - 0xffff * wdt_freq / (2 * (256 - WT0BCGST) * 1000)); /* 22s */
    }

  priv->reload = wt0pstst;
  putreg32(WT0BCGST, WDT_WT0BCG);

  putreg32((getreg32(WDT_PT0CTL) & ~0x3) | WT0SRCCLK, WDT_PT0CTL);

#ifdef CONFIG_LC823450_WDT_INTERRUPT
  /* interrupt mode */

  modifyreg32(WDT_PT0CTL, 1 << WDT_PT0CTL_WT0MOD, 0);
#else /* CONFIG_LC823450_WDT_INTERRUPT */
  /* reset mode */

  modifyreg32(WDT_PT0CTL, 0, 1 << WDT_PT0CTL_WT0MOD);
#endif /* CONFIG_LC823450_WDT_INTERRUPT */

  putreg32(wt0pstst, WDT_WT0PST);

  lc823450_wdt_start(lower);

  wdinfo("Entry: timeout=%" PRId32 "\n", timeout);

  return OK;
}

/****************************************************************************
 * Name: lc823450_wdt_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of
 *                the "lower-half" driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Value:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t lc823450_wdt_capture(struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
#ifndef CONFIG_LC823450_WDT_INTERRUPT
  wdinfo("ERROR: Not configured for this mode\n");
  return NULL;
#else
  struct lc823450_wdt_lowerhalf_s *priv =
    (struct lc823450_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  DEBUGASSERT(priv);
  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  flags = enter_critical_section();
  oldhandler = priv->handler;

  /* Save the new handler */

  priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  if (handler)
    {
      /* Attaching... Enable the WDT interrupt */

      up_enable_irq(SAM_IRQ_WDT);
    }
  else
    {
      /* Detaching... Disable the WDT interrupt */

      up_disable_irq(SAM_IRQ_WDT);
    }

  leave_critical_section(flags);
  return oldhandler;
#endif
}

/****************************************************************************
 * Name: lc823450_wdt_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *   cmd   - The ioctol command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lc823450_wdt_ioctl(struct watchdog_lowerhalf_s *lower,
                              int cmd, unsigned long arg)
{
  wdinfo("cmd=%d arg=%ld\n", cmd, arg);

  /* No ioctls are supported */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_wdginitialize
 *
 * Description:
 *   Initialize the WDT watchdog time.  The watchdog timer is initialized and
 *   registered as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int lc823450_wdt_initialize(void)
{
  struct lc823450_wdt_lowerhalf_s *priv = &g_wdtdev;

  priv->wdt_lh.ops = &g_wdgops;

  /* PTM clock enable */

  modifyreg32(MCLKCNTEXT1, 0,
              MCLKCNTEXT1_PTM0C_CLKEN | MCLKCNTEXT1_PTM0_CLKEN);
  modifyreg32(MRSTCNTEXT1, 0, MRSTCNTEXT1_PTM0_RSTB);

#ifdef CONFIG_LC823450_WDT_INTERRUPT
  /* Attach our WDT interrupt handler (But don't enable it yet) */

  irq_attach(LC823450_IRQ_WDT0, lc823450_wdt_interrupt, NULL);
#else
  if (getreg32(WDT_WT0RSTS) & (1 << WDT_WT0RSTS_RSTS))
    {
      syslog(LOG_EMERG, "**** WATCHDOG RESET****\n");
    }
#endif

  /* Register the watchdog driver as /dev/watchdog0 */

  watchdog_register("/dev/watchdog0",
                    (struct watchdog_lowerhalf_s *)priv);
  return OK;
}

#ifdef CONFIG_WATCHDOG_WORK
/****************************************************************************
 * Name: lc823450_wdt_work_enable
 ****************************************************************************/

void lc823450_wdt_work_enable(int en)
{
  if (en)
    {
      /* PTM clock enable */

      modifyreg32(MCLKCNTEXT1, 0,
                  MCLKCNTEXT1_PTM0C_CLKEN | MCLKCNTEXT1_PTM0_CLKEN);
      modifyreg32(MRSTCNTEXT1, 0, MRSTCNTEXT1_PTM0_RSTB);

      if (getreg32(WDT_WT0RSTS) & (1 << WDT_WT0RSTS_RSTS))
        {
          syslog(LOG_EMERG, "**** WATCHDOG RESET****\n");
        }

      if (getreg32(LOCKUPR) & LOCKUPR_LOCKUPR0)
        {
          wdinfo("**** LOCKUP DETECTED ****\n");
          putreg32(LOCKUPR_LOCKUPR0, LOCKUPR);
        }

      lc823450_wdt_settimeout(&g_wdtdev.wdt_lh,
                              CONFIG_WATCHDOG_WORK_TIMEOUT);

      work_queue(HPWORK, &wdg_work, wdg_work_func, NULL,
                 MSEC2TICK(CONFIG_WATCHDOG_WORK_TIMEOUT / 2));
    }
  else if (g_wdtdev.reload)
    {
      work_cancel(HPWORK, &wdg_work);
      lc823450_wdt_stop(&g_wdtdev.wdt_lh);
      g_wdtdev.reload = 0;
    }
}
#endif /* CONFIG_WATCHDOG_WORK */

#endif /* CONFIG_WATCHDOG && CONFIG_LC823450_WDT */
