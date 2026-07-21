/****************************************************************************
 * arch/arm/src/common/ameba/ameba_wdg.c
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

/* NuttX watchdog lower half for the Realtek Ameba system watchdog.  It
 * exposes the on-chip WDG as a NuttX watchdog at /dev/watchdog0 with the
 * usual start/stop/keepalive/getstatus/settimeout methods, plus capture
 * (a pre-timeout "early interrupt" callback).
 *
 * The WDG is driven through the SDK fwlib WDG API.  Those routines are
 * marked _LONG_CALL_ and are resolved from ROM (they appear in the fwlib
 * ROM symbol table), so -- like the UART/SPI drivers -- nothing extra is
 * added to the board build.  Each call takes a WDG_TypeDef * register base;
 * the base and the interrupt vector are the only chip-specific facts and
 * live in the per-chip ameba_wdg_chip.h.  To keep the vendor headers out of
 * the NuttX include world the few fwlib symbols, constants and structures
 * used here are declared locally (layout-compatible mirrors) rather than
 * pulled in from the SDK <ameba_wdg.h>.
 *
 * Hardware note (the "one catch"): the Ameba system WDG can be enabled but
 * it *cannot be stopped by software* -- the fwlib exposes WDG_Enable() with
 * no WDG_Cmd(DISABLE) counterpart in ROM, and the vendor HAL asserts on
 * stop.  To honour the NuttX stop() contract we instead arm the WDG early
 * interrupt and refresh the counter from its handler, so the timer keeps
 * running but can never reach the reset threshold -- an effective "stop".
 * The same early interrupt backs capture(): when a user handler is
 * registered it is called at the pre-timeout point instead.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/watchdog.h>

#include "ameba_wdg.h"
#include "ameba_wdg_chip.h"

#ifdef CONFIG_WATCHDOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* "state" argument for the fwlib enable/disable style APIs. */

#define AMEBA_DISABLE               0x0
#define AMEBA_ENABLE                0x1

/* fwlib WDG_CR interrupt bits (from the SDK ameba_wdg.h; identical on every
 * current Ameba chip): early-interrupt enable and its write-1-to-clear flag.
 */

#define AMEBA_WDG_BIT_EIE           ((uint32_t)1 << 16)  /* early int enable  */
#define AMEBA_WDG_BIT_EIC           ((uint32_t)1 << 17)  /* early int clear   */

/* WDG timeout is a 16-bit millisecond count; the early interrupt fires this
 * many milliseconds before the timeout.  Match the vendor HAL: 100ms of head
 * room, or half the timeout when that is under 100ms (at least 1ms).
 */

#define AMEBA_WDG_TIMEOUT_MAX       65535u
#define AMEBA_WDG_DEFAULT_MS        5000u
#define AMEBA_WDG_EICNT(ms)         ((ms) > 100 ? 100 : (((ms) + 1) >> 1))

/* The chip header hands us the register base as a plain address. */

#define AMEBA_WDG  ((struct ameba_wdg_dev_s *)AMEBA_WDG_BASE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Layout-compatible mirrors of the fwlib WDG structures (same field order
 * and types); passed by address to the fwlib WDG API.  The register block is
 * only ever touched through the fwlib, so it is an opaque base here -- this
 * driver never dereferences any field below, it only hands the base pointer
 * to the ROM routines.  That is what keeps it chip-neutral: amebasmart's
 * WDG_TypeDef has no 0x010 "dummy" register (only 4 words), but since the
 * field is never read the extra word is harmless padding on that chip.
 */

/* WDG_TypeDef */

struct ameba_wdg_dev_s
{
  volatile uint32_t mkeyr;           /* 0x000 magic key register  */
  volatile uint32_t cr;              /* 0x004 control register    */
  volatile uint32_t rlr;             /* 0x008 reload register     */
  volatile uint32_t winr;            /* 0x00C window register     */
  volatile uint32_t dummy;           /* 0x010 (absent on amebasmart) */
};

/* WDG_InitTypeDef */

struct ameba_wdg_init_s
{
  uint16_t window;                   /* feed-protect window       */
  uint16_t timeout;                  /* timeout count, in ms      */
  uint16_t eicnt;                    /* early-interrupt threshold */
  uint16_t eimod;                    /* early-interrupt enable    */
};

/* This is the private watchdog lower-half state.  It must be cast-compatible
 * with struct watchdog_lowerhalf_s (the ops pointer is first).
 */

struct ameba_wdg_lowerhalf_s
{
  const struct watchdog_ops_s *ops;  /* Lower-half operations vtable */

  spinlock_t lock;                   /* Protects the state below */
  uint32_t   timeout;                /* The last programmed timeout (ms) */
  clock_t    lastreset;              /* systime of the last (re)start/feed */
  bool       configured;             /* WDG_Init has run at least once */
  bool       started;                /* Counting towards a reset */
  bool       autofeed;               /* stop(): EI handler refreshes to inhibit
                                      * the reset (the WDG cannot be halted) */
  xcpt_t     handler;                /* capture() pre-timeout callback */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib WDG API (resolved from ROM).  Each call takes the WDG register
 * base; none of them need the vendor headers.
 */

extern void WDG_StructInit(struct ameba_wdg_init_s *init);
extern void WDG_Init(struct ameba_wdg_dev_s *wdg,
                     struct ameba_wdg_init_s *init);
extern void WDG_Enable(struct ameba_wdg_dev_s *wdg);
extern void WDG_Timeout(struct ameba_wdg_dev_s *wdg, uint32_t timeout);
extern void WDG_Refresh(struct ameba_wdg_dev_s *wdg);
extern void WDG_INTConfig(struct ameba_wdg_dev_s *wdg, uint32_t wdg_it,
                          uint32_t newstate);
extern void WDG_ClearINT(struct ameba_wdg_dev_s *wdg, uint32_t intrbit);

static int  ameba_wdg_interrupt(int irq, void *context, void *arg);

/* Watchdog lower-half operations. */

static int  ameba_wdg_start(struct watchdog_lowerhalf_s *lower);
static int  ameba_wdg_stop(struct watchdog_lowerhalf_s *lower);
static int  ameba_wdg_keepalive(struct watchdog_lowerhalf_s *lower);
static int  ameba_wdg_getstatus(struct watchdog_lowerhalf_s *lower,
                                struct watchdog_status_s *status);
static int  ameba_wdg_settimeout(struct watchdog_lowerhalf_s *lower,
                                 uint32_t timeout);
static xcpt_t ameba_wdg_capture(struct watchdog_lowerhalf_s *lower,
                                xcpt_t handler);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_wdg_ops =
{
  .start      = ameba_wdg_start,
  .stop       = ameba_wdg_stop,
  .keepalive  = ameba_wdg_keepalive,
  .getstatus  = ameba_wdg_getstatus,
  .settimeout = ameba_wdg_settimeout,
  .capture    = ameba_wdg_capture,
  .ioctl      = NULL,
};

static struct ameba_wdg_lowerhalf_s g_wdg_lowerhalf =
{
  .ops  = &g_wdg_ops,
  .lock = SP_UNLOCKED,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_wdg_hwinit
 *
 * Description:
 *   Program the fwlib WDG for a timeout (in ms) with the early interrupt
 *   configured but left masked.  Only valid before the WDG is enabled; once
 *   running, settimeout() uses WDG_Timeout() instead.
 *
 ****************************************************************************/

static void ameba_wdg_hwinit(struct ameba_wdg_lowerhalf_s *priv,
                             uint32_t timeout)
{
  struct ameba_wdg_init_s init;

  WDG_StructInit(&init);
  init.timeout = (uint16_t)timeout;
  init.eicnt   = (uint16_t)AMEBA_WDG_EICNT(timeout);

  /* The early-interrupt channel (and its EICNT threshold) is only actually
   * armed by WDG_Init when EIMOD is enabled -- the vendor HAL always inits
   * with EIMOD=ENABLE when it wants the EI.  So arm it here, then gate the
   * delivery off with the EIE bit; stop()/capture() flip EIE at run time
   * (which is valid after WDG_Enable) without ever re-initialising.
   */

  init.eimod   = AMEBA_ENABLE;
  WDG_Init(AMEBA_WDG, &init);
  WDG_INTConfig(AMEBA_WDG, AMEBA_WDG_BIT_EIE, AMEBA_DISABLE);

  priv->timeout    = timeout;
  priv->configured = true;
}

/****************************************************************************
 * Name: ameba_wdg_ei
 *
 * Description:
 *   Enable or disable delivery of the early interrupt via the CR EIE bit.
 *   The IRQ itself is attached once in ameba_wdg_initialize(); the NVIC line
 *   stays enabled and this bit gates whether the WDG actually raises it.
 *   Called with the driver lock held.
 *
 ****************************************************************************/

static void ameba_wdg_ei(bool enable)
{
  /* Clear a possibly-pending flag before (un)masking.  The WDG runs off a
   * slow clock, so the vendor examples clear the EI flag twice.
   */

  WDG_ClearINT(AMEBA_WDG, AMEBA_WDG_BIT_EIC);
  WDG_ClearINT(AMEBA_WDG, AMEBA_WDG_BIT_EIC);
  WDG_INTConfig(AMEBA_WDG, AMEBA_WDG_BIT_EIE,
                enable ? AMEBA_ENABLE : AMEBA_DISABLE);
}

/****************************************************************************
 * Name: ameba_wdg_interrupt
 *
 * Description:
 *   WDG early-interrupt handler, fired AMEBA_WDG_EICNT() ms before the reset
 *   threshold.  A registered capture handler gets first refusal; otherwise,
 *   when a stop() has requested it, the counter is refreshed to inhibit the
 *   reset (the hardware cannot be halted any other way).
 *
 ****************************************************************************/

static int ameba_wdg_interrupt(int irq, void *context, void *arg)
{
  struct ameba_wdg_lowerhalf_s *priv =
    (struct ameba_wdg_lowerhalf_s *)arg;

  if (priv->handler != NULL)
    {
      priv->handler(irq, context, arg);
    }

  if (priv->autofeed)
    {
      /* stop() asked us to keep the counter fed so the reset never lands. */

      WDG_Refresh(AMEBA_WDG);
    }
  else if (priv->handler != NULL)
    {
      /* Pure capture: the handler has had its pre-timeout notification and
       * we deliberately do NOT feed, so the reset must follow.  The EI is
       * level-based -- with the counter left past the EI threshold it would
       * re-assert the moment EIC is cleared and storm the CPU, starving
       * everything else until the reset.  Mask EIE so this fires exactly
       * once and the WDG runs cleanly on to the timeout reset.
       */

      WDG_INTConfig(AMEBA_WDG, AMEBA_WDG_BIT_EIE, AMEBA_DISABLE);
    }

  /* Clear the EI flag twice -- the WDG's slow clock can miss a single
   * write-1-to-clear, which would re-enter this handler immediately.
   */

  WDG_ClearINT(AMEBA_WDG, AMEBA_WDG_BIT_EIC);
  WDG_ClearINT(AMEBA_WDG, AMEBA_WDG_BIT_EIC);

  return OK;
}

/****************************************************************************
 * Name: ameba_wdg_start
 *
 * Description:
 *   Start (enable) the watchdog timer.
 *
 ****************************************************************************/

static int ameba_wdg_start(struct watchdog_lowerhalf_s *lower)
{
  struct ameba_wdg_lowerhalf_s *priv =
    (struct ameba_wdg_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  /* If start() precedes any settimeout(), come up on a sane default. */

  if (!priv->configured)
    {
      ameba_wdg_hwinit(priv, AMEBA_WDG_DEFAULT_MS);
    }

  /* A restart clears any prior stop() auto-feed. */

  priv->autofeed = false;

  WDG_Enable(AMEBA_WDG);

  /* hwinit() leaves the EI gated off, so re-open it here iff a capture
   * handler is registered (capture() may have run before this start()).
   * The EIE bit only takes effect after WDG_Enable(), so this must run
   * *after* the enable above -- gating it earlier is silently ignored.
   */

  ameba_wdg_ei(priv->handler != NULL);

  priv->started   = true;
  priv->lastreset = clock_systime_ticks();

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: ameba_wdg_stop
 *
 * Description:
 *   Stop the watchdog timer.  The Ameba WDG cannot actually be halted once
 *   enabled, so this arms the early interrupt to refresh the counter
 *   forever, which prevents the reset from ever being reached.
 *
 ****************************************************************************/

static int ameba_wdg_stop(struct watchdog_lowerhalf_s *lower)
{
  struct ameba_wdg_lowerhalf_s *priv =
    (struct ameba_wdg_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  if (priv->started)
    {
      /* Hardware cannot be disabled; inhibit the reset via the early
       * interrupt (unless a capture handler is already driving it).
       */

      priv->autofeed = true;
      if (priv->handler == NULL)
        {
          ameba_wdg_ei(true);
        }

      WDG_Refresh(AMEBA_WDG);
      priv->started = false;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: ameba_wdg_keepalive
 *
 * Description:
 *   Reset ("feed") the watchdog timer.
 *
 ****************************************************************************/

static int ameba_wdg_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct ameba_wdg_lowerhalf_s *priv =
    (struct ameba_wdg_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  WDG_Refresh(AMEBA_WDG);
  priv->lastreset = clock_systime_ticks();
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: ameba_wdg_getstatus
 *
 * Description:
 *   Return the current watchdog timer status.
 *
 ****************************************************************************/

static int ameba_wdg_getstatus(struct watchdog_lowerhalf_s *lower,
                               struct watchdog_status_s *status)
{
  struct ameba_wdg_lowerhalf_s *priv =
    (struct ameba_wdg_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t elapsed;

  flags = spin_lock_irqsave(&priv->lock);

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  if (priv->handler != NULL)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }
  else if (!priv->autofeed)
    {
      status->flags |= WDFLAGS_RESET;
    }

  status->timeout = priv->timeout;

  /* Time left = timeout - time since the last feed/start (clamped at 0). */

  elapsed = TICK2MSEC(clock_systime_ticks() - priv->lastreset);
  status->timeleft = (elapsed >= priv->timeout) ? 0 :
                     (priv->timeout - elapsed);

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: ameba_wdg_settimeout
 *
 * Description:
 *   Set a new timeout value (in milliseconds) and reset the watchdog.
 *
 ****************************************************************************/

static int ameba_wdg_settimeout(struct watchdog_lowerhalf_s *lower,
                                uint32_t timeout)
{
  struct ameba_wdg_lowerhalf_s *priv =
    (struct ameba_wdg_lowerhalf_s *)lower;
  irqstate_t flags;

  if (timeout == 0 || timeout > AMEBA_WDG_TIMEOUT_MAX)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->lock);

  if (priv->started)
    {
      /* Already counting: RLR reconfiguration is not allowed, so use the
       * fwlib run-time timeout update and re-feed.
       */

      WDG_Timeout(AMEBA_WDG, timeout);
      WDG_Refresh(AMEBA_WDG);
      priv->timeout = timeout;
    }
  else
    {
      ameba_wdg_hwinit(priv, timeout);
    }

  priv->lastreset = clock_systime_ticks();

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: ameba_wdg_capture
 *
 * Description:
 *   Register a pre-timeout ("early interrupt") handler in place of the reset
 *   behaviour, or restore the reset behaviour when handler is NULL.  Returns
 *   the previous handler.
 *
 ****************************************************************************/

static xcpt_t ameba_wdg_capture(struct watchdog_lowerhalf_s *lower,
                                xcpt_t handler)
{
  struct ameba_wdg_lowerhalf_s *priv =
    (struct ameba_wdg_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  flags = spin_lock_irqsave(&priv->lock);

  oldhandler    = priv->handler;
  priv->handler = handler;

  /* Keep the early interrupt on while either a capture handler or a stop()
   * auto-feed needs it; otherwise mask it and let the WDG reset on timeout.
   */

  ameba_wdg_ei(handler != NULL || priv->autofeed);

  spin_unlock_irqrestore(&priv->lock, flags);
  return oldhandler;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_wdg_initialize
 ****************************************************************************/

int ameba_wdg_initialize(void)
{
  struct ameba_wdg_lowerhalf_s *priv = &g_wdg_lowerhalf;

  /* Attach the early interrupt once.  The NVIC line stays enabled for the
   * lifetime of the driver; the CR EIE bit (flipped by stop()/capture()) is
   * what actually gates whether the WDG raises it, and it comes up masked.
   */

  irq_attach(AMEBA_WDG_IRQ, ameba_wdg_interrupt, priv);
  up_enable_irq(AMEBA_WDG_IRQ);

  /* Register the watchdog in the stopped state; nothing counts until the
   * application programs a timeout and starts it.
   */

  if (watchdog_register(CONFIG_WATCHDOG_DEVPATH,
                        (struct watchdog_lowerhalf_s *)priv) == NULL)
    {
      up_disable_irq(AMEBA_WDG_IRQ);
      irq_detach(AMEBA_WDG_IRQ);
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_WATCHDOG */
