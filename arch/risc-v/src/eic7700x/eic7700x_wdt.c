/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_wdt.c
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

/* The watchdogs, whose bark is optional but whose bite is real.
 *
 * A timeout here resets the whole chip: the reset chapter lists the
 * watchdog as a source beside power-on, and the cause register remembers
 * it afterwards, which is the entire point on a board administered from
 * the far end of a network cable: a hang becomes a reboot instead of a
 * trip to the debugger.
 *
 * The block's one eccentricity drives the shape of this driver.  The
 * enable bit can be set but never cleared; only a reset of the watchdog
 * itself forgets it.  So stopping the dog is done by asserting the
 * block's reset line in the CRG and leaving it held, which is exactly how
 * the boot firmware hands the blocks over, and starting is a full bring
 * up from that state every time: release the reset, program the timeout,
 * enable.  Nothing read from the block while it was held in reset is to
 * be trusted, so the driver keeps its own idea of the timeout.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>

#include "eic7700x_wdt.h"
#include "hardware/eic7700x_wdt.h"
#include "riscv_internal.h"

#ifdef CONFIG_EIC7700X_WDT

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct eic7700x_wdt_s
{
  struct watchdog_lowerhalf_s lower;  /* Must be first                    */
  uintptr_t   base;                   /* Register base                    */
  FAR const char *clkname;            /* Its pclk gate                    */
  uint32_t    rstmask;                /* Its release bit in the CRG       */
  int         irq;                    /* Its PLIC number                  */
  uint32_t    rate;                   /* pclk in Hz once measured         */
  uint32_t    timeout;                /* Granted timeout in ms            */
  xcpt_t      handler;                /* User handler, capture mode       */
  bool        started;                /* Out of reset and counting        */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int eic7700x_wdt_start(FAR struct watchdog_lowerhalf_s *lower);
static int eic7700x_wdt_stop(FAR struct watchdog_lowerhalf_s *lower);
static int eic7700x_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int eic7700x_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                                  FAR struct watchdog_status_s *status);
static int eic7700x_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                   uint32_t timeout);
static xcpt_t eic7700x_wdt_capture(FAR struct watchdog_lowerhalf_s *lower,
                                   xcpt_t handler);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_eic7700x_wdt_ops =
{
  .start      = eic7700x_wdt_start,
  .stop       = eic7700x_wdt_stop,
  .keepalive  = eic7700x_wdt_keepalive,
  .getstatus  = eic7700x_wdt_getstatus,
  .settimeout = eic7700x_wdt_settimeout,
  .capture    = eic7700x_wdt_capture,
};

static struct eic7700x_wdt_s g_eic7700x_wdt[EIC7700X_WDT_COUNT] =
{
  {
    .lower.ops = &g_eic7700x_wdt_ops,
    .base      = EIC7700X_WDT_BASE(0),
    .clkname   = "lsp_wdt0_pclk",
    .rstmask   = WDT_RST_RELEASE(0),
    .irq       = EIC7700X_IRQ_WDT(0),
  },
  {
    .lower.ops = &g_eic7700x_wdt_ops,
    .base      = EIC7700X_WDT_BASE(1),
    .clkname   = "lsp_wdt1_pclk",
    .rstmask   = WDT_RST_RELEASE(1),
    .irq       = EIC7700X_IRQ_WDT(1),
  },
  {
    .lower.ops = &g_eic7700x_wdt_ops,
    .base      = EIC7700X_WDT_BASE(2),
    .clkname   = "lsp_wdt2_pclk",
    .rstmask   = WDT_RST_RELEASE(2),
    .irq       = EIC7700X_IRQ_WDT(2),
  },
  {
    .lower.ops = &g_eic7700x_wdt_ops,
    .base      = EIC7700X_WDT_BASE(3),
    .clkname   = "lsp_wdt3_pclk",
    .rstmask   = WDT_RST_RELEASE(3),
    .irq       = EIC7700X_IRQ_WDT(3),
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_wdt_top
 *
 * Description:
 *   The smallest power-of-two period not shorter than the millisecond
 *   count asked for.  The block offers 2^(16+TOP) clocks, TOP 0 to 15,
 *   and rounding down would promise more protection than the hardware
 *   delivers, so the rounding is always up and the caller learns the
 *   granted value.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_wdt_blockreset
 *
 * Description:
 *   Hold the block in reset or let it out, with one register write and
 *   no framework in the way.  Deliberately safe to call from a dying
 *   kernel: the panic notifier stops watchdogs to preserve a crash
 *   scene, and this is the path it takes.
 *
 ****************************************************************************/

static void eic7700x_wdt_blockreset(FAR struct eic7700x_wdt_s *priv,
                                    bool held)
{
  irqstate_t flags;
  uint32_t v;

  flags = enter_critical_section();
  v = getreg32(EIC7700X_WDT_RST_CTRL);
  if (held)
    {
      v &= ~priv->rstmask;
    }
  else
    {
      v |= priv->rstmask;
    }

  putreg32(v, EIC7700X_WDT_RST_CTRL);
  leave_critical_section(flags);
}

static uint32_t eic7700x_wdt_top(FAR struct eic7700x_wdt_s *priv,
                                 uint32_t ms, FAR uint32_t *granted)
{
  uint64_t cycles = (uint64_t)ms * priv->rate / 1000;
  uint32_t top;

  for (top = 0; top < WDT_TORR_TOP_MAX; top++)
    {
      if ((1ull << (WDT_TORR_BASE_SHIFT + top)) >= cycles)
        {
          break;
        }
    }

  *granted = (uint32_t)((1ull << (WDT_TORR_BASE_SHIFT + top)) * 1000 /
                        priv->rate);
  return top;
}

/****************************************************************************
 * Name: eic7700x_wdt_program
 *
 * Description:
 *   Bring the block from held-in-reset to counting.  This is the only
 *   path that starts it, because the enable bit cannot be cleared and so
 *   every start must begin from the block's reset state.
 *
 ****************************************************************************/

static int eic7700x_wdt_program(FAR struct eic7700x_wdt_s *priv)
{
  uint32_t granted;
  uint32_t top;
  uint32_t cr;

  eic7700x_wdt_blockreset(priv, true);
  eic7700x_wdt_blockreset(priv, false);

  /* Wait until the block answers with its own name, which both proves
   * the reset really released and spends the moment the synchronisers
   * need before the registers are trustworthy.
   */

  while (getreg32(priv->base + EIC7700X_WDT_COMP_TYPE) !=
         WDT_COMP_TYPE_VALUE)
    {
    }

  /* Leave the protection level register alone.  Its reset value is what
   * PERMITS these writes: clearing it, the obvious defensive move, and
   * the first thing this driver tried, makes the block silently drop
   * every write to the timeout register while still accepting the enable
   * and the feed, which arms a third-of-a-millisecond watchdog nothing
   * can outrun.  One boot died that way and another hung forever proving
   * it.  Only a block reset restores the register, since with it cleared
   * it cannot even be written back.
   */

  top = eic7700x_wdt_top(priv, priv->timeout, &granted);
  priv->timeout = granted;

  /* Written and then read back, because a silently dropped write to this
   * register is precisely how the block turns into a landmine; see
   * above.  If it will not take, the caller hears about it rather than
   * the machine finding out.
   */

  putreg32(WDT_TORR_TOP(top), priv->base + EIC7700X_WDT_TORR);
  if ((getreg32(priv->base + EIC7700X_WDT_TORR) & 0xf) != top)
    {
      wderr("ERROR: wdt timeout register will not hold %lu\n",
            (unsigned long)top);
      return -EIO;
    }

  cr = WDT_CR_EN;
  if (priv->handler != NULL)
    {
      cr |= WDT_CR_RMOD;
    }

  putreg32(cr, priv->base + EIC7700X_WDT_CR);

  /* The programmed period takes effect at the first kick */

  putreg32(WDT_CRR_RESTART, priv->base + EIC7700X_WDT_CRR);
  return OK;
}

/****************************************************************************
 * Name: eic7700x_wdt_interrupt
 *
 * Description:
 *   First timeout in interrupt-then-reset mode.  Only wired while a
 *   capture handler is installed; the handler decides whether to feed.
 *   If it neither feeds nor is installed, the second timeout resets the
 *   chip, which is the point.
 *
 ****************************************************************************/

static int eic7700x_wdt_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct eic7700x_wdt_s *priv = arg;

  if (priv->handler != NULL)
    {
      priv->handler(irq, context, arg);
    }

  getreg32(priv->base + EIC7700X_WDT_EOI);
  return OK;
}

/****************************************************************************
 * Name: eic7700x_wdt_start
 *
 * Description:
 *   Arm the watchdog.  Goes through eic7700x_wdt_program(), since the
 *   enable bit cannot be cleared and every start therefore begins from
 *   the block's reset state.
 *
 ****************************************************************************/

static int eic7700x_wdt_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct eic7700x_wdt_s *priv = (FAR struct eic7700x_wdt_s *)lower;
  int ret;

  if (priv->started)
    {
      return OK;
    }

  if (priv->timeout == 0)
    {
      return -EINVAL;
    }

  ret = eic7700x_wdt_program(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: eic7700x_wdt_stop
 *
 * Description:
 *   Disarm the watchdog by holding its block in reset, which is the only
 *   way to clear an enable bit the hardware makes unclearable.
 *
 ****************************************************************************/

static int eic7700x_wdt_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct eic7700x_wdt_s *priv = (FAR struct eic7700x_wdt_s *)lower;

  /* The enable bit cannot be cleared, so stopping is asserting the
   * block's reset and leaving it held: the state the firmware hands
   * the block over in.  Direct and lock-free on purpose: the panic
   * notifier calls this from a kernel that has stopped being one, which
   * is how a crash scene is kept from the dogs.
   */

  eic7700x_wdt_blockreset(priv, true);
  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: eic7700x_wdt_keepalive
 *
 * Description:
 *   Feed the watchdog, restarting its count.  Does nothing unless the
 *   watchdog is armed, so a stray feed cannot start one.
 *
 ****************************************************************************/

static int eic7700x_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct eic7700x_wdt_s *priv = (FAR struct eic7700x_wdt_s *)lower;

  if (priv->started)
    {
      putreg32(WDT_CRR_RESTART, priv->base + EIC7700X_WDT_CRR);
    }

  return OK;
}

/****************************************************************************
 * Name: eic7700x_wdt_getstatus
 *
 * Description:
 *   Report whether the watchdog is armed, whether a handler is installed,
 *   the timeout in force and the time left, read from the current count.
 *
 ****************************************************************************/

static int eic7700x_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                                  FAR struct watchdog_status_s *status)
{
  FAR struct eic7700x_wdt_s *priv = (FAR struct eic7700x_wdt_s *)lower;

  status->flags = priv->started ? WDFLAGS_ACTIVE : 0;
  if (priv->handler != NULL)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }

  status->timeout = priv->timeout;
  if (priv->started)
    {
      status->timeleft =
        (uint32_t)((uint64_t)getreg32(priv->base + EIC7700X_WDT_CCVR) *
                   1000 / priv->rate);
    }
  else
    {
      status->timeleft = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: eic7700x_wdt_settimeout
 *
 * Description:
 *   Set the timeout, rounded up to the next power of two of the peripheral
 *   clock, which is what the hardware offers.  A request beyond the
 *   ceiling is refused rather than shortened, so a caller that plans its
 *   feeding around the value it asked for is never given less.
 *
 ****************************************************************************/

static int eic7700x_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                   uint32_t timeout)
{
  FAR struct eic7700x_wdt_s *priv = (FAR struct eic7700x_wdt_s *)lower;
  uint32_t granted;
  uint32_t top;

  if (timeout == 0)
    {
      return -EINVAL;
    }

  /* Refuse what the hardware cannot deliver rather than quietly granting
   * less.  A caller plans its feeding schedule around the number it asked
   * for: the auto-monitor does exactly that, at compile time, and a
   * shorter dog under a longer schedule dies on time, every time.  Found
   * the hard way: a sixty second request silently became eleven seconds,
   * fed every thirty.
   */

  eic7700x_wdt_top(priv, timeout, &granted);
  if (granted < timeout)
    {
      wderr("ERROR: %lu ms is more than this watchdog has; its most is "
            "%lu ms\n", (unsigned long)timeout, (unsigned long)granted);
      return -ERANGE;
    }

  priv->timeout = timeout;

  if (priv->started)
    {
      /* The block is live: write the new range and kick so it latches.
       * The granted value may be longer than asked; the status call
       * reports the truth.
       */

      top = eic7700x_wdt_top(priv, timeout, &granted);
      priv->timeout = granted;
      putreg32(WDT_TORR_TOP(top), priv->base + EIC7700X_WDT_TORR);
      putreg32(WDT_CRR_RESTART, priv->base + EIC7700X_WDT_CRR);
    }
  else
    {
      /* Not running: just record it, and report what would be granted */

      top = eic7700x_wdt_top(priv, timeout, &granted);
      priv->timeout = granted;
    }

  return OK;
}

/****************************************************************************
 * Name: eic7700x_wdt_capture
 *
 * Description:
 *   Install a handler to run on timeout instead of letting the chip reset,
 *   and return the handler it replaces.
 *
 ****************************************************************************/

static xcpt_t eic7700x_wdt_capture(FAR struct watchdog_lowerhalf_s *lower,
                                   xcpt_t handler)
{
  FAR struct eic7700x_wdt_s *priv = (FAR struct eic7700x_wdt_s *)lower;
  irqstate_t flags;
  xcpt_t old;
  uint32_t cr;

  flags = enter_critical_section();
  old = priv->handler;
  priv->handler = handler;

  if (priv->started)
    {
      /* Flip the response mode in place.  The mode bit is not write-once
       * the way the enable bit is.
       */

      cr = getreg32(priv->base + EIC7700X_WDT_CR);
      if (handler != NULL)
        {
          cr |= WDT_CR_RMOD;
        }
      else
        {
          cr &= ~WDT_CR_RMOD;
        }

      putreg32(cr, priv->base + EIC7700X_WDT_CR);
    }

  if (handler != NULL)
    {
      up_enable_irq(priv->irq);
    }
  else
    {
      up_disable_irq(priv->irq);
    }

  leave_critical_section(flags);
  return old;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_wdt_initialize
 ****************************************************************************/

int eic7700x_wdt_initialize(int n, FAR const char *devpath)
{
  FAR struct eic7700x_wdt_s *priv;
  FAR struct clk_s *clk;
  int ret;

  if (n < 0 || n >= EIC7700X_WDT_COUNT)
    {
      return -EINVAL;
    }

  priv = &g_eic7700x_wdt[n];

  clk = clk_get(priv->clkname);
  if (clk == NULL)
    {
      wderr("ERROR: no clock %s\n", priv->clkname);
      return -ENODEV;
    }

  ret = clk_enable(clk);
  if (ret < 0)
    {
      wderr("ERROR: clock %s will not start: %d\n", priv->clkname, ret);
      return ret;
    }

  priv->rate = clk_get_rate(clk);
  if (priv->rate == 0)
    {
      wderr("ERROR: clock %s reports no rate\n", priv->clkname);
      clk_disable(clk);
      return -ENODEV;
    }

  ret = irq_attach(priv->irq, eic7700x_wdt_interrupt, priv);
  if (ret < 0)
    {
      wderr("ERROR: wdt%d irq %d: %d\n", n, priv->irq, ret);
      clk_disable(clk);
      return ret;
    }

  if (watchdog_register(devpath, &priv->lower) == NULL)
    {
      wderr("ERROR: wdt%d will not register as %s\n", n, devpath);
      irq_detach(priv->irq);
      clk_disable(clk);
      return -EEXIST;
    }

  return OK;
}

#endif /* CONFIG_EIC7700X_WDT */
