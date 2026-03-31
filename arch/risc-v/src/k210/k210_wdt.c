/****************************************************************************
 * arch/risc-v/src/k210/k210_wdt.c
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

#if defined(CONFIG_K210_WDT0) || defined(CONFIG_K210_WDT1)

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>

#include "riscv_internal.h"
#include "k210.h"
#include "k210_sysctl.h"
#include "k210_wdt.h"
#include "hardware/k210_sysctl.h"
#include "hardware/k210_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K210_WDT_IN0_FREQ          26000000u
#define K210_WDT_DEFAULT_PCLK      (K210_WDT_IN0_FREQ / 2u)
#define K210_WDT_MAX_TOP           0x0f
#define K210_WDT_BASE_COUNT        (1ull << 16)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct k210_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;
  uintptr_t                    base;
  int                          irq;
  k210_clockid_t               clkid;
  k210_rstidx_t                rstidx;
  xcpt_t                       handler;
  void                        *upper;
  uint32_t                     timeout;
  uint8_t                      top;
  bool                         started;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int k210_wdt_start(struct watchdog_lowerhalf_s *lower);
static int k210_wdt_stop(struct watchdog_lowerhalf_s *lower);
static int k210_wdt_keepalive(struct watchdog_lowerhalf_s *lower);
static int k210_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                              struct watchdog_status_s *status);
static int k210_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                               uint32_t timeout);
static xcpt_t k210_wdt_capture(struct watchdog_lowerhalf_s *lower,
                               xcpt_t handler);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_k210_wdt_ops =
{
  .start      = k210_wdt_start,
  .stop       = k210_wdt_stop,
  .keepalive  = k210_wdt_keepalive,
  .getstatus  = k210_wdt_getstatus,
  .settimeout = k210_wdt_settimeout,
  .capture    = k210_wdt_capture,
  .ioctl      = NULL,
};

#ifdef CONFIG_K210_WDT0
static struct k210_wdt_lowerhalf_s g_k210_wdt0_lowerhalf =
{
  .ops     = &g_k210_wdt_ops,
  .base    = K210_WDT0_BASE,
  .irq     = K210_IRQ_WDT0,
  .clkid   = K210_CLOCK_WDT0,
  .rstidx  = K210_RESET_WDT0,
  .timeout = 0,
  .top     = K210_WDT_MAX_TOP,
};
#endif

#ifdef CONFIG_K210_WDT1
static struct k210_wdt_lowerhalf_s g_k210_wdt1_lowerhalf =
{
  .ops     = &g_k210_wdt_ops,
  .base    = K210_WDT1_BASE,
  .irq     = K210_IRQ_WDT1,
  .clkid   = K210_CLOCK_WDT1,
  .rstidx  = K210_RESET_WDT1,
  .timeout = 0,
  .top     = K210_WDT_MAX_TOP,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t k210_wdt_get_pclk(struct k210_wdt_lowerhalf_s *priv)
{
  uint32_t pclk;

  if (!priv->started)
    {
      return K210_WDT_DEFAULT_PCLK;
    }

  pclk = k210_sysctl_clock_get_freq(priv->clkid);

  if (pclk == 0)
    {
      pclk = K210_WDT_DEFAULT_PCLK;
    }

  return pclk;
}

static void k210_wdt_set_threshold_zero(struct k210_wdt_lowerhalf_s *priv)
{
  uint32_t clearbits = (priv->clkid == K210_CLOCK_WDT0) ?
                       CLK_TH6_WDT0_MASK : CLK_TH6_WDT1_MASK;

  modifyreg32(K210_SYSCTL_CLK_TH6, clearbits, 0);
}

static void k210_wdt_enable(struct k210_wdt_lowerhalf_s *priv)
{
  uint32_t regval;

  putreg32(K210_WDT_CRR_RESTART, K210_WDT_CRR(priv->base));
  regval = getreg32(K210_WDT_CR(priv->base));
  regval |= K210_WDT_CR_ENABLE;
  putreg32(regval, K210_WDT_CR(priv->base));
}

static void k210_wdt_disable(struct k210_wdt_lowerhalf_s *priv)
{
  uint32_t regval;

  putreg32(K210_WDT_CRR_RESTART, K210_WDT_CRR(priv->base));
  regval = getreg32(K210_WDT_CR(priv->base));
  regval &= ~K210_WDT_CR_ENABLE;
  putreg32(regval, K210_WDT_CR(priv->base));
}

static void k210_wdt_feed_internal(struct k210_wdt_lowerhalf_s *priv)
{
  putreg32(K210_WDT_CRR_RESTART, K210_WDT_CRR(priv->base));
}

static void k210_wdt_clear_interrupt(struct k210_wdt_lowerhalf_s *priv)
{
  uint32_t regval;

  regval = getreg32(K210_WDT_EOI(priv->base));
  putreg32(regval, K210_WDT_EOI(priv->base));
}

static void k210_wdt_set_timeout_top(struct k210_wdt_lowerhalf_s *priv,
                                     uint8_t top)
{
  putreg32(K210_WDT_TORR_TOP(top), K210_WDT_TORR(priv->base));
}

static void k210_wdt_set_response_mode(struct k210_wdt_lowerhalf_s *priv,
                                       uint32_t mode)
{
  uint32_t regval;

  regval = getreg32(K210_WDT_CR(priv->base));
  regval &= ~K210_WDT_CR_RMOD_MASK;
  regval |= mode;
  putreg32(regval, K210_WDT_CR(priv->base));
}

static uint8_t k210_wdt_get_top(struct k210_wdt_lowerhalf_s *priv,
                                uint32_t timeout_ms)
{
  uint64_t counts;
  uint64_t level;
  uint8_t top = 0;

  counts = ((uint64_t)timeout_ms * (uint64_t)k210_wdt_get_pclk(priv) +
            999u) / 1000u;

  level = (counts + K210_WDT_BASE_COUNT - 1) / K210_WDT_BASE_COUNT;

  if (level == 0)
    {
      level = 1;
    }

  while ((1ull << top) < level && top < K210_WDT_MAX_TOP)
    {
      top++;
    }

  return top;
}

static uint32_t k210_wdt_top_to_timeout(struct k210_wdt_lowerhalf_s *priv,
                                        uint8_t top)
{
  uint64_t timeout;
  uint32_t pclk = k210_wdt_get_pclk(priv);

  timeout = (K210_WDT_BASE_COUNT << top) * 1000ull;
  timeout /= (uint64_t)pclk;

  return (uint32_t)timeout;
}

static int k210_wdt_interrupt(int irq, void *context, void *arg)
{
  struct k210_wdt_lowerhalf_s *priv = arg;

  if (priv->handler != NULL)
    {
      priv->handler(irq, context, priv->upper);
    }

  k210_wdt_clear_interrupt(priv);
  return OK;
}

/****************************************************************************
 * Name: k210_wdt_start
 ****************************************************************************/

static int k210_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  struct k210_wdt_lowerhalf_s *priv =
    (struct k210_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  if (priv->started)
    {
      return -EBUSY;
    }

  flags = enter_critical_section();

  ret = k210_sysctl_reset(priv->rstidx);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  k210_wdt_set_threshold_zero(priv);

  ret = k210_sysctl_clock_enable(priv->clkid);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  putreg32(1, K210_PLIC_PRIORITY +
              ((priv->irq - RISCV_IRQ_MEXT) * sizeof(uint32_t)));

  if (priv->handler != NULL)
    {
      k210_wdt_set_response_mode(priv, K210_WDT_CR_RMOD_INTERRUPT);
      up_enable_irq(priv->irq);
    }
  else
    {
      k210_wdt_set_response_mode(priv, K210_WDT_CR_RMOD_RESET);
      up_disable_irq(priv->irq);
    }

  k210_wdt_set_timeout_top(priv, priv->top);
  k210_wdt_enable(priv);
  priv->started = true;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: k210_wdt_stop
 ****************************************************************************/

static int k210_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  struct k210_wdt_lowerhalf_s *priv =
    (struct k210_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();
  up_disable_irq(priv->irq);
  k210_wdt_disable(priv);
  priv->started = false;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: k210_wdt_keepalive
 ****************************************************************************/

static int k210_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct k210_wdt_lowerhalf_s *priv =
    (struct k210_wdt_lowerhalf_s *)lower;

  if (!priv->started)
    {
      return -EIO;
    }

  k210_wdt_feed_internal(priv);
  return OK;
}

/****************************************************************************
 * Name: k210_wdt_getstatus
 ****************************************************************************/

static int k210_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                              struct watchdog_status_s *status)
{
  struct k210_wdt_lowerhalf_s *priv =
    (struct k210_wdt_lowerhalf_s *)lower;
  uint64_t timeleft = priv->timeout;

  status->flags = 0;

  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
      timeleft = ((uint64_t)getreg32(K210_WDT_CCVR(priv->base)) * 1000ull) /
                 (uint64_t)k210_wdt_get_pclk(priv);
      if (timeleft > priv->timeout)
        {
          timeleft = priv->timeout;
        }
    }

  if (priv->handler != NULL)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }
  else
    {
      status->flags |= WDFLAGS_RESET;
    }

  status->timeout = priv->timeout;
  status->timeleft = (uint32_t)timeleft;
  return OK;
}

/****************************************************************************
 * Name: k210_wdt_settimeout
 ****************************************************************************/

static int k210_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                               uint32_t timeout)
{
  struct k210_wdt_lowerhalf_s *priv =
    (struct k210_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  if (timeout == 0)
    {
      return -EINVAL;
    }

  if (timeout > k210_wdt_top_to_timeout(priv, K210_WDT_MAX_TOP))
    {
      return -ERANGE;
    }

  priv->top = k210_wdt_get_top(priv, timeout);
  priv->timeout = k210_wdt_top_to_timeout(priv, priv->top);

  if (!priv->started)
    {
      return OK;
    }

  flags = enter_critical_section();
  k210_wdt_set_timeout_top(priv, priv->top);
  k210_wdt_feed_internal(priv);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: k210_wdt_capture
 ****************************************************************************/

static xcpt_t k210_wdt_capture(struct watchdog_lowerhalf_s *lower,
                               xcpt_t handler)
{
  struct k210_wdt_lowerhalf_s *priv =
    (struct k210_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  flags = enter_critical_section();
  oldhandler = priv->handler;
  priv->handler = handler;

  if (priv->started)
    {
      if (handler != NULL)
        {
          putreg32(1, K210_PLIC_PRIORITY +
                      ((priv->irq - RISCV_IRQ_MEXT) * sizeof(uint32_t)));
          k210_wdt_set_response_mode(priv, K210_WDT_CR_RMOD_INTERRUPT);
          up_enable_irq(priv->irq);
        }
      else
        {
          up_disable_irq(priv->irq);
          k210_wdt_set_response_mode(priv, K210_WDT_CR_RMOD_RESET);
        }
    }

  leave_critical_section(flags);
  return oldhandler;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_wdt_initialize
 ****************************************************************************/

int k210_wdt_initialize(const char *devpath, k210_wdt_id_t id)
{
  struct k210_wdt_lowerhalf_s *priv;
  void *handle;
  int ret;

  switch (id)
    {
#ifdef CONFIG_K210_WDT0
      case K210_WDT_DEVICE0:
        priv = &g_k210_wdt0_lowerhalf;
        break;
#endif
#ifdef CONFIG_K210_WDT1
      case K210_WDT_DEVICE1:
        priv = &g_k210_wdt1_lowerhalf;
        break;
#endif
      default:
        return -EINVAL;
    }

  if (priv->timeout == 0)
    {
      priv->timeout = k210_wdt_top_to_timeout(priv, priv->top);
    }

  ret = irq_attach(priv->irq, k210_wdt_interrupt, priv);
  if (ret < 0)
    {
      return ret;
    }

  up_disable_irq(priv->irq);

  handle = watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
  if (handle == NULL)
    {
      return -EEXIST;
    }

  priv->upper = handle;
  return OK;
}

#endif /* CONFIG_K210_WDT0 || CONFIG_K210_WDT1 */
