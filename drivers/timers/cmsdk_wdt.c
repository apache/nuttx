/****************************************************************************
 * drivers/timers/cmsdk_wdt.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>
#include <nuttx/timers/cmsdk_wdt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CMSDK APB watchdog register offsets (ARM DDI0479) */

#define CMSDK_WDT_LOAD      0x000  /* Reload value */
#define CMSDK_WDT_VALUE     0x004  /* Current count (RO) */
#define CMSDK_WDT_CONTROL   0x008  /* Control register */
#define CMSDK_WDT_INTCLR    0x00c  /* Interrupt clear (WO): clears + reloads */
#define CMSDK_WDT_RIS       0x010  /* Raw interrupt status */
#define CMSDK_WDT_MIS       0x014  /* Masked interrupt status */
#define CMSDK_WDT_LOCK      0xc00  /* Lock register */

/* Control register bits */

#define CMSDK_WDT_CTRL_INTEN  (1 << 0)  /* Enable counter + interrupt */
#define CMSDK_WDT_CTRL_RESEN  (1 << 1)  /* Enable reset output */

/* Magic value that unlocks write access via WDOGLOCK */

#define CMSDK_WDT_UNLOCK      0x1acce551

#define cmsdk_wdt_getreg(b, o) \
  (*(FAR volatile uint32_t *)((b) + (o)))
#define cmsdk_wdt_putreg(v, b, o) \
  (*((FAR volatile uint32_t *)((b) + (o))) = (v))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cmsdk_wdt_lowerhalf_s
{
  struct watchdog_lowerhalf_s lower;  /* Publicly visible portion */
  uintptr_t base;                     /* Register base address */
  int       irq;                      /* WDOGINT interrupt number */
  uint32_t  freq;                     /* WDOGCLK frequency (Hz) */
  uint32_t  timeout;                  /* Configured per-stage timeout (ms) */
  uint32_t  load;                     /* Per-stage reload value (ticks) */
  bool      started;                  /* True if the watchdog is running */
  xcpt_t    handler;                  /* Capture (early-warning) handler */
  spinlock_t lock;                    /* Serializes register seq + state
                                       * (SMP-safe; also masks local IRQs) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cmsdk_wdt_start(FAR struct watchdog_lowerhalf_s *lower);
static int cmsdk_wdt_stop(FAR struct watchdog_lowerhalf_s *lower);
static int cmsdk_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int cmsdk_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                               FAR struct watchdog_status_s *status);
static int cmsdk_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                uint32_t timeout);
static xcpt_t cmsdk_wdt_capture(FAR struct watchdog_lowerhalf_s *lower,
                                xcpt_t handler);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_cmsdk_wdt_ops =
{
  .start      = cmsdk_wdt_start,
  .stop       = cmsdk_wdt_stop,
  .keepalive  = cmsdk_wdt_keepalive,
  .getstatus  = cmsdk_wdt_getstatus,
  .settimeout = cmsdk_wdt_settimeout,
  .capture    = cmsdk_wdt_capture,
  .ioctl      = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void cmsdk_wdt_unlock(FAR struct cmsdk_wdt_lowerhalf_s *priv)
{
  cmsdk_wdt_putreg(CMSDK_WDT_UNLOCK, priv->base, CMSDK_WDT_LOCK);
}

static inline void cmsdk_wdt_lock(FAR struct cmsdk_wdt_lowerhalf_s *priv)
{
  cmsdk_wdt_putreg(0, priv->base, CMSDK_WDT_LOCK);
}

/****************************************************************************
 * Name: cmsdk_wdt_start_locked
 *
 * Description:
 *   Program the LOAD value and control register to (re)start the counter.
 *   The reset stage is enabled only when no capture handler is installed.
 *   This is the lock-free core of cmsdk_wdt_start(): the caller must hold
 *   priv->lock (it touches the shared WDOGLOCK register sequence and
 *   priv->started).
 *
 ****************************************************************************/

static void cmsdk_wdt_start_locked(FAR struct cmsdk_wdt_lowerhalf_s *priv)
{
  uint32_t control = CMSDK_WDT_CTRL_INTEN;

  if (priv->handler == NULL)
    {
      control |= CMSDK_WDT_CTRL_RESEN;
    }

  cmsdk_wdt_unlock(priv);
  cmsdk_wdt_putreg(priv->load, priv->base, CMSDK_WDT_LOAD);
  cmsdk_wdt_putreg(1, priv->base, CMSDK_WDT_INTCLR);
  cmsdk_wdt_putreg(control, priv->base, CMSDK_WDT_CONTROL);
  cmsdk_wdt_lock(priv);

  priv->started = true;
}

/****************************************************************************
 * Name: cmsdk_wdt_interrupt
 *
 * Description:
 *   Only fires when a capture (early-warning) handler is installed and the
 *   WDOGINT line is enabled at the interrupt controller.  In that mode we
 *   acknowledge the interrupt (which also reloads the counter, i.e. defers
 *   the reset stage) and invoke the user handler.  When no capture handler
 *   is installed the WDOGINT line stays masked at the controller and the
 *   hardware proceeds to the reset stage on the second expiry.
 *
 ****************************************************************************/

static int cmsdk_wdt_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cmsdk_wdt_lowerhalf_s *priv = arg;
  irqstate_t flags;
  xcpt_t handler;

  /* Serialize the WDOGLOCK register sequence and the handler read against
   * concurrent watchdog ops on other CPUs.
   */

  flags = spin_lock_irqsave(&priv->lock);

  /* Acknowledge: clears WDOGRIS.INT and reloads the counter. */

  cmsdk_wdt_unlock(priv);
  cmsdk_wdt_putreg(1, priv->base, CMSDK_WDT_INTCLR);
  cmsdk_wdt_lock(priv);

  handler = priv->handler;

  spin_unlock_irqrestore(&priv->lock, flags);

  /* Invoke the user handler outside the lock to avoid holding the spinlock
   * across arbitrary callback code (deadlock / latency).
   */

  if (handler != NULL)
    {
      handler(irq, context, arg);
    }

  return 0;
}

/****************************************************************************
 * Name: cmsdk_wdt_start
 ****************************************************************************/

static int cmsdk_wdt_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct cmsdk_wdt_lowerhalf_s *priv =
    (FAR struct cmsdk_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Arm the interrupt stage always.  Arm the reset stage only when no
   * capture handler is installed (WDIOC_CAPTURE means "call handler
   * instead of resetting").
   */

  flags = spin_lock_irqsave(&priv->lock);
  cmsdk_wdt_start_locked(priv);
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: cmsdk_wdt_stop
 ****************************************************************************/

static int cmsdk_wdt_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct cmsdk_wdt_lowerhalf_s *priv =
    (FAR struct cmsdk_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  cmsdk_wdt_unlock(priv);
  cmsdk_wdt_putreg(0, priv->base, CMSDK_WDT_CONTROL);
  cmsdk_wdt_lock(priv);

  priv->started = false;
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: cmsdk_wdt_keepalive
 ****************************************************************************/

static int cmsdk_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct cmsdk_wdt_lowerhalf_s *priv =
    (FAR struct cmsdk_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Writing INTCLR reloads the counter and clears any pending interrupt,
   * i.e. "pets the dog".
   */

  flags = spin_lock_irqsave(&priv->lock);
  cmsdk_wdt_unlock(priv);
  cmsdk_wdt_putreg(1, priv->base, CMSDK_WDT_INTCLR);
  cmsdk_wdt_lock(priv);
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: cmsdk_wdt_getstatus
 ****************************************************************************/

static int cmsdk_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                               FAR struct watchdog_status_s *status)
{
  FAR struct cmsdk_wdt_lowerhalf_s *priv =
    (FAR struct cmsdk_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t value;
  uint32_t timeout;
  bool started;
  bool capture;

  /* Take a consistent snapshot of the shared state under the lock, then do
   * the (64-bit divide) conversion outside it.
   */

  flags = spin_lock_irqsave(&priv->lock);
  started = priv->started;
  capture = (priv->handler != NULL);
  timeout = priv->timeout;
  value   = cmsdk_wdt_getreg(priv->base, CMSDK_WDT_VALUE);
  spin_unlock_irqrestore(&priv->lock, flags);

  status->flags = 0;
  if (started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  if (capture)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }
  else
    {
      status->flags |= WDFLAGS_RESET;
    }

  status->timeout = timeout;

  status->timeleft = (uint32_t)(((uint64_t)value * MSEC_PER_SEC) /
                                priv->freq);

  return OK;
}

/****************************************************************************
 * Name: cmsdk_wdt_settimeout
 ****************************************************************************/

static int cmsdk_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                uint32_t timeout)
{
  FAR struct cmsdk_wdt_lowerhalf_s *priv =
    (FAR struct cmsdk_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  uint64_t load;

  if (timeout == 0)
    {
      return -EINVAL;
    }

  /* Per-stage reload value.  The full time to reset spans two stages
   * (interrupt then reset), matching the CMSDK two-stage behaviour.
   */

  load = ((uint64_t)timeout * priv->freq) / MSEC_PER_SEC;
  if (load < 1)
    {
      load = 1;
    }
  else if (load > UINT32_MAX)
    {
      wderr("timeout %" PRIu32 "ms exceeds range at %" PRIu32 "Hz\n",
            timeout, priv->freq);
      load = UINT32_MAX;
    }

  flags = spin_lock_irqsave(&priv->lock);

  priv->timeout = timeout;
  priv->load    = (uint32_t)load;

  cmsdk_wdt_unlock(priv);
  cmsdk_wdt_putreg(priv->load, priv->base, CMSDK_WDT_LOAD);
  if (priv->started)
    {
      cmsdk_wdt_putreg(1, priv->base, CMSDK_WDT_INTCLR);
    }

  cmsdk_wdt_lock(priv);

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: cmsdk_wdt_capture
 ****************************************************************************/

static xcpt_t cmsdk_wdt_capture(FAR struct watchdog_lowerhalf_s *lower,
                                xcpt_t handler)
{
  FAR struct cmsdk_wdt_lowerhalf_s *priv =
    (FAR struct cmsdk_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  flags = spin_lock_irqsave(&priv->lock);

  oldhandler = priv->handler;
  priv->handler = handler;

  if (handler != NULL)
    {
      /* Deliver the early-warning interrupt to the CPU. */

      up_enable_irq(priv->irq);
    }
  else
    {
      /* Restore pure reset behaviour: keep the WDOGINT line masked. */

      up_disable_irq(priv->irq);
    }

  /* Re-arm control if already running so RESEN tracks the handler state.
   * Use the lock-free helper since we already hold priv->lock.
   */

  if (priv->started)
    {
      cmsdk_wdt_start_locked(priv);
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return oldhandler;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cmsdk_wdt_initialize(FAR const char *devpath, uintptr_t base,
                         int irq, uint32_t freq)
{
  FAR struct cmsdk_wdt_lowerhalf_s *priv;
  FAR void *handle;

  DEBUGASSERT(devpath != NULL && base != 0 && freq != 0);

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->lower.ops = &g_cmsdk_wdt_ops;
  priv->base      = base;
  priv->irq       = irq;
  priv->freq      = freq;
  priv->timeout   = 0;
  priv->load      = freq;   /* Default 1s per stage until settimeout() */
  priv->started   = false;
  priv->handler   = NULL;

  /* Explicitly initialize the spinlock per the API contract, rather than
   * relying on kmm_zalloc() having cleared it.
   */

  spin_lock_init(&priv->lock);

  /* Make sure the watchdog is stopped, then attach (but leave the WDOGINT
   * line masked; it is only unmasked when a capture handler is installed).
   */

  cmsdk_wdt_unlock(priv);
  cmsdk_wdt_putreg(0, priv->base, CMSDK_WDT_CONTROL);
  cmsdk_wdt_lock(priv);

  irq_attach(irq, cmsdk_wdt_interrupt, priv);
  up_disable_irq(irq);

  handle = watchdog_register(devpath, &priv->lower);
  if (handle == NULL)
    {
      irq_detach(irq);
      kmm_free(priv);
      return -EEXIST;
    }

  return OK;
}
