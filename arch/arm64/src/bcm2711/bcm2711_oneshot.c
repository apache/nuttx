/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_oneshot.c
 *
 * Contributed by: Matteo Golin <linguini@apache.org>
 *
 * SPDX-License-Identifer: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
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

/* This one-shot timer driver is based off of the BCM2711's system timer
 * interface, which provides four channels.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/arch.h>
#include <nuttx/config.h>

#include <nuttx/debug.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "arm64_arch.h"
#include "arm64_gic.h"

#include "bcm2711_oneshot.h"

#include "hardware/bcm2711_systimer.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcm2711_chan_s
{
  struct oneshot_lowerhalf_s dev;
  mutex_t lock; /* Exclusive access */
  clkcnt_t cmp; /* Full-size compare value */
  uint8_t chan; /* Which timer is being used 0-3 */
  bool init;    /* Initialized or not */
  bool running; /* Is the timer actively running or not */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static clkcnt_t bcm2711_current(FAR struct oneshot_lowerhalf_s *lower);
static void bcm2711_start(FAR struct oneshot_lowerhalf_s *lower,
                          clkcnt_t delay);
static void bcm2711_start_absolute(FAR struct oneshot_lowerhalf_s *lower,
                                   clkcnt_t cnt);
static void bcm2711_cancel(FAR struct oneshot_lowerhalf_s *lower);
static clkcnt_t bcm2711_max_delay(FAR struct oneshot_lowerhalf_s *lower);

static void bcm2711_tim_irq_en(struct bcm2711_chan_s *priv, bool en);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct oneshot_operations_s g_ops =
{
  .current = bcm2711_current,
  .start = bcm2711_start,
  .start_absolute = bcm2711_start_absolute,
  .cancel = bcm2711_cancel,
  .max_delay = bcm2711_max_delay,
};

static struct bcm2711_chan_s g_chans[BCM_SYST_NUMCHANS] =
{
  {
    .chan = 0,
    .dev =
      {
        .ops = &g_ops,
      },
    .init = false,
  },
  {
    .chan = 1,
    .dev =
      {
        .ops = &g_ops,
      },
    .init = false,
  },
  {
    .chan = 2,
    .dev =
      {
        .ops = &g_ops,
      },
    .init = false,
  },
  {
    .chan = 3,
    .dev =
      {
        .ops = &g_ops,
      },
    .init = false,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getcurrent
 *
 * Description:
 *   Get the current timer count value.
 *
 * Returned Value:
 *   The timer counter value.
 *
 ****************************************************************************/

static clkcnt_t get_current_count(void)
{
  clkcnt_t count;
  count = (clkcnt_t)getreg32(BCM_SYST_CHI) << 32;
  count |= getreg32(BCM_SYST_CLO);
  return count;
}

/****************************************************************************
 * Name: compare_exceeded
 *
 * Description:
 *   Check if the timer compare has been exceeded by the comparison that
 *   generated the last interrupt
 *
 * Input Parameters:
 *   cnt - Current timer count
 *   cmp - Full resolution comparison count
 *
 * Returned Value:
 *   True if the comparison has been exceeded (timer expired), false if not.
 *
 ****************************************************************************/

static bool compare_exceeded(clkcnt_t cnt, clkcnt_t cmp)
{
  /* We've passed the compare value, and we're under 32 bits different (i.e.
   * this is the first possible compare trigger)
   */

  if (cnt >= cmp && cnt - cmp < UINT32_MAX)
    {
      return true;
    }
  else
    {
      /* Our compare is larger than the count, which is possible when there
       * is a roll-over. Let's check if we're within the 32-bit compare
       * window.
       */

      return (cmp - cnt) < UINT32_MAX;
    }
}

/****************************************************************************
 * Name: bcm2711_tim_handler
 *
 * Description:
 *   Timer interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number
 *   ctx - The interrupt context
 *   arg - The handler argument (channel lower-half casted to void)
 *
 * Returned Value:
 *   0 on success, negated errno on error.
 *
 ****************************************************************************/

static int bcm2711_tim_handler(int irq, void *ctx, void *arg)
{
  struct bcm2711_chan_s *priv = (struct bcm2711_chan_s *)arg;
  clkcnt_t cur;
  irqstate_t flags;

  /* Check that we actually got a compare interrupt using the status bits. */

  if (!(getreg32(BCM_SYST_CS) & BCM_SYST_CS_M(priv->chan)))
    {
      return -EIO; /* Shouldn't have gotten an interrupt */
    }

  /* We have a match, which means the lower 32-bits of the counter match our
   * compare register. We compare the full 64 bit current count against our
   * full resolution compare value to see if we've exceed the point where the
   * timer should expire.
   */

  flags = enter_critical_section();
  cur = bcm2711_current(&priv->dev);
  leave_critical_section(flags);

  /* Trigger the callback, and disable our interrupt if we should fire.
   * Otherwise, keep ticking until it's time.
   */

  if (compare_exceeded(cur, priv->cmp))
    {
      priv->running = false;
      bcm2711_tim_irq_en(priv, false);
      tmrinfo("Callback...");
      priv->dev.callback(&priv->dev, priv->dev.arg);
      tmrinfo("Callback called for oneshot %u.", priv->chan);
    }

  /* Clear the interrupt status before leaving */

  putreg32(BCM_SYST_CS_M(priv->chan), BCM_SYST_CS);
  return 0;
}

/****************************************************************************
 * Name: bcm2711_tim_irq_en
 *
 * Description:
 *   Enable the timer's interrupt.
 *
 * Input Parameters:
 *   priv - The timer channel lower-half
 *   en   - True to enable, false to disable
 *
 ****************************************************************************/

static void bcm2711_tim_irq_en(struct bcm2711_chan_s *priv, bool en)
{
  if (en)
    {
      /* For safety, clear any match status in advance. We don't want a
       * spurious interrupt from something leftover.
       */

      putreg32(BCM_SYST_CS_M(priv->chan), BCM_SYST_CS);
      up_enable_irq(BCM_IRQ_VC_TIMER(priv->chan));
      tmrinfo("Interrupt for timer %u enabled.", priv->chan);
      return;
    }

  up_disable_irq(BCM_IRQ_VC_TIMER(priv->chan));
  tmrinfo("Interrupt for timer %u disabled.", priv->chan);
}

/****************************************************************************
 * Name: bcm2711_tim_irqinit
 *
 * Description:
 *   The interrupt handler used for the timer interrupt.
 *
 * Input Parameters:
 *   priv - The lower-half timer driver
 *
 * Returned Value:
 *   0 on success, negated errno on failure.
 *
 ****************************************************************************/

static int bcm2711_tim_irqinit(struct bcm2711_chan_s *priv)
{
  int err;

  err = irq_attach(BCM_IRQ_VC_TIMER(priv->chan), bcm2711_tim_handler, priv);
  if (err < 0)
    {
      tmrerr("Couldn't attach interrupt for timer %u: %d", priv->chan, err);
      return err;
    }

  tmrinfo("Interrupt handler attached for timer %u", priv->chan);

#ifdef CONFIG_ARCH_IRQPRIO
  err = up_prioritize_irq(BCM_IRQ_VC_TIMER(priv->chan), 0);
  if (err < 0)
    {
      tmrerr("Failed to prioritize timer %u interrupt", priv->chan);
    }
#endif

  err = up_set_irq_type(BCM_IRQ_VC_TIMER(priv->chan), IRQ_RISING_EDGE);
  if (err < 0)
    {
      tmrerr("Failed to timer %u IRQ type", priv->chan);
    }

  return err;
}

/****************************************************************************
 * Name: bcm2711_current
 *
 * Description:
 *   Get the current timer count.
 *
 * Input Parameters:
 *   lower - The lower-half driver instance
 *
 * Returned Value:
 *   The current timer count.
 *
 ****************************************************************************/

static clkcnt_t bcm2711_current(FAR struct oneshot_lowerhalf_s *lower)
{
  UNUSED(lower);
  clkcnt_t count;
  irqstate_t flags;

  /* We have to perform two register reads; one for high bits, one for low
   * bits. We want to make sure we're not interrupted here and lose sync.
   */

  flags = enter_critical_section();
  count = get_current_count();
  leave_critical_section(flags);

  return count;
}

/****************************************************************************
 * Name: bcm2711_start
 *
 * Description:
 *   Start a relative timer.
 *
 * Input Parameters:
 *   lower - The lower-half driver instance
 *   delay - The relative delay from the current
 *
 ****************************************************************************/

static void bcm2711_start(FAR struct oneshot_lowerhalf_s *lower,
                          clkcnt_t delay)
{
  struct bcm2711_chan_s *priv = (struct bcm2711_chan_s *)lower;
  irqstate_t flags;
  clkcnt_t cnt;

  /* Compute the absolute time with the offset. Set the resulting absolute
   * timer.
   */

  nxmutex_lock(&priv->lock);
  if (priv->running)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  flags = enter_critical_section();
  cnt = get_current_count() + delay;
  priv->cmp = cnt;
  leave_critical_section(flags);

  priv->running = true;
  putreg32((uint32_t)(cnt & UINT32_MAX), BCM_SYST_C(priv->chan));
  bcm2711_tim_irq_en(priv, true); /* Enable interrupts */
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: bcm2711_start_absolute
 *
 * Description:
 *   Start an absolute timer.
 *
 * Input Parameters:
 *   lower - The lower-half driver instance
 *   cnt - The absolute count for the timer to go off
 *
 ****************************************************************************/

static void bcm2711_start_absolute(FAR struct oneshot_lowerhalf_s *lower,
                                   clkcnt_t cnt)
{
  struct bcm2711_chan_s *priv = (struct bcm2711_chan_s *)lower;

  nxmutex_lock(&priv->lock);
  if (priv->running)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  priv->running = true;
  priv->cmp = cnt;
  putreg32((uint32_t)(cnt & UINT32_MAX), BCM_SYST_C(priv->chan));
  bcm2711_tim_irq_en(priv, true); /* Enable interrupts */
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: bcm2711_cancel
 *
 * Description:
 *   Cancel the timer event
 *
 * Input Parameters:
 *   lower - The lower-half driver instance
 *
 ****************************************************************************/

static void bcm2711_cancel(FAR struct oneshot_lowerhalf_s *lower)
{
  struct bcm2711_chan_s *priv = (struct bcm2711_chan_s *)lower;

  /* Disable the timer's interrupt handler, which will stop events from
   * happening for it.
   */

  nxmutex_lock(&priv->lock);
  priv->running = false;
  bcm2711_tim_irq_en((struct bcm2711_chan_s *)lower, false);
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: bcm2711_max_delay
 *
 * Description:
 *   Get the timer's maximum delay.
 *
 * Input Parameters:
 *   lower - The lower-half driver instance
 *
 * Returned Value:
 *   The timer's maximum delay.
 *
 ****************************************************************************/

static clkcnt_t bcm2711_max_delay(FAR struct oneshot_lowerhalf_s *lower)
{
  UNUSED(lower);
  return UINT64_MAX;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower half driver
 *   instance.
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned.  NULL is return on any failure.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *oneshot_initialize(int chan, uint16_t resolution)
{
  int err;

  /* Channels in the range 0-3 */

  if (chan < 0 || chan > BCM_SYST_NUMCHANS - 1)
    {
      tmrerr("Timer %d does not exist.", chan);
      return NULL;
    }

  /* The system has a minimum resolution of 1us. We otherwise ignore the
   * argument, because we will operate in a resolution of 1us always.
   */

  if (resolution < 1)
    {
      tmrerr("Resolution of %u uS is too small.", resolution);
      return NULL;
    }

  /* Channel initialized already */

  if (g_chans[chan].init)
    {
      return &g_chans[chan].dev;
    }

  g_chans[chan].chan = chan;
  g_chans[chan].running = false;
  nxmutex_init(&g_chans[chan].lock);

  err = bcm2711_tim_irqinit(&g_chans[chan]);
  if (err < 0)
    {
      return NULL;
    }

  /* Tell the upper-half how to perform the timing conversion using the clock
   * frequency. The system timer runs at 1MHz.
   */

  oneshot_count_init(&g_chans[chan].dev, BCM_SYST_FREQ);

  g_chans[chan].init = true;
  return &g_chans[chan].dev;
}
