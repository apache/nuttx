/****************************************************************************
 * arch/rh850/src/u2c/u2c_ostm.c
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

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/oneshot.h>

#include "rh850_internal.h"
#include "hardware/u2c_ostm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The 32-bit counter is extended in software; a delay below half the
 * counter range keeps at least one counter read per wrap.
 */

#define OSTM_MAX_DELAY      0x7fffffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct u2c_ostm_s
{
  struct oneshot_lowerhalf_s lower;
  uintptr_t base;
  uint64_t  high;     /* Counter bits 63-32 */
  uint32_t  last;     /* Last counter value read */
  bool      armed;    /* A compare match is expected */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static clkcnt_t u2c_ostm_current(FAR struct oneshot_lowerhalf_s *lower);
static void u2c_ostm_start(FAR struct oneshot_lowerhalf_s *lower,
                           clkcnt_t delay);
static void u2c_ostm_start_absolute(FAR struct oneshot_lowerhalf_s *lower,
                                    clkcnt_t cnt);
static void u2c_ostm_cancel(FAR struct oneshot_lowerhalf_s *lower);
static clkcnt_t u2c_ostm_max_delay(FAR struct oneshot_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_u2c_ostm_ops =
{
  .current        = u2c_ostm_current,
  .start          = u2c_ostm_start,
  .start_absolute = u2c_ostm_start_absolute,
  .cancel         = u2c_ostm_cancel,
  .max_delay      = u2c_ostm_max_delay,
};

static struct u2c_ostm_s g_u2c_ostm0 =
{
  .lower.ops = &g_u2c_ostm_ops,
  .base      = U2C_OSTM0_BASE,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Called with interrupts disabled */

static uint64_t u2c_ostm_count(FAR struct u2c_ostm_s *priv)
{
  uint32_t cnt = getreg32(priv->base + OSTM_CNT_OFFSET);

  if (cnt < priv->last)
    {
      priv->high += (uint64_t)1 << 32;
    }

  priv->last = cnt;
  return priv->high | cnt;
}

static clkcnt_t u2c_ostm_current(FAR struct oneshot_lowerhalf_s *lower)
{
  FAR struct u2c_ostm_s *priv = (FAR struct u2c_ostm_s *)lower;
  irqstate_t flags;
  uint64_t cnt;

  flags = up_irq_save();
  cnt = u2c_ostm_count(priv);
  up_irq_restore(flags);
  return cnt;
}

/* A CMP that the counter has already passed waits a whole wrap: a deadline
 * reached by the time CMP is written raises the request in the INTC.
 */

static void u2c_ostm_start_absolute(FAR struct oneshot_lowerhalf_s *lower,
                                    clkcnt_t expected)
{
  FAR struct u2c_ostm_s *priv = (FAR struct u2c_ostm_s *)lower;
  irqstate_t flags;
  uint64_t now;

  flags = up_irq_save();

  now = u2c_ostm_count(priv);
  if (expected > now + OSTM_MAX_DELAY)
    {
      expected = now + OSTM_MAX_DELAY;
    }

  priv->armed = true;
  putreg32((uint32_t)expected, priv->base + OSTM_CMP_OFFSET);
  getreg32(priv->base + OSTM_CMP_OFFSET);

  if (u2c_ostm_count(priv) >= expected)
    {
      u2c_irq_raise(RH850_IRQ_OSTM0);
    }

  up_irq_restore(flags);
}

static void u2c_ostm_start(FAR struct oneshot_lowerhalf_s *lower,
                           clkcnt_t delay)
{
  u2c_ostm_start_absolute(lower, u2c_ostm_current(lower) + delay);
}

static void u2c_ostm_cancel(FAR struct oneshot_lowerhalf_s *lower)
{
  FAR struct u2c_ostm_s *priv = (FAR struct u2c_ostm_s *)lower;
  irqstate_t flags;

  flags = up_irq_save();
  priv->armed = false;
  u2c_irq_pending(RH850_IRQ_OSTM0);
  up_irq_restore(flags);
}

static clkcnt_t u2c_ostm_max_delay(FAR struct oneshot_lowerhalf_s *lower)
{
  return OSTM_MAX_DELAY;
}

static int u2c_ostm_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct u2c_ostm_s *priv = arg;

  if (priv->armed)
    {
      priv->armed = false;
      oneshot_process_callback(&priv->lower);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   OSTM0 in free-run compare mode as the oneshot timer of arch_alarm.  The
 *   module stop of OSTM0 is released by rh850_clockconfig().
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  FAR struct u2c_ostm_s *priv = &g_u2c_ostm0;

  putreg8(OSTM_TT_STOP, priv->base + OSTM_TT_OFFSET);
  putreg32(UINT32_MAX, priv->base + OSTM_CMP_OFFSET);
  putreg8(OSTM_CTL_IE | OSTM_CTL_MD1, priv->base + OSTM_CTL_OFFSET);
  putreg8(OSTM_TS_START, priv->base + OSTM_TS_OFFSET);

  oneshot_count_init(&priv->lower, CONFIG_RH850_CLK_HSB_FREQUENCY);

  u2c_irq_pending(RH850_IRQ_OSTM0);
  u2c_irq_setpriority(RH850_IRQ_OSTM0, CONFIG_RH850_OSTM0_PRIORITY);
  irq_attach(RH850_IRQ_OSTM0, u2c_ostm_interrupt, priv);
  up_enable_irq(RH850_IRQ_OSTM0);

  up_alarm_set_lowerhalf(&priv->lower);
}
