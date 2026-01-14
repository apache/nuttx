/****************************************************************************
 * arch/arm/src/armv7-r/arm_timer.c
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
#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <arch/barriers.h>

#include <sys/param.h>

#include "arm_timer.h"
#include "gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CNT_CTL_ENABLE_BIT      0
#define CNT_CTL_IMASK_BIT       1
#define CNT_CTL_ISTATUS_BIT     2

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
#  define GIC_IRQ_TIMER         GIC_IRQ_STM
#else
#  define GIC_IRQ_TIMER         GIC_IRQ_PTM
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void arm_timer_set_freq(uint32_t freq)
{
  CP15_SET(CNTFRQ, freq);
  UP_ISB();
}

static inline uint64_t arm_timer_phy_count(void)
{
  return CP15_GET64(CNTPCT);
}

static inline void arm_timer_phy_set_relative(uint32_t tval)
{
  CP15_SET(CNTP_TVAL, tval);
}

static inline void arm_timer_phy_set_absolute(uint64_t cval)
{
  CP15_SET64(CNTP_CVAL, cval);
}

static inline void arm_timer_phy_enable(bool enable)
{
  CP15_MODIFY((uint32_t)enable << CNT_CTL_ENABLE_BIT,
              BIT(CNT_CTL_ENABLE_BIT), CNTP_CTL);
  UP_ISB();
}

static inline void arm_timer_phy_set_irq_mask(bool mask)
{
  CP15_MODIFY((uint32_t)mask << CNT_CTL_IMASK_BIT,
              BIT(CNT_CTL_IMASK_BIT), CNTP_CTL);
  UP_ISB();
}

static int arm_timer_interrupt(int irq, void *regs, void *arg)
{
  struct oneshot_lowerhalf_s *priv = (struct oneshot_lowerhalf_s *)arg;
  arm_timer_phy_set_absolute(UINT64_MAX);
  oneshot_process_callback(priv);
  return OK;
}

static clkcnt_t arm_oneshot_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t arm_oneshot_current(struct oneshot_lowerhalf_s *lower)
{
  /* We do not need memory barrier here. */

  return arm_timer_phy_count();
}

static void arm_oneshot_start_absolute(struct oneshot_lowerhalf_s *lower,
                                       clkcnt_t expected)
{
  arm_timer_phy_set_absolute(expected);
}

static void arm_oneshot_start(struct oneshot_lowerhalf_s *lower,
                              clkcnt_t delta)
{
  arm_timer_phy_set_relative(MIN(UINT32_MAX, delta));
  arm_timer_phy_set_irq_mask(false);
}

static void arm_oneshot_cancel(struct oneshot_lowerhalf_s *lower)
{
  arm_timer_phy_set_absolute(UINT64_MAX);
}

static void arm_timer_initialize_per_cpu(unsigned int freq)
{
  /* Enable timer */

  if (freq)
    {
      arm_timer_set_freq(freq);
    }

  arm_timer_phy_set_absolute(UINT64_MAX);
  arm_timer_phy_enable(true);
  arm_timer_phy_set_irq_mask(false);

  up_enable_irq(GIC_IRQ_TIMER);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_arm_oneshot_ops =
{
  .current        = arm_oneshot_current,
  .start          = arm_oneshot_start,
  .start_absolute = arm_oneshot_start_absolute,
  .cancel         = arm_oneshot_cancel,
  .max_delay      = arm_oneshot_max_delay,
};

static struct oneshot_lowerhalf_s g_arm_oneshot_lowerhalf =
{
  .ops = &g_arm_oneshot_ops
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct oneshot_lowerhalf_s *arm_timer_initialize(unsigned int freq)
{
  struct oneshot_lowerhalf_s  *lower = &g_arm_oneshot_lowerhalf;

  /* The init freq is for trust-zone only since CNTFRQ is only
   * allowed to access in secure state.
   */

  arm_timer_initialize_per_cpu(freq);

  oneshot_count_init(lower, arm_timer_get_freq());

  irq_attach(GIC_IRQ_TIMER, arm_timer_interrupt, lower);

  return lower;
}

void arm_timer_secondary_init(unsigned int freq)
{
  arm_timer_initialize_per_cpu(freq);
}
