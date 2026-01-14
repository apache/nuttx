/****************************************************************************
 * arch/arm/src/armv8-r/arm_timer.c
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
#include <debug.h>
#include <assert.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <arch/barriers.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/oneshot.h>

#include "arm_timer.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CNT_CTL_ENABLE_BIT       0
#define CNT_CTL_IMASK_BIT        1
#define CNT_CTL_ISTATUS_BIT      2

#define GIC_IRQ_HYP_TIMER        26 /* Hypervisor Timer (HTM) IRQ */
#define GIC_IRQ_VIRT_TIMER       27 /* Virtual Timer (VTM) IRQ */
#define GIC_IRQ_SEC_PHY_TIMER    29 /* Secure Physical Timer IRQ */
#define GIC_IRQ_NONSEC_PHY_TIMER 30 /* Non-secure Physical Timer IRQ */

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
#  define GIC_IRQ_PHY_TIMER      GIC_IRQ_SEC_PHY_TIMER
#else
#  define GIC_IRQ_PHY_TIMER      GIC_IRQ_NONSEC_PHY_TIMER
#endif

#define ARM_ARCH_TIMER_IRQ       GIC_IRQ_PHY_TIMER
#define ARM_ARCH_TIMER_PRIO      IRQ_DEFAULT_PRIORITY
#define ARM_ARCH_TIMER_FLAGS     IRQ_TYPE_LEVEL

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

/****************************************************************************
 * Name: arm_oneshot_compare_isr
 *
 * Description:
 *   Common timer interrupt callback.  When any oneshot timer interrupt
 *   expires, this function will be called.  It will forward the call to
 *   the next level up.
 *
 * Input Parameters:
 *   oneshot - The state associated with the expired timer
 *
 * Returned Value:
 *   Always returns OK
 *
 ****************************************************************************/

static int arm_oneshot_compare_isr(int irq, void *regs, void *arg)
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
  arm_timer_phy_set_relative(delta <= UINT32_MAX ? delta : UINT32_MAX);
}

static void arm_oneshot_cancel(struct oneshot_lowerhalf_s *lower)
{
  arm_timer_phy_set_absolute(UINT64_MAX);
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
  .max_delay      = arm_oneshot_max_delay
};

static struct oneshot_lowerhalf_s g_arm_oneshot_lowerhalf =
{
  .ops = &g_arm_oneshot_ops
};

static void arm_oneshot_secondary_init(void)
{
  arm_timer_phy_set_absolute(UINT64_MAX);

  /* Enable interrupt */

  up_enable_irq(ARM_ARCH_TIMER_IRQ);
  arm_timer_phy_enable(true);
  arm_timer_phy_set_irq_mask(false);
}

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

static struct oneshot_lowerhalf_s *arm_oneshot_initialize(void)
{
  struct oneshot_lowerhalf_s *priv = &g_arm_oneshot_lowerhalf;
  uint64_t freq;

  /* Attach handler */

  irq_attach(ARM_ARCH_TIMER_IRQ, arm_oneshot_compare_isr, priv);

  freq = arm_timer_get_freq();

  DEBUGASSERT(freq <= UINT32_MAX);

  tmrinfo("%s: cp15 timer(s) running at %" PRIu64 ".%" PRIu64 "MHz\n",
          __func__, freq / 1000000, (freq / 10000) % 100);

  oneshot_count_init(priv, (uint32_t)freq);

  arm_oneshot_secondary_init();

  tmrinfo("oneshot_initialize ok %p \n", priv);

  return priv;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the system timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  up_alarm_set_lowerhalf(arm_oneshot_initialize());
}

#ifdef CONFIG_SMP
/****************************************************************************
 * Function: arm_timer_secondary_init
 *
 * Description:
 *   This function is called during start-up to initialize the system timer
 *   interrupt for smp.
 *
 * Notes:
 * The origin design for ARMv8-A timer is assigned private timer to
 * every PE(CPU core), the ARM_ARCH_TIMER_IRQ is a PPI so it's
 * should be enable at every core.
 *
 * But for NuttX, it's design only for primary core to handle timer
 * interrupt and call nxsched_process_timer at timer tick mode.
 * So we need only enable timer for primary core
 *
 * IMX6 use GPT which is a SPI rather than generic timer to handle
 * timer interrupt
 ****************************************************************************/

void arm_timer_secondary_init(unsigned int freq)
{
#ifdef CONFIG_SCHED_TICKLESS
  tmrinfo("arm_arch_timer_secondary_init\n");

  arm_oneshot_secondary_init();
#endif
}
#endif
