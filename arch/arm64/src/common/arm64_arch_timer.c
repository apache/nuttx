/****************************************************************************
 * arch/arm64/src/common/arm64_arch_timer.c
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
#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_alarm.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_gic.h"
#include "arm64_arch_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_ARM_TIMER_SECURE_IRQ         (GIC_PPI_INT_BASE + 13)
#define CONFIG_ARM_TIMER_NON_SECURE_IRQ     (GIC_PPI_INT_BASE + 14)
#define CONFIG_ARM_TIMER_VIRTUAL_IRQ        (GIC_PPI_INT_BASE + 11)
#define CONFIG_ARM_TIMER_HYP_IRQ            (GIC_PPI_INT_BASE + 10)

#define ARM_ARCH_TIMER_IRQ                  CONFIG_ARM_TIMER_VIRTUAL_IRQ
#define ARM_ARCH_TIMER_PRIO                 IRQ_DEFAULT_PRIORITY
#define ARM_ARCH_TIMER_FLAGS                IRQ_TYPE_LEVEL

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void arm64_arch_timer_set_compare(uint64_t value)
{
  write_sysreg(value, cntv_cval_el0);
}

static inline void arm64_arch_timer_set_relative(uint64_t value)
{
  write_sysreg(value, cntv_tval_el0);
}

static inline void arm64_arch_timer_enable(bool enable)
{
  modify_sysreg(enable ? CNTV_CTL_ENABLE_BIT : 0u,
                CNTV_CTL_ENABLE_BIT, cntv_ctl_el0);
}

static inline void arm64_arch_timer_set_irq_mask(bool mask)
{
  modify_sysreg(mask ? CNTV_CTL_IMASK_BIT : 0u,
                CNTV_CTL_IMASK_BIT, cntv_ctl_el0);
}

static inline uint64_t arm64_arch_timer_count(void)
{
  return read_sysreg(cntvct_el0);
}

static inline uint64_t arm64_arch_timer_get_cntfrq(void)
{
  return read_sysreg(cntfrq_el0);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_arch_timer_compare_isr
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

static int arm64_arch_timer_compare_isr(int irq, void *regs, void *arg)
{
  struct oneshot_lowerhalf_s *priv = (struct oneshot_lowerhalf_s *)arg;

  arm64_arch_timer_set_compare(UINT64_MAX);

  /* Then perform the callback */

  oneshot_process_callback(priv);

  return OK;
}

static clkcnt_t arm64_oneshot_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t arm64_oneshot_current(struct oneshot_lowerhalf_s *lower)
{
  /* We do not need memory barrier here. */

  return arm64_arch_timer_count();
}

static void arm64_oneshot_start_absolute(struct oneshot_lowerhalf_s *lower,
                                         clkcnt_t expected)
{
  arm64_arch_timer_set_compare(expected);
}

static void arm64_oneshot_start(struct oneshot_lowerhalf_s *lower,
                                clkcnt_t delta)
{
  arm64_arch_timer_set_relative(delta);
}

static void arm64_oneshot_cancel(struct oneshot_lowerhalf_s *lower)
{
  arm64_arch_timer_set_compare(UINT64_MAX);
}

/****************************************************************************
 * Name: arm64_oneshot_initialize_per_cpu
 *
 * Description:
 *   Initialize the ARM generic timer for secondary CPUs.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void arm64_oneshot_initialize_per_cpu(void)
{
  /* Enable int */

  up_enable_irq(ARM_ARCH_TIMER_IRQ);

  arm64_arch_timer_set_compare(UINT64_MAX);

  arm64_arch_timer_enable(true);

  arm64_arch_timer_set_irq_mask(false);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_arm64_oneshot_ops =
{
  .current        = arm64_oneshot_current,
  .start          = arm64_oneshot_start,
  .start_absolute = arm64_oneshot_start_absolute,
  .cancel         = arm64_oneshot_cancel,
  .max_delay      = arm64_oneshot_max_delay
};

static struct oneshot_lowerhalf_s g_arm64_oneshot_lowerhalf =
{
  .ops = &g_arm64_oneshot_ops
};

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

struct oneshot_lowerhalf_s *arm64_oneshot_initialize(void)
{
  struct oneshot_lowerhalf_s *priv = &g_arm64_oneshot_lowerhalf;
  uint64_t freq;

  tmrinfo("oneshot_initialize\n");

  /* Attach handler */

  irq_attach(ARM_ARCH_TIMER_IRQ,
             arm64_arch_timer_compare_isr, priv);

  arm64_oneshot_initialize_per_cpu();

  freq = arm64_arch_timer_get_cntfrq();

  DEBUGASSERT(freq <= UINT32_MAX);

  oneshot_count_init(priv, (uint32_t)freq);

  tmrinfo("oneshot_initialize ok %p \n", priv);

  return priv;
}

void up_timer_initialize(void)
{
  up_alarm_set_lowerhalf(arm64_oneshot_initialize());
}

void arm64_timer_secondary_init(void)
{
  arm64_oneshot_initialize_per_cpu();
}
