/****************************************************************************
 * arch/arm64/src/common/arm64_arch_timer.c
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

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <nuttx/spinlock.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_gic.h"
#include "arm64_arch_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MIN_DELAY  (1000)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t     last_cycle;
static uint64_t     cycle_per_tick;
static uint32_t     arch_timer_rate;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void arm64_arch_timer_set_compare(uint64_t value)
{
  write_sysreg(value, cntv_cval_el0);
}

static inline void arm64_arch_timer_enable(bool enable)
{
  uint64_t value;

  value = read_sysreg(cntv_ctl_el0);

  if (enable)
    {
      value |= CNTV_CTL_ENABLE_BIT;
    }
  else
    {
      value &= ~CNTV_CTL_ENABLE_BIT;
    }

  write_sysreg(value, cntv_ctl_el0);
}

static inline void arm64_arch_timer_set_irq_mask(bool mask)
{
  uint64_t value;

  value = read_sysreg(cntv_ctl_el0);

  if (mask)
    {
      value |= CNTV_CTL_IMASK_BIT;
    }
  else
    {
      value &= ~CNTV_CTL_IMASK_BIT;
    }

  write_sysreg(value, cntv_ctl_el0);
}

static inline uint64_t arm64_arch_timer_count(void)
{
  return read_sysreg(cntvct_el0);
}

static inline uint32_t arm64_arch_timer_get_cntfrq(void)
{
  return read_sysreg(cntfrq_el0);
}

#ifdef CONFIG_SCHED_TICKLESS
static int arm64_arch_timer_compare_isr(int irq, void *regs, void *arg)
{
  irqstate_t    flags;
  uint64_t      curr_cycle;
  uint32_t      delta_ticks;

  UNUSED(regs);
  UNUSED(arg);

  flags = spin_lock_irqsave(&g_arch_timer_lock);

  curr_cycle    = arm64_arch_timer_count();
  delta_ticks   = (uint32_t)((curr_cycle - last_cycle) / cycle_per_tick);

  last_cycle += delta_ticks * cycle_per_tick;

  arm_arch_timer_set_irq_mask(true);

  spin_unlock_irqrestore(&g_arch_timer_lock, flags);

  nxsched_process_timer();
  return OK;
}

#else

static int arm64_arch_timer_compare_isr(int irq, void *regs, void *arg)
{
  uint64_t      curr_cycle;
  uint32_t      delta_ticks;
  uint64_t      next_cycle;

  UNUSED(irq);
  UNUSED(regs);
  UNUSED(arg);

  curr_cycle    = arm64_arch_timer_count();
  delta_ticks   = (uint32_t)((curr_cycle - last_cycle) / cycle_per_tick);

  last_cycle += delta_ticks * cycle_per_tick;

  next_cycle = last_cycle + cycle_per_tick;

  if ((uint64_t)(next_cycle - curr_cycle) < MIN_DELAY)
    {
      next_cycle += cycle_per_tick;
    }

  arm64_arch_timer_set_compare(next_cycle);
  arm64_arch_timer_set_irq_mask(false);

  nxsched_process_timer();
  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SMP
/* Notes:
 *
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
 */

void arm64_smp_timer_init(void)
{
  uint64_t curr_cycle;

  /* set the initial status of timer0 of each secondary core */

  curr_cycle = arm64_arch_timer_count();

  arm64_arch_timer_set_compare(curr_cycle + cycle_per_tick);
  arm64_arch_timer_enable(true);
  up_enable_irq(ARM_ARCH_TIMER_IRQ);
  arm64_arch_timer_set_irq_mask(false);
}

#endif

uint64_t arm64_counter_read(void)
{
  return arm64_arch_timer_count();
}

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint64_t curr_cycle;

  arch_timer_rate   = arm64_arch_timer_get_cntfrq();
  cycle_per_tick    = ((uint64_t)arch_timer_rate / (uint64_t)TICK_PER_SEC);

  sinfo("%s: cp15 timer(s) running at %lu.%02luMHz, cycle %ld\n", __func__,
        (unsigned long)arch_timer_rate / 1000000,
        (unsigned long)(arch_timer_rate / 10000) % 100, cycle_per_tick);

  irq_attach(ARM_ARCH_TIMER_IRQ, arm64_arch_timer_compare_isr, 0);

  curr_cycle = arm64_arch_timer_count();
  arm64_arch_timer_set_compare(curr_cycle + cycle_per_tick);
  arm64_arch_timer_enable(true);

  up_enable_irq(ARM_ARCH_TIMER_IRQ);
  arm64_arch_timer_set_irq_mask(false);
}
