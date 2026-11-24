/****************************************************************************
 * arch/arm/src/pp502x/pp502x_timerisr.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "clock/clock.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer 1 counts microseconds, so the period field is the tick less one */

#define PP502X_TICK_PERIOD  (CONFIG_USEC_PER_TICK - 1)
#define PP502X_TICK_CFG     (PP502X_TIMER_CFG_ENABLE | \
                             PP502X_TIMER_CFG_REPEAT | \
                             (PP502X_TICK_PERIOD & \
                              PP502X_TIMER_CFG_PERIOD_MASK))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pp502x_timerisr
 *
 * Description:
 *   The system timer interrupt handler.  Reading TIMER1_VAL acknowledges
 *   the interrupt.
 *
 ****************************************************************************/

static int pp502x_timerisr(int irq, uint32_t *regs, void *arg)
{
  getreg32(PP502X_TIMER1_VAL);

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Start Timer 1 as a periodic interrupt at CONFIG_USEC_PER_TICK.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  up_disable_irq(PP502X_IRQ_SYSTIMER);

  putreg32(0, PP502X_TIMER1_CFG);
  getreg32(PP502X_TIMER1_VAL);

  irq_attach(PP502X_IRQ_SYSTIMER, (xcpt_t)pp502x_timerisr, NULL);

  putreg32(PP502X_TICK_CFG, PP502X_TIMER1_CFG);

  up_enable_irq(PP502X_IRQ_SYSTIMER);
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Busy-wait on the free-running microsecond counter.  The unsigned
 *   difference stays correct across counter wrap.
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  uint32_t start = getreg32(PP502X_USEC_TIMER);

  while (getreg32(PP502X_USEC_TIMER) - start < microseconds)
    {
    }
}
