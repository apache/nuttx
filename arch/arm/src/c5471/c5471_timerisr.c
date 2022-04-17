/****************************************************************************
 * arch/arm/src/c5471/c5471_timerisr.c
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
#include <debug.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "clock/clock.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We want the general purpose timer running at the rate
 * USEC_PER_TICK. The C5471 clock is 47.5MHz and we're using
 * a timer PTV value of 3 (3 == divide incoming frequency by
 * 16) which then yields a 16 bitCLKS_PER_INT value
 * of 29687.
 *
 *   47500000 / 16 = 2968750 clocks/sec
 *   2968750 / 100 = 29687   clocks/ 100Hz interrupt
 *
 */

#define CLKS_PER_INT       29687
#define CLKS_PER_INT_SHIFT 5
#define AR                 0x00000010
#define ST                 0x00000008
#define PTV                0x00000003

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  c5471_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for
 *   various portions of the systems.
 *
 ****************************************************************************/

static int c5471_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t val;

  up_disable_irq(C5471_IRQ_SYSTIMER);

  /* Start the general purpose timer running in auto-reload mode
   * so that an interrupt is generated at the rate USEC_PER_TICK.
   */

  val = ((CLKS_PER_INT - 1) << CLKS_PER_INT_SHIFT) | AR | ST | PTV;
  putreg32(val, C5471_TIMER2_CTRL);

  /* Attach and enable the timer interrupt */

  irq_attach(C5471_IRQ_SYSTIMER, (xcpt_t)c5471_timerisr, NULL);
  up_enable_irq(C5471_IRQ_SYSTIMER);
}
