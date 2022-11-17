/****************************************************************************
 * arch/sparc/src/s698pm/s698pm-timerisr.c
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
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "sparc_internal.h"
#include "s698pm.h"
#include "s698pm_tim.h"
#include "s698pm_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The CPU frequency is given by BOARD_CPU_CLOCK (defined in board.h).  The
 * desired interrupt frequency is given by CONFIG_USEC_PER_TICK.  An unscaled
 * ideal match is given by:
 *
 *   CLOCK = CPU_CLOCK / DIVISOR                      # CPU clocks/sec
 *   MATCH = CLOCK / CLOCKS_PER_SEC                   # CPU clocks/timer tick
 *   MATCH = CPU_CLOCK / DIVISOR / CLOCKS_PER_SEC     # CPU clocks/timer tick
 *
 * But we only have 16-bits of accuracy so we need to pick the smallest
 * divisor using the following brute force calculation:
 */

#define S698PM_TIMER_CLOCK                  1000000
#define MATCH1                              (( 1000000 / CLOCKS_PER_SEC) - 1)

/* Bit 0: enables the timer when set */

#define TIMCTR_ENABLE_COUNTER               (1 << 0)

/* Bit 1: automatically reloaded with the reload value after each underflow */

#define TIMCTR_AUTO_RELOAD            	    (1 << 1)

/* Bit 2: Set 1, will load the timer reload register into the timer counter
 * register
 */

#define TIMCTR_LOAD_COUNTER                 (1 << 2)

/* Bit 3: Set 1, will triger underflow interrupt each underflow */

#define TIMCTR_ENABLE_INT                   (1 << 3)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  s698pm_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int s698pm_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear the pending timer interrupt */

  sparc_clrpend_irq(S698PM_IRQ_TIMER1);

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
 *   This function is called during start-up to initialize the timer
 *   interrupt.  NOTE:  This function depends on setup of OSC32 by
 *   up_clkinitialize().
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

  regval = (BOARD_CPU_CLOCK / S698PM_TIMER_CLOCK) - 1;

  putreg16(regval, S698PM_TIMPRE_BASE + S698PM_TIM_PSCLOAD_OFFSET);

  putreg16(regval, S698PM_TIMPRE_BASE + S698PM_TIM_PSCCONT_OFFSET);

  /* Setup timer 1 compare match A to generate a tick interrupt.
   *
   * First, setup the match value for compare match A.
   */

  putreg32(MATCH1, S698PM_TIM1_BASE + S698PM_TIM_CNT_OFFSET);
  putreg32(MATCH1, S698PM_TIM1_BASE + S698PM_TIM_ARR_OFFSET);

  regval = (TIMCTR_ENABLE_COUNTER | TIMCTR_AUTO_RELOAD |
            TIMCTR_LOAD_COUNTER | TIMCTR_ENABLE_INT);
  putreg32(regval, S698PM_TIM1_BASE + S698PM_TIM_CR_OFFSET);

  /* Configure the timer interrupt */

  sparc_clrpend_irq(S698PM_IRQ_TIMER1);

  /* Attach the timer interrupt vector */

  irq_attach(S698PM_IRQ_TIMER1, (xcpt_t)s698pm_timerisr, NULL);

  /* Set up to timer1 interrupts on the current CPU */

#ifdef CONFIG_ARCH_IRQPRIO
  (void)s698pm_setup_irq(0, S698PM_IRQ_TIMER1, CONFIG_S698PM_TIMER1PRIO);
#else
  (void)s698pm_setup_irq(0, S698PM_IRQ_TIMER1, 0);
#endif

  /* And enable the timer interrupt */

  up_enable_irq(S698PM_IRQ_TIMER1);
}
