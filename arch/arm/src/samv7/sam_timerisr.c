/****************************************************************************
 * arch/arm/src/samv7/sam_timerisr.c
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

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select MCU-specific settings
 *
 * The SysTick timer is driven by the output of the Mast Clock Controller
 * prescaler output (i.e., the MDIV output divider is not applied so that
 * the driving frequency is the same as the CPU frequency).
 *
 * The SysTick calibration value is fixed to 37500 which allows the
 * generation of a time base of 1 ms with SysTick clock to the maximum
 * frequency on MCK divided by 8. (?)
 */

#define SAM_SYSTICK_CLOCK  BOARD_CPU_FREQUENCY

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 */

#define SYSTICK_RELOAD ((SAM_SYSTICK_CLOCK / CLK_TCK) - 1)

/* The size of the reload field is 24 bits.  Verify that the reload value
 * will fit in the reload register.
 */

#if SYSTICK_RELOAD > 0x00ffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sam_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int sam_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_CLOCK_ADJTIME

/****************************************************************************
 * Function:  up_adj_timer_period
 *
 * Description:
 *   Adjusts timer period. This call is used when adjusting timer period as
 *   defined in adjtime() function.
 *
 * Input Parameters:
 *   period_inc_usec  - period adjustment in usec (reset to default value
 *                      if 0)
 *
 ****************************************************************************/

void up_adj_timer_period(long long period_inc_usec)
{
  uint32_t period;
  long long period_inc;

  if (period_inc_usec == 0)
    {
      period_inc = 0;
    }
  else
    {
      period_inc = (SAM_SYSTICK_CLOCK / 1000000) * period_inc_usec - 1;
    }

  period = SYSTICK_RELOAD + period_inc;

  /* Check whether period is at maximum value. */

  if (period > 0x00ffffff)
    {
      period = 0x00ffffff;
    }

  putreg32(period, NVIC_SYSTICK_RELOAD);
}

/****************************************************************************
 * Function:  up_get_timer_period
 *
 * Description:
 *   This function returns the timer period in usec.
 *
 * Input Parameters:
 *   period_usec  - returned timer period in usec
 *
 ****************************************************************************/

void up_get_timer_period(long long *period_usec)
{
  uint32_t period;

  period = getreg32(NVIC_SYSTICK_RELOAD);

  *period_usec = ((period + 1) / (SAM_SYSTICK_CLOCK / 1000000));
}

#endif

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Attach the timer interrupt vector */

  irq_attach(SAM_IRQ_SYSTICK, (xcpt_t)sam_timerisr, NULL);

  /* Enable SysTick interrupts (no divide-by-8) */

  regval = (NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE);
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(SAM_IRQ_SYSTICK);
}
