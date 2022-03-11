/****************************************************************************
 * arch/arm/src/max326xx/common/max326_timerisr.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "max326_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SysTick clock will may be clocked internally the processor clock
 * (CLKSOURCE==1).  The SysTick Function clock is equal to:
 *
 *   Fsystick = Fcpu / SYSTICKCLKDIV
 *
 * The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 *    reload = (Fsystick / CLK_TICK) - 1
 *
 * The resulting reload value should be as large as possible, but must be
 * less than 2^24:
 *
 *   SYSTICKDIV > Fcpu / CLK_TCK / 2^24
 */

#define SYSTICK_RELOAD ((max326_cpu_frequency() / CLK_TCK) - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  max326_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int max326_timerisr(int irq, uint32_t *regs, void *arg)
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
  uint32_t regval;

  /* Make sure that the SYSTICK clock source is set to use the CPU clock
   * (CLKSOURCE==1).
   *
   * REVISIT:  This is duplicate logic.  The same setting is make below.
   */

  regval  = getreg32(NVIC_SYSTICK_CTRL);
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* Configure SysTick to interrupt at the requested rate */

  regval = SYSTICK_RELOAD;
  DEBUGASSERT(regval < 0x01000000);

  putreg32(regval, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(MAX326_IRQ_SYSTICK, (xcpt_t)max326_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(MAX326_IRQ_SYSTICK);
}
