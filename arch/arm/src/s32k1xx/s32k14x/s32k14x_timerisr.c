/****************************************************************************
 * arch/arm/src/s32k1xx/s32k14x/s32k14x_timerisr.c
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
#include "arm_internal.h"
#include "clock/clock.h"
#include "s32k1xx_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SysTick clock input (Fsystick) is determined by the CLKSOURCE file of
 * the SysTick CSR register:  The CLKSOURCE field in SysTick Control and
 * Status register selects either the core clock (when CLKSOURCE = 1) or a
 * divide-by-16 of the core clock (when CLKSOURCE = 0).
 *
 * The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 *    reload = (Fsystick / CLK_TICK) - 1
 */

#define SYSTICK_RELOAD(coreclk) (((coreclk) / CLK_TCK) - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  s32k14x_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int s32k14x_timerisr(int irq, uint32_t *regs, void *arg)
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
  uint32_t coreclk;
  uint32_t reload;

  /* Make sure that the SYSTICK clock source is set to use the SysTick
   * function clock (CLKSOURCE==1).
   */

  putreg32(NVIC_SYSTICK_CTRL_CLKSOURCE, NVIC_SYSTICK_CTRL);

  /* Get the reload value */

  coreclk = s32k1xx_get_coreclk();
  reload  = SYSTICK_RELOAD(coreclk);

  /* The size of the reload field is 24 bits. */

  DEBUGASSERT(reload <= 0x00ffffff);

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(reload, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(S32K1XX_IRQ_SYSTICK, (xcpt_t)s32k14x_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(S32K1XX_IRQ_SYSTICK);
}
