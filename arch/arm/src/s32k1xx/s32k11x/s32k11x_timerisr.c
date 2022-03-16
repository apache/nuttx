/****************************************************************************
 * arch/arm/src/s32k1xx/s32k11x/s32k11x_timerisr.c
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

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The CLKSOURCE field in SysTick Control and Status register selects either
 * the core clock (when CLKSOURCE = 1) or a divide-by-16 of the core clock
 * (when CLKSOURCE = 0).

 * Then, for example, if the core clock is the SysTick close source and
 * the core clock is 48MHz and CLK_TCK is 100, then the reload value would
 * be:
 *
 *   SYSTICK_RELOAD = (48,000,000 / 100) - 1
 *                  = 479,999
 *                  = 0x752ff
 *
 * Which fits within the maximum 24-bit reload value.
 */

#define SYSTICK_RELOAD(coreclk) (((coreclk) / CLK_TCK) - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  s32k11x_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int s32k11x_timerisr(int irq, uint32_t *regs, void *arg)
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
  uint32_t regval;

  /* Set the SysTick interrupt to the default priority */

  regval  = getreg32(ARMV6M_SYSCON_SHPR3);
  regval &= ~SYSCON_SHPR3_PRI_15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << SYSCON_SHPR3_PRI_15_SHIFT);
  putreg32(regval, ARMV6M_SYSCON_SHPR3);

  /* Set set CSR CLKSOURCE bit to select the core clock as the SysTick
   * source clock.
   */

  putreg32(SYSTICK_CSR_CLKSOURCE, ARMV6M_SYSTICK_CSR);

  /* Get the reload value */

  coreclk = s32k1xx_get_coreclk();
  reload  = SYSTICK_RELOAD(coreclk);

  /* The size of the reload field is 24 bits. */

  DEBUGASSERT(reload <= 0x00ffffff);

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(reload, ARMV6M_SYSTICK_RVR);

  /* Attach the timer interrupt vector */

  irq_attach(S32K1XX_IRQ_SYSTICK, (xcpt_t)s32k11x_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE |
            SYSTICK_CSR_CLKSOURCE),
           ARMV6M_SYSTICK_CSR);

  /* And enable the timer interrupt */

  up_enable_irq(S32K1XX_IRQ_SYSTICK);
}
