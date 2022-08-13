/****************************************************************************
 * arch/arm/src/kl/kl_timerisr.c
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

/* "The CLKSOURCE bit in SysTick Control and Status register selects either
 * the core clock (when CLKSOURCE = 1) or a divide-by-16 of the core clock
 * (when CLKSOURCE = 0). ..."
 */

#if defined(CONFIG_KL_SYSTICK_CORECLK)
#  define SYSTICK_CLOCK BOARD_CORECLK_FREQ        /* Core clock */
#elif defined(CONFIG_KL_SYSTICK_CORECLK_DIV16)
#  define (SYSTICK_CLOCK BOARD_CORECLK_FREQ / 16) /* Core clock divided by 16 */
#endif

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * Then, for example, if the external high speed crystal is the SysTick
 * clock source and BOARD_XTALHI_FREQUENCY is 12MHz and CLK_TCK is 100, then
 * the reload value would be:
 *
 *   SYSTICK_RELOAD = (12,000,000 / 100) - 1
 *                  = 119,999
 *                  = 0x1d4bf
 *
 * Which fits within the maximum 24-bit reload value.
 */

#define SYSTICK_RELOAD ((SYSTICK_CLOCK / CLK_TCK) - 1)

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
 * Function:  kl_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int kl_timerisr(int irq, uint32_t *regs, void *arg)
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

  /* Set the SysTick interrupt to the default priority */

  regval = getreg32(ARMV6M_SYSCON_SHPR3);
  regval &= ~SYSCON_SHPR3_PRI_15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << SYSCON_SHPR3_PRI_15_SHIFT);
  putreg32(regval, ARMV6M_SYSCON_SHPR3);

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, ARMV6M_SYSTICK_RVR);

  /* Attach the timer interrupt vector */

  irq_attach(KL_IRQ_SYSTICK, (xcpt_t)kl_timerisr, NULL);

  /* Enable SysTick interrupts.  "The CLKSOURCE bit in SysTick Control and
   * Status register selects either the core clock (when CLKSOURCE = 1) or
   * a divide-by-16 of the core clock (when CLKSOURCE = 0). ..."
   */

#ifdef CONFIG_KL_SYSTICK_CORECLK
  putreg32((SYSTICK_CSR_CLKSOURCE |
            SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE),
           ARMV6M_SYSTICK_CSR);
#else
  putreg32((SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE), ARMV6M_SYSTICK_CSR);
#endif

  /* And enable the timer interrupt */

  up_enable_irq(KL_IRQ_SYSTICK);
}
