/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_timerisr.c
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
#include "hardware/nuc_clk.h"
#include "hardware/nuc_gcr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get the frequency of the selected clock source */

#if defined(CONFIG_NUC_SYSTICK_CORECLK)
#  define SYSTICK_CLOCK BOARD_HCLK_FREQUENCY       /* Core clock */
#elif defined(CONFIG_NUC_SYSTICK_XTALHI)
#  define SYSTICK_CLOCK BOARD_XTALHI_FREQUENCY     /* High speed XTAL clock */
#elif defined(CONFIG_NUC_SYSTICK_XTALLO)
#  define SYSTICK_CLOCK BOARD_XTALLO_FREQUENCY     /* Low speed XTAL clock */
#elif defined(CONFIG_NUC_SYSTICK_XTALHId2)
#  define SYSTICK_CLOCK (BOARD_XTALHI_FREQUENCY/2) /* High speed XTAL clock/2 */
#elif defined(CONFIG_NUC_SYSTICK_HCLKd2)
#  define SYSTICK_CLOCK (BOARD_HCLK_FREQUENCY/2)   /* HCLK/2 */
#elif defined(CONFIG_NUC_SYSTICK_INTHId2)
#  define SYSTICK_CLOCK (NUC_INTHI_FREQUENCY/2)    /* Internal high speed clock/2 */
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
 * Name: nuc_unlock
 *
 * Description:
 *   Unlock registers
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NUC_SYSTICK_CORECLK
static inline void nuc_unlock(void)
{
  putreg32(0x59, NUC_GCR_REGWRPROT);
  putreg32(0x16, NUC_GCR_REGWRPROT);
  putreg32(0x88, NUC_GCR_REGWRPROT);
}
#endif

/****************************************************************************
 * Name: nuclock
 *
 * Description:
 *   Lok registers
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NUC_SYSTICK_CORECLK
static inline void nuc_lock(void)
{
  putreg32(0, NUC_GCR_REGWRPROT);
}
#endif

/****************************************************************************
 * Function:  nuc_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int nuc_timerisr(int irq, uint32_t *regs, void *arg)
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

  /* Configure the SysTick clock source. This is only necessary if we are not
   * using the Cortex-M0 core clock as the frequency source.
   */

#ifndef CONFIG_NUC_SYSTICK_CORECLK

  /* This field is write protected and must be unlocked */

  nuc_unlock();

  /* Read the CLKSEL0 register and set the STCLK_S field appropriately */

  regval  = getreg32(NUC_CLK_CLKSEL0);
  regval &= ~CLK_CLKSEL0_STCLK_S_MASK;
#if defined(CONFIG_NUC_SYSTICK_XTALHI)
  regval |= CLK_CLKSEL0_STCLK_S_XTALHI;   /* High speed XTAL clock */
#elif defined(CONFIG_NUC_SYSTICK_XTALLO)
  regval |= CLK_CLKSEL0_STCLK_S_XTALLO;   /* Low speed XTAL clock */
#elif defined(CONFIG_NUC_SYSTICK_XTALHId2)
  regval |= CLK_CLKSEL0_STCLK_S_XTALDIV2; /* High speed XTAL clock/2 */
#elif defined(CONFIG_NUC_SYSTICK_HCLKd2)
  regval |= CLK_CLKSEL0_STCLK_S_HCLKDIV2; /* HCLK/2 */
#elif defined(CONFIG_NUC_SYSTICK_INTHId2)
  regval |= CLK_CLKSEL0_STCLK_S_INTDIV2;  /* Internal high speed clock/2 */
#endif
  putreg32(regval, NUC_CLK_CLKSEL0);

  /* Re-lock the register */

  nuc_lock();
#endif

  /* Set the SysTick interrupt to the default priority */

  regval = getreg32(ARMV6M_SYSCON_SHPR3);
  regval &= ~SYSCON_SHPR3_PRI_15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << SYSCON_SHPR3_PRI_15_SHIFT);
  putreg32(regval, ARMV6M_SYSCON_SHPR3);

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, ARMV6M_SYSTICK_RVR);

  /* Attach the timer interrupt vector */

  irq_attach(NUC_IRQ_SYSTICK, (xcpt_t)nuc_timerisr, NULL);

  /* Enable SysTick interrupts.  We need to select the core clock here if
   * we are not using one of the alternative clock sources above.
   */

#ifdef CONFIG_NUC_SYSTICK_CORECLK
  putreg32((SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT |
            SYSTICK_CSR_ENABLE),
            ARMV6M_SYSTICK_CSR);
#else
  putreg32((SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE), ARMV6M_SYSTICK_CSR);
#endif

  /* And enable the timer interrupt */

  up_enable_irq(NUC_IRQ_SYSTICK);
}
