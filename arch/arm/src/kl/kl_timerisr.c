/****************************************************************************
 * arch/arm/src/kl/kl_timerisr.c
 *
 *   Copyright (C) 2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "arm_arch.h"

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

static int kl_timerisr(int irq, uint32_t *regs, FAR void *arg)
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
  putreg32((SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE),
           ARMV6M_SYSTICK_CSR);
#else
  putreg32((SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE), ARMV6M_SYSTICK_CSR);
#endif

  /* And enable the timer interrupt */

  up_enable_irq(KL_IRQ_SYSTICK);
}
