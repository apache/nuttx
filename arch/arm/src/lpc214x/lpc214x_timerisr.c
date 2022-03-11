/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_timerisr.c
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
#include <arch/board/board.h>

#include "chip.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "lpc214x_timer.h"
#include "lpc214x_vic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The timers count at the rate of PCLK which is determined by PLL_M and
 * and APBDIV:
 */

#define LPC214X_CCLKFREQ  (LPC214X_FOSC*LPC214X_PLL_M)
#define LPC214X_PCLKFREQ  (LPC214X_CCLKFREQ/LPC214X_APB_DIV)

#define tmr_getreg8(o)    getreg8(LPC214X_TMR0_BASE+(o))
#define tmr_getreg16(o)   getreg16(LPC214X_TMR0_BASE+(o))
#define tmr_getreg32(o)   getreg32(LPC214X_TMR0_BASE+(o))

#define tmr_putreg8(v,o)  putreg8((v), LPC214X_TMR0_BASE+(o))
#define tmr_putreg16(v,o) putreg16((v), LPC214X_TMR0_BASE+(o))
#define tmr_putreg32(v,o) putreg32((v), LPC214X_TMR0_BASE+(o))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  lpc214x_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for
 *   various portions of the systems.
 *
 ****************************************************************************/

#ifdef CONFIG_VECTORED_INTERRUPTS
static int lpc214x_timerisr(uint32_t *regs)
#else
static int lpc214x_timerisr(int irq, uint32_t *regs, void *arg)
#endif
{
  /* Process timer interrupt */

  nxsched_process_timer();

  /* Clear the MR0 match interrupt */

  tmr_putreg8(LPC214X_TMR_IR_MR0I, LPC214X_TMR_IR_OFFSET);

  /* Reset the VIC as well */

#ifdef CONFIG_VECTORED_INTERRUPTS
  vic_putreg(0, LPC214X_VIC_VECTADDR_OFFSET);
#endif
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
  uint16_t mcr;

  /* Clear all match and capture event interrupts */

  tmr_putreg8(LPC214X_TMR_IR_ALLI, LPC214X_TMR_IR_OFFSET);

  /* Clear the timer counter */

  tmr_putreg32(0, LPC214X_TMR_TC_OFFSET);

  /* No pre-scaler */

  tmr_putreg32(0, LPC214X_TMR_PR_OFFSET);

  /* Set timer match registger to get a TICK_PER_SEC rate
   */

  tmr_putreg32(LPC214X_PCLKFREQ / TICK_PER_SEC, LPC214X_TMR_MR0_OFFSET);

  /* Reset timer counter register and interrupt on match */

  mcr = tmr_getreg16(LPC214X_TMR_MCR_OFFSET);
  mcr &= ~LPC214X_TMR_MCR_MR1I;
  mcr |= (LPC214X_TMR_MCR_MR0I | LPC214X_TMR_MCR_MR0R);
  tmr_putreg16(mcr, LPC214X_TMR_MCR_OFFSET);

  /* Enable counting */

  tmr_putreg8(LPC214X_TMR_CR_ENABLE, LPC214X_TMR_TCR_OFFSET);

  /* Attach the timer interrupt vector */

#ifdef CONFIG_VECTORED_INTERRUPTS
  up_attach_vector(LPC214X_IRQ_SYSTIMER, LPC214X_SYSTIMER_VEC,
                   (vic_vector_t)lpc214x_timerisr);
#else
  irq_attach(LPC214X_IRQ_SYSTIMER, (xcpt_t)lpc214x_timerisr, NULL);
#endif

  /* And enable the timer interrupt */

  up_enable_irq(LPC214X_IRQ_SYSTIMER);
}
