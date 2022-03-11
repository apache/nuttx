/****************************************************************************
 * arch/arm/src/am335x/am335x_timerisr.c
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

#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/am335x_timer.h"

#include "am335x_sysclk.h"

#define USE_TIMER1MS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer clock selects system clock CLK_M_OSC */

#  define TMR_CLOCK             ((int64_t)am335x_get_sysclk())

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * Timer 1 counts down from the interval reload value to zero, generating
 * an interrupt (and reload) when the counts decrements to zero.
 */

#define TMR_TLDR                (0xffffffff - (TMR_CLOCK / CLK_TCK) + 1)
#define TMR_TCRR                (0xffffffff - (TMR_CLOCK / CLK_TCK) + 1)

#define TMR_TPIR \
    (((TMR_CLOCK / CLK_TCK + 1) * 1000000ll) - \
     (TMR_CLOCK * (1000000ll / CLK_TCK)))
#define TMR_TNIR \
    (((TMR_CLOCK / CLK_TCK) * 1000000ll) - \
     (TMR_CLOCK * (1000000ll / CLK_TCK)))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  am335x_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int am335x_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear the pending interrupt by writing a '1' to the status register */

#ifdef USE_TIMER1MS
  putreg32(TMR1MS_IRQ_FLAG_OVF, AM335X_TMR1MS_TISR);
#else
  putreg32(TMR_IRQ_FLAG_OVF, AM335X_TMR2_IRQ_STAT);
#endif

  /* Process timer interrupt */

  nxsched_process_timer();

  return OK;
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

#ifdef USE_TIMER1MS
  /* Make sure that interrupts from the Timer 1 are disabled */

  up_disable_irq(AM335X_IRQ_TIMER1_1MS);

  /* Soft reset the timer */

  putreg32(TMR1MS_TIOCP_SOFT_RESET, AM335X_TMR1MS_TIOCP_CFG);

  while (!(getreg32(AM335X_TMR1MS_TISTAT) & TMR1MS_TISTAT))
    {
    }

  putreg32(TMR_TPIR, AM335X_TMR1MS_TPIR);
  putreg32(TMR_TNIR, AM335X_TMR1MS_TNIR);
  putreg32(TMR_TLDR, AM335X_TMR1MS_TLDR);
  putreg32(TMR_TCRR, AM335X_TMR1MS_TCRR);

  /* Setup auto-reload and start timer */

  regval = TMR1MS_TCLR_AR | TMR1MS_TCLR_ST;
  putreg32(regval, AM335X_TMR1MS_TCLR);

  /* Attach the timer interrupt vector */

  irq_attach(AM335X_IRQ_TIMER1_1MS, (xcpt_t)am335x_timerisr, NULL);

  /* Clear interrupt status */

  regval = TMR1MS_IRQ_FLAG_MAT | TMR1MS_IRQ_FLAG_OVF |
           TMR1MS_IRQ_FLAG_TCAR;
  putreg32(regval, AM335X_TMR1MS_TISR);

  /* Enable overflow interrupt */

  putreg32(TMR1MS_IRQ_FLAG_OVF, AM335X_TMR1MS_TIER);

  /* And enable the timer interrupt */

  up_enable_irq(AM335X_IRQ_TIMER1_1MS);

#else
  /* Make sure that interrupts from the Timer 2 are disabled */

  up_disable_irq(AM335X_IRQ_TIMER2);

  /* Soft reset the timer */

  putreg32(TMR_TIOCP_SOFT_RESET, AM335X_TMR2_TIOCP_CFG);

  while ((getreg32(AM335X_TMR2_TIOCP_CFG) & TMR_TIOCP_SOFT_RESET))
    {
    }

  putreg32(TMR_TLDR, AM335X_TMR2_TLDR);
  putreg32(TMR_TCRR, AM335X_TMR2_TCRR);

  /* Setup auto-reload and start timer */

  regval = TMR_TCLR_AR | TMR_TCLR_ST;
  putreg32(regval, AM335X_TMR2_TCLR);

  /* Attach the timer interrupt vector */

  irq_attach(AM335X_IRQ_TIMER2, (xcpt_t)am335x_timerisr, NULL);

  /* Enable overflow interrupt */

  putreg32(TMR_IRQ_FLAG_OVF, AM335X_TMR2_IRQ_EN_SET);

  /* And enable the timer interrupt */

  up_enable_irq(AM335X_IRQ_TIMER2);
#endif
}
