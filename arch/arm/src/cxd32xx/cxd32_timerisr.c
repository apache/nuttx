/****************************************************************************
 * arch/arm/src/cxd32xx/cxd32_timerisr.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "clock/clock.h"
#include "arm_internal.h"
#include "hardware/cxd32_timer.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The Clock Source: Either the internal CCLK or external STCLK (P3.26) clock
 * as the source in the STCTRL register.  This file alwyays configures the
 * timer to use CCLK as its source.
 */

#define TICK_RELOAD ((CXD32_TIMER_BASEFREQ / CLK_TCK) - 1)

/* The size of the reload field is 24 bits.  Verify taht the reload value
 * will fit in the reload register.
 */

#if TICK_RELOAD > 0x00ffffff
#  error TICK_RELOAD exceeds the range of the RELOAD register
#endif

#define TIMER4_CH0            (CXD32_TIMER4_BASE+CXD32_TIMER_CH0_OFFSET)
#define TIMER4_CH1            (CXD32_TIMER4_BASE+CXD32_TIMER_CH1_OFFSET)
#define TIMER4_CH1_INITVALUE  0xFFFFFFFF

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  cxd32_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int cxd32_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  putreg32(TIMER_INTERRUPT, TIMER4_CH0 + CXD32_TIMER_INTCLR);

  nxsched_process_timer();

  return 0;
}

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

static void cxd32_timer0_initialize(void)
{
  uint32_t ctrl;

  /* Configure Tick to interrupt at the requested rate */

  putreg32(TICK_RELOAD, TIMER4_CH0 + CXD32_TIMER_LOAD);

  /* Attach the timer interrupt vector */

  (void)irq_attach(CXD32_IRQ_TIM41, (xcpt_t)cxd32_timerisr, NULL);

  /* Enable Timer */

  ctrl = (TIMERCTRL_ENABLE | TIMERCTRL_DIV_1 |
                        TIMERCTRL_SIZE_32BIT | TIMERCTRL_MODE_WRAP);
  ctrl |= (TIMERCTRL_PERIODIC | TIMERCTRL_INTENABLE);

  putreg32(ctrl, TIMER4_CH0 + CXD32_TIMER_CONTROL);

  /* And enable the timer interrupt */

  up_enable_irq(CXD32_IRQ_TIM41);
}

static void cxd32_timer1_initialize(void)
{
  uint32_t ctrl;

  /* Configure the coutner */

  putreg32(TIMER4_CH1_INITVALUE, TIMER4_CH1 + CXD32_TIMER_LOAD);

  /* Enable Timer */

  ctrl = (TIMERCTRL_ENABLE | TIMERCTRL_DIV_1 |
                        TIMERCTRL_SIZE_32BIT | TIMERCTRL_MODE_WRAP);
  ctrl |= (TIMERCTRL_PERIODIC);

  putreg32(ctrl, TIMER4_CH1 + CXD32_TIMER_CONTROL);
}

void up_timer_initialize(void)
{
  cxd32_timer0_initialize();
  cxd32_timer1_initialize();
}

/****************************************************************************
 * Function:  cxd32_timerget
 *
 * Description:
 *   This function returns the count of timer.
 *
 ****************************************************************************/

uint32_t cxd32_timerget(void)
{
  return getreg32(TIMER4_CH1 + CXD32_TIMER_VALUE);
}
