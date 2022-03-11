/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_timerisr.c
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

#include "clock/clock.h"
#include "mips_internal.h"
#include "pic32mz_config.h"
#include "hardware/pic32mz_timer.h"
#include "hardware/pic32mz_int.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer Setup **************************************************************/

/* Timer 1 is a type A timer.  Setting the TCS bit in the timer control
 * register will select the SOSC as the clock source.  Otherwise, PBCLK3
 * is the clock source.
 */

#ifdef CONFIG_PIC32MZ_T1_SOSC
#  define TIMER1_SRC_FREQ BOARD_SOSC_FREQ
#  define TIMER1_CON_TCS  TIMER_CON_TCS
#else
#  define TIMER1_SRC_FREQ BOARD_PBCLK3
#  define TIMER1_CON_TCS  (0)
#endif

/* Select a timer 1 prescale value.  Our goal is to select the timer MATCH
 * register value given the timer 1 input clock frequency and the desired
 * system timer frequency:
 *
 *   TIMER1_MATCH = TIMER1_SRC_FREQ / TIMER1_PRESCALE / CLOCKS_PER_SEC
 *
 * We want the largest possible value for MATCH that is less than 65,535, the
 * maximum value for the 16-bit timer register:
 *
 *   TIMER1_PRESCALE >= TIMER1_SRC_FREQ / CLOCKS_PER_SEC / 65535
 *
 * Timer 1 does not have very many options for the prescaler value.  So we
 * can pick the best by brute force.  Example:
 *
 * Example 1. Given:
 *   BOARD_TIMER1_SOSC      = Defined
 *   BOARD_SOSC_FREQ        = 32768
 *   CLOCKS_PER_SEC         = 100
 * Then:
 *   OPTIMAL_PRESCALE       = 1
 *   TIMER1_PRESCALE        = 1
 *   TIMER1_MATCH           = 327 -> 100.3 ticks/sec
 *
 * Example 2. Given:
 *   BOARD_TIMER1_SOSC      = Not defined
 *   BOARD_PBCLK3           = 60000000
 *   CLOCKS_PER_SEC         = 100
 * Then:
 *   OPTIMAL_PRESCALE       = 9.2
 *   TIMER1_PRESCALE        = 64
 *   TIMER1_MATCH           = 9375 -> 100.0 ticks/sec
 */

#define OPTIMAL_PRESCALE (TIMER1_SRC_FREQ / CLOCKS_PER_SEC / 65535)
#if OPTIMAL_PRESCALE <= 1
#  define TIMER1_CON_TCKPS    TIMER1_CON_TCKPS_1
#  define TIMER1_PRESCALE     1
#elif OPTIMAL_PRESCALE <= 8
#  define TIMER1_CON_TCKPS    TIMER1_CON_TCKPS_8
#  define TIMER1_PRESCALE     8
#elif OPTIMAL_PRESCALE <= 64
#  define TIMER1_CON_TCKPS    TIMER1_CON_TCKPS_64
#  define TIMER1_PRESCALE     64
#elif OPTIMAL_PRESCALE <= 256
#  define TIMER1_CON_TCKPS    TIMER1_CON_TCKPS_256
#  define TIMER1_PRESCALE     256
#else
#  error "This timer frequency cannot be represented"
#endif

#define TIMER1_MATCH (TIMER1_SRC_FREQ / TIMER1_PRESCALE / CLOCKS_PER_SEC)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  pc32mz_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int pc32mz_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear the pending timer interrupt */

  up_clrpend_irq(PIC32MZ_IRQ_T1);

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
  /* Configure and enable TIMER1.  Used the computed TCKPS divider and timer
   * match value.  The source will be either the internal PBCLOCK (TCS=0) or
   * the external SOSC (TCS=1)
   */

  putreg32((TIMER1_CON_TCKPS | TIMER1_CON_TCS), PIC32MZ_TIMER1_CON);
  putreg32(0, PIC32MZ_TIMER1_CNT);
  putreg32(TIMER1_MATCH - 1, PIC32MZ_TIMER1_PR);
  putreg32(TIMER_CON_ON, PIC32MZ_TIMER1_CONSET);

  /* Configure the timer interrupt */

  up_clrpend_irq(PIC32MZ_IRQ_T1);

  /* Attach the timer interrupt vector */

  irq_attach(PIC32MZ_IRQ_T1, (xcpt_t)pc32mz_timerisr, NULL);

  /* And enable the timer interrupt */

  up_enable_irq(PIC32MZ_IRQ_T1);
}
