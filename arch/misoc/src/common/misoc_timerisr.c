/****************************************************************************
 * arch/misoc/src/common/misoc_timerisr.c
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
#include <nuttx/irq.h>
#include <arch/board/board.h>
#include <arch/board/generated/csr.h>

#include "chip.h"
#include "misoc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLOCKS_PER_SEC (see include/time.h).  CLOCKS_PER_SEC defines the desired
 * number of system clock ticks per second.  That value is a user
 * configurable setting based on CONFIG_USEC_PER_TICK.  It defaults to 100
 * (100 ticks per second = 10 MS interval).
 *
 * Given the timer input frequency (Finput).  The timer correct reload
 * value is:
 *
 *   reload = Finput / CLOCKS_PER_SEC
 */

#define SYSTICK_RELOAD ((SYSTEM_CLOCK_FREQUENCY / CLOCKS_PER_SEC) - 1)

/* The size of the reload field is 30 bits.  Verify that the reload value
 * will fit in the reload register.
 */

#if SYSTICK_RELOAD > 0x3fffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  misoc_timer_isr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

int misoc_timer_isr(int irq, void *context, void *arg)
{
  /* Clear event pending */

  timer0_ev_pending_write(timer0_ev_pending_read());

  /* Process timer interrupt */

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

void up_timer_initialize(void)
{
  /* Clear event pending */

  timer0_ev_pending_write(timer0_ev_pending_read());

  /* Disable timer */

  timer0_en_write(0);

  /* Setup the timer reload register to generate interrupts at the rate of
   * CLOCKS_PER_SEC.
   */

  timer0_reload_write(SYSTICK_RELOAD * 20);
  timer0_load_write(SYSTICK_RELOAD * 20);

  /* Enable timer */

  timer0_en_write(1);

  /* Attach the timer interrupt vector */

  irq_attach(TIMER0_INTERRUPT, misoc_timer_isr, NULL);

  /* And enable the timer interrupt */

  up_enable_irq(TIMER0_INTERRUPT);

  /* Enable IRQ of the timer core */

  timer0_ev_enable_write(1);
}
