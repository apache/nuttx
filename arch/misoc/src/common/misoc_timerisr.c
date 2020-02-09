/****************************************************************************
 * arch/risc-v/src/nr5m100/nr5_timerisr.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Modified for MISOC:
 *
 *   Copyright (C) 2016 Ramtin Amin. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
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

  /* Disable timer*/

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
