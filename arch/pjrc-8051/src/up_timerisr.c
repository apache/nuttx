/************************************************************
 * up_timerisr.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <8052.h>
#include "clock_internal.h"
#include "up_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Types
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/************************************************************
 * Global Functions
 ************************************************************/

/************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for
 *   various portions of the systems.
 *
 ************************************************************/

int up_timerisr(int irq, FAR ubyte *frame)
{
   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ************************************************************/

void up_timerinit(void)
{
#ifdef CONFIG_80C52_TIMER2
  up_disable_irq(TIMER2_IRQ);

  /* Set up timer 2 -- See up_internal.h for details */

  T2MOD  = 0;

  /* Set up the capture count to generate 100Hz system
   * interrupts.
   */

  RCAP2L = TIMER2_CAPTURE_LOW;
  RCAP2H = TIMER2_CAPTURE_HIGH;

  TL2    = TIMER2_CAPTURE_LOW;
  TH2    = TIMER2_CAPTURE_HIGH;

  /* Configure for interrupts */

  T2CON  = 0x40;

  /* Attach and enable the timer interrupt */

  irq_attach(TIMER2_IRQ, (xcpt_t)up_timerisr);
  up_enable_irq(TIMER2_IRQ);

#else
# warning "No support for timer 0 as the system timer"
#endif
}

