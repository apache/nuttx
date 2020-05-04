/****************************************************************************
 * arch/risc-v/src/nr5m100/nr5_timerisr.c
 *
 *   Copyright (C) 2009, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Modified for RISC-V:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include "riscv_arch.h"

#include "nr5.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The RCC feeds the Cortex System Timer (SysTick) with the AHB clock (HCLK)
 * divided by 8.  The SysTick can work either with this clock or with the
 * Cortex clock (HCLK), configurable in the SysTick Control and Status
 * register.
 */

#ifdef CONFIG_NR5_SYSTICK_SCLK
#  define SYSTICK_RELOAD ((NR5_SCLK_FREQUENCY / CLOCKS_PER_SEC) - 1)
#else
#  define SYSTICK_RELOAD ((NR5_HCLK_FREQUENCY / CLOCKS_PER_SEC) - 1)
#endif

/* The size of the reload field is 30 bits.  Verify that the reload value
 * will fit in the reload register.
 */

#if SYSTICK_RELOAD > 0x3fffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t g_systick = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  nr5m100_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int nr5m100_timerisr(int irq, void *context, FAR void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_get_systick
 *
 * Description:
 *   Returns the current value of systick.
 *
 ****************************************************************************/

uint64_t up_get_systick(void)
{
  return g_systick;
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
  /* Set the SysTick interrupt to the default priority */

  up_clearpri1bit(NR5_IRQ_SYSTICK);
  up_clearpri2bit(NR5_IRQ_SYSTICK);
  up_clearpri3bit(NR5_IRQ_SYSTICK);

  /* Attach the timer interrupt vector */

  irq_attach(NR5_IRQ_SYSTICK, nr5m100_timerisr, NULL);

  /* Configure and enable SysTick to interrupt at the requested rate */

  up_setsystick(0x80000000 | SYSTICK_RELOAD);

  /* And enable the timer interrupt */

  up_enable_irq(NR5_IRQ_SYSTICK);
}
