/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_timerisr.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip.h"
#include "at32uc3_internal.h"
#include "at32uc3_rtc.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is normally provided by the
 * definition CLK_TCK (see include/time.h).  CLK_TCK defines the desired
 * number of system clock ticks per second.  That value is a user configurable
 * setting that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * However, the AVR RTC does not support that default value well and so, we
 * will insist that default is over-ridden by CONFIG_TICKS_PER_MSEC in the
 * configuration file.  Further, we will insist that CONFIG_TICKS_PER_MSEC
 * have the value 8 (see reasoning below).
 */

#if defined(CONFIG_MSEC_PER_TICK) && CONFIG_MSEC_PER_TICK != 10
#  error "Only a 100KHz system clock is supported"
#endif

/* The frequency of the RTC is given by:
 *
 * fRTC = fINPUT / 2**(PSEL + 1)
 *
 * Using the 32KHz clock, various RTC counting can be obtained:
 *
 * fRTC = 32000 / 2**16 = 32000/65536 = 0.49Hz  -> 2048 ms per tick
 * fRTC = 32000 / 2**15 = 32000/32768 = 0.98Hz  -> 1024 ms per tick
 * fRTC = 32000 / 2**14 = 32000/16384 = 1.95Hz  ->  512 ms per tick
 * fRTC = 32000 / 2**13 = 32000/8192  = 3.9Hz   ->  256 ms per tick
 * fRTC = 32000 / 2**12 = 32000/4096  = 7.8Hz   ->  128 ms per tick
 * fRTC = 32000 / 2**11 = 32000/2048  = 15.6Hz  ->   64 ms per tick
 * fRTC = 32000 / 2**10 = 32000/1024  = 31.3Hz  ->   32 ms per tick
 * fRTC = 32000 / 2**9  = 32000/512   = 62.5Hz  ->   16 ms per tick
 * fRTC = 32000 / 2**8  = 32000/256   = 125Hz   ->    8 ms per tick
 * fRTC = 32000 / 2**7  = 32000/128   = 250Hz   ->    4 ms per tick
 * fRTC = 32000 / 2**6  = 32000/64    = 500Hz   ->    2 ms per tick
 * fRTC = 32000 / 2**5  = 32000/32    = 1KHz    ->    1 ms per tick
 * fRTC = 32000 / 2**4  = 32000/16    = 2KHz    ->  500 us per tick
 * fRTC = 32000 / 2**3  = 32000/8     = 4KHz    ->  250 us per tick
 * fRTC = 32000 / 2**2  = 32000/4     = 8KHz    ->  125 us per tick
 * fRTC = 32000 / 2                   = 16KHz   -> 62.5 us per tick
 *
 * We'll use PSEL == 1 (fRTC == 125ns) and we will set TOP to 79.
 * Therefore, the TOP interrupt should occur after 79+1=80 counts
 * at a rate of 125us x 80 = 10 ms
 */
 
#define AV32_PSEL 15
#define AV32_TOP (80-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  rtc_busy
 *
 * Description:
 *   Make sure that the RTC is no busy before trying to operate on it.  If
 *   the RTC is busy, it will discard writes to TOP, VAL, and CTRL.
 *
 ****************************************************************************/

static void rtc_waitnotbusy(void)
{
  while ((getreg32(AVR32_RTC_CTRL) & RTC_CTRL_BUSY) != 0);
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

int up_timerisr(int irq, uint32_t *regs)
{
   /* Clear the pending timer interrupt */
 
   putreg32(RTC_INT_TOPI, AVR32_RTC_ICR);

   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/****************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.  NOTE:  This function depends on setup of OSC32 by
 *   up_clkinitialize().
 *
 ****************************************************************************/

void up_timerinit(void)
{
  uint32_t regval;

  /* Configure the RTC.  Source == 32KHz OSC32 */

  rtc_waitnotbusy();
  putreg32((RTC_CTRL_CLK32 | (AV32_PSEL << RTC_CTRL_PSEL_SHIFT) | RTC_CTRL_CLKEN),
           AVR32_RTC_CTRL);

  /* Set the counter value to zero and the TOP value to AVR32_TOP (see above) */

  rtc_waitnotbusy();
  putreg32(0, AVR32_RTC_VAL);
  putreg32(AV32_TOP, AVR32_RTC_TOP);

  /* Attach the timer interrupt vector */

  (void)irq_attach(AVR32_IRQ_RTC, (xcpt_t)up_timerisr);

  /* Enable RTC interrupts */

  putreg32(RTC_INT_TOPI, AVR32_RTC_IER);
  
  /* Enable the RTC */

  rtc_waitnotbusy();
  regval = getreg32(AVR32_RTC_CTRL);
  regval |= RTC_CTRL_EN;
  putreg32(regval, AVR32_RTC_CTRL);
}
