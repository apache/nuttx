/****************************************************************************
 * arch/z16/src/z16f/z16f_timerisr.c
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
#include "z16_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLOCKS_PER_SEC (see include/time.h).  CLOCKS_PER_SEC defines the desired
 * number of system clock ticks per second.  That value is a user
 * configurable setting that defaults to 100 (100 ticks per second = 10 MS
 * interval).
 *
 * The RCC feeds the Cortex System Timer (SysTick) with the AHB clock (HCLK)
 * divided by 8.  The SysTick can work either with this clock or with the
 * Cortex clock (HCLK), configurable in the SysTick Control and Status
 * register.
 */

/* System clock frequency value from ZDS target settings */

extern _Erom uint8_t SYS_CLK_FREQ;
#define _DEFCLK ((uint32_t)&SYS_CLK_FREQ)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  z16f_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the system.
 *
 ****************************************************************************/

static int z16f_timerisr(int irq, uint32_t *regs, void *arg)
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
  uint32_t reload;
  uint32_t scaledfreq;
  uint32_t rawdiv;
  uint8_t divider;
  uint8_t regval;
  int shift;

  up_disable_irq(Z16F_IRQ_SYSTIMER);

  /* Disable the timer and configure for divide by 1 and continuous mode. */

  regval = Z16F_TIMERSCTL1_DIV1 | Z16F_TIMERSCTL1_CONT;
  putreg8(regval, Z16F_TIMER0_CTL1);

  /* Assign an initial timer value */

  putreg16(0x0001, Z16F_TIMER0_HL);

  /* Calculate timer reload value (continuous mode)
   *
   *   timer_period    = reload_value * divisor / system_clock_freqency
   *   timer_frequency = system_clock_freqency / divisor / reload_value
   * or
   *   reload_value = (system_clock_frequency / timer_frequency / divisor
   *
   * The prescale value ranges from 1 to 128, the reload value must be less
   * then or equal to 0xffff.  We would like to select the smallest prescaler
   * value and the largest reload value for the greatest accuracy.
   *
   * Example: system_clock_frequency=20MHz, timer_frequency=100Hz:
   *  scaledfreq   = 20,000,000 / 100
   *               = 200,000
   *  rawdiv       = (200,000 >> 16) + 1
   *               = 4
   *  divider      = Z16F_TIMERSCTL1_DIV4
   *  shift        = 2
   *  reload       = 200,000 >> 2
   *               = 50,000
   *
   * Example: system_clock_frequency=18.432MHz, timer_frequency=100Hz:
   *  scaledfreq   = 20,000,000 / 100
   *               = 200,000
   *  divisor      = ((18,432,000 / 100) >> 16) + 1
   *               = 3 -> 4 (need to go up to next power of two)
   *  reload_value = 20,000,000 / 100 / 4
   *               = 56,080
   */

#if 0 /* Does not work ??? */
  scaledfreq = _DEFCLK / CLOCKS_PER_SEC;
#else
  scaledfreq = (BOARD_SYSTEM_FREQUENCY / CLOCKS_PER_SEC);
#endif

  rawdiv = (scaledfreq >> 16) + 1;
  if (rawdiv < 2)
    {
      divider = Z16F_TIMERSCTL1_DIV1;
      shift   = 0;
    }
  else if (rawdiv < 3)
    {
      divider = Z16F_TIMERSCTL1_DIV2;
      shift   = 1;
    }
  else if (rawdiv < 7)
    {
      divider = Z16F_TIMERSCTL1_DIV4;
      shift   = 2;
    }
  else if (rawdiv < 15)
    {
      divider = Z16F_TIMERSCTL1_DIV8;
      shift   = 3;
    }
  else if (rawdiv < 31)
    {
      divider = Z16F_TIMERSCTL1_DIV16;
      shift   = 4;
    }
  else if (rawdiv < 63)
    {
      divider = Z16F_TIMERSCTL1_DIV32;
      shift   = 5;
    }
  else if (rawdiv < 127)
    {
      divider = Z16F_TIMERSCTL1_DIV64;
      shift   = 6;
    }
  else
    {
      divider = Z16F_TIMERSCTL1_DIV128;
      shift   = 7;
    }

  reload = scaledfreq >> shift;
  DEBUGASSERT(reload <= 0xffff);

  /* Set the timer reload value */

  putreg16((uint16_t)reload, Z16F_TIMER0_R);

  /* Set the prescale value */

  regval  = getreg8(Z16F_TIMER0_CTL1);
  regval &= ~Z16F_TIMERSCTL1_DIVMASK;
  regval |= divider;
  putreg8(regval, Z16F_TIMER0_CTL1);

  /* Enable the timer */

  regval |= Z16F_TIMERCTL1_TEN;
  putreg8(regval, Z16F_TIMER0_CTL1);

  /* Set the timer priority */

  /* Attach and enable the timer interrupt (leaving at priority 0) */

  irq_attach(Z16F_IRQ_SYSTIMER, (xcpt_t)z16f_timerisr, NULL);
  up_enable_irq(Z16F_IRQ_SYSTIMER);
}
