/****************************************************************************
 * arch/z80/src/ez80/ez80_timerisr.c
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

#include <arch/io.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "clock/clock.h"
#include "z80_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ez80_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the system.
 *
 ****************************************************************************/

static int ez80_timerisr(int irq, chipreg_t *regs, void *arg)
{
  /* Read the appropriate timer0 register to clear the interrupt */

#ifdef CONFIG_ARCH_CHIP_EZ80F91
  inp(EZ80_TMR0_IIR);
#else
  /* _EZ80190, _EZ80L92, _EZ80F92, _EZ80F93:  PRT_IRQ, is set to 1 when the
   * timer reloads the start value in CONTINUOUS mode.  The PRT_IRQ is
   * cleared to 0 and the interrupt service request signal is inactivated
   * when the CPU reads from the timer control register, TMRx_CTL.
   */

  inp(EZ80_TMR0_CTL);
#endif

  /* Process timer interrupt */

  nxsched_process_timer();

#ifdef CONFIG_ARCH_TIMERHOOK
  /* Architecture specific hook into the timer interrupt handler */

  up_timerhook();
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
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint16_t reload;

  /* Disable the timer */

  outp(EZ80_TMR0_CTL, 0x00);

  /* Attach system timer interrupts */

  irq_attach(EZ80_IRQ_SYSTIMER, (xcpt_t)ez80_timerisr, NULL);

  /* Set up the timer reload value */

  /* Write to the timer reload register to set the reload value.
   *
   * In continuous mode:
   *
   *   timer_period = reload_value x clock_divider / system_clock_frequency
   * or
   *   reload_value = (timer_period * system_clock_frequency) / clock_divider
   *
   * eZ80F91:
   *   For timer_period=10mS, and clock_divider=16, that would yield:
   *
   *     reload_value = system_clock_frequency / 1600
   *
   *   For a system timer of 50,000,000, that would result in a reload value
   *   of 31,250.
   *
   * eZ80F92:
   *   For timer_period=10mS, and clock_divider=4, that would yield:
   *
   *     reload_value = system_clock_frequency / 400
   *
   *   For a system timer of 20,000,000, * divider of 4, that would result
   *   in a reload value of 50,000.
   *
   * NOTE: The system clock frequency value is defined in the board.h file
   */

  reload = (uint16_t)(ez80_systemclock / 1600);
  outp(EZ80_TMR0_RRH, (uint8_t)(reload >> 8));
  outp(EZ80_TMR0_RRL, (uint8_t)(reload));

#if defined(CONFIG_ARCH_CHIP_EZ80F91)
  /* Clear any pending timer interrupts by reading the IIR register */

  inp(EZ80_TMR0_IIR);

#elif defined(CONFIG_ARCH_CHIP_EZ80L92) || defined(CONFIG_ARCH_CHIP_EZ80F92) || \
      defined(CONFIG_ARCH_CHIP_EZ80F93)
  /* Clear any pending timer interrupts by reading the CTL register */

  inp(EZ80_TMR0_CTL);

#endif

  /* Configure and enable the timer */

#if defined(_EZ80190)

  outp(EZ80_TMR0_CTL, 0x5f);

#elif defined(CONFIG_ARCH_CHIP_EZ80F91)
  /* EZ80_TMRCTL_TIMEN:   Bit 0: The programmable reload timer is enabled
   * EZ80_TMRCTL_RLD:     Bit 1: Force reload
   * EZ80_TMRCTL_TIMCONT: Bit 2: The timer operates in CONTINUOUS mode.
   * EZ80_TMRCLKDIV_16:   Bits 3-4: System clock divider = 16
   */

  outp(EZ80_TMR0_CTL, (EZ80_TMRCTL_TIMEN | EZ80_TMRCTL_RLD |
                       EZ80_TMRCTL_TIMCONT | EZ80_TMRCLKDIV_16));

  /* Enable timer end-of-count interrupts */

  outp(EZ80_TMR0_IER, EZ80_TMRIER_EOCEN);

#elif defined(CONFIG_ARCH_CHIP_EZ80L92) || defined(CONFIG_ARCH_CHIP_EZ80F92) || \
      defined(CONFIG_ARCH_CHIP_EZ80F93)
  /* EZ80_TMRCTL_TIMEN:   Bit 0: Programmable reload timer enabled.
   * EZ80_TMRCTL_RSTEN:   Bit 1: Reload and start function enabled.
   * EZ80_TMRCLKDIV_4:    Bits 2-3: Timer input clock divided by 4 (5Mhz)
   * EZ80_TMRCTL_TIMCONT: Bit 4: Continuous mode
   * EZ80_TMRCTL_EN:      Bit 6: Enable timer interrupt requests
   */

  outp(EZ80_TMR0_CTL, (EZ80_TMRCTL_TIMEN | EZ80_TMRCTL_RSTEN |
                       EZ80_TMRCLKDIV_4 | EZ80_TMRCTL_TIMCONT |
                       EZ80_TMRCTL_EN));

#endif
}
