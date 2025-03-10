/****************************************************************************
 * arch/avr/src/avrdx/avrdx_timerisr.c
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
#include <arch/board/board.h>
#include <avr/io.h>

#include "avr_internal.h"
#include "avrdx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The system timer is clocked from 32kHz internal oscillator
 * with a period of 61us. (Using division by 2 in case oscillator
 * correction is used at some point.)
 *
 * For this clock frequency, the shortest period we can support
 * with no errors is 15625us, 64Hz. Multiplies of this value will
 * also lead to timer with no errors.
 */

#define RTC_PER_VAL ((CONFIG_USEC_PER_TICK * 16384ul / 1000000ul) - 1)

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  avrdx_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int avrdx_timerisr(int irq, uint32_t *regs, FAR void *arg)
{
  RTC.INTFLAGS = (RTC_OVF_bm | RTC_CMP_bm);
  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Function:  avrdx_rtc_await_nobusy
 *
 * Description:
 *   RTC peripheral runs asynchronously from main clock, writes to some
 *   registers take time to synchronize and subsequent writes will
 *   not take effect.
 *
 *   This method reads RTC.STATUS and spins until the requested register
 *   isn't busy.
 *
 *   If instructed, it will also enable and disable interrupts while
 *   spinning so at least something can run, this is otherwise quite
 *   multitasking-unfriendly
 *
 ****************************************************************************/

void avrdx_rtc_await_nobusy(uint8_t wait_flag, irqstate_t irqstate)
{
  while (RTC.STATUS & wait_flag)
    {
      up_irq_restore(irqstate); /* Might not actually enable interrupts
                                 * based on irqstate contents */
      asm volatile("nop");      /* Why not, we are waiting for 2 ticks
                                 * of 32768Hz clock anyway */
      up_irq_save();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the system timer.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  irqstate_t irqstate;

  /* Setup RTC timer
   */

  irqstate = up_irq_save();

  /* RTC initialization procedure as per docs:
   * 1. Configure desired oscillator in Clock Controller
   *    (Using OSC32K which is preconfigured. Done.)
   * 2. Write RTC.CLKSEL clock select field.
   *    (Using OSC32K which is preconfigured. Done.)
   * 3. Write compare and period registers.
   */

  avrdx_rtc_await_nobusy(RTC_PERBUSY_bm, \
                         irqstate); /* Break the line to make nxstyle happy */
  RTC.PER = RTC_PER_VAL;

  /* 4. Enable desired interrupts */

  RTC.INTCTRL |= (RTC_OVF_bm);

  /* 5. Configure prescaler */

  avrdx_rtc_await_nobusy(RTC_CTRLABUSY_bm, \
                         irqstate);
  RTC.CTRLA = (RTC.CTRLA & ~RTC_PRESCALER_GM) | RTC_PRESCALER_DIV2_GC;

  /* 6. Enable the RTC */

  avrdx_rtc_await_nobusy(RTC_CTRLABUSY_bm, \
                         irqstate);
  RTC.CTRLA |= RTC_RTCEN_bm;

  irq_attach(AVRDX_IRQ_RTC_CNT, (xcpt_t)avrdx_timerisr, NULL);

  up_irq_restore(irqstate);
}
