/****************************************************************************
 * arch/arm/src/moxart/moxart_timer.c
 * MoxaRT internal Timer Driver
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Anton D. Kachalov <mouse@mayc.ru>
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

#include <stdio.h>
#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm.h"
#include "up_arch.h"

/****************************************************************************
 * {re-processor Definitions
 ****************************************************************************/

#define TM1_ADDR  0x98400000

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum timer_reg
{
  COUNTER_TIMER    = 0x00,
  CNTL_TIMER       = 0x30,
  LOAD_TIMER       = 0x04,
  MATCH1_TIMER     = 0x08,
  MATCH2_TIMER     = 0x0C,
  INTR_STATE_TIMER = 0x34,
  INTR_MASK_TIMER  = 0x38,
};

enum timer_ctl
{
  TM1_ENABLE      = (1 << 0),
  TM1_CLOCK       = (1 << 1),
  TM1_OFENABLE    = (1 << 5),
  TM1_UPDOWN      = (1 << 9),
};

enum timer_int
{
  TM1_MATCH1      = (1 << 0),
  TM1_MATCH2      = (1 << 1),
  TM1_OVERFLOW    = (1 << 2),
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t cmp = BOARD_32KOSC_FREQUENCY / 100;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  moxart_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int moxart_timerisr(int irq, uint32_t *regs, void *arg)
{
  uint32_t state;

  /* Process timer interrupt */

  state = getreg32(TM1_ADDR + INTR_STATE_TIMER);
  state &= ~0x7;
  putreg32(state, TM1_ADDR + INTR_STATE_TIMER);

  /* Ready for the next interrupt */

  putreg32(cmp, TM1_ADDR + COUNTER_TIMER);

  sched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  arm_timer_initialize
 *
 * Description:
 *   Setup MoxaRT timer 0 to cause system ticks.
 *
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void arm_timer_initialize(void)
{
  uint32_t tmp;

//  up_disable_irq(IRQ_SYSTIMER);
  putreg32(0, TM1_ADDR + CNTL_TIMER);
  putreg32(0, TM1_ADDR + INTR_STATE_TIMER);
  putreg32(0x1ff, TM1_ADDR + INTR_MASK_TIMER);

  /* Initialize to a known state */

  putreg32(cmp, TM1_ADDR + COUNTER_TIMER);
  putreg32(0, TM1_ADDR + LOAD_TIMER);
  putreg32(0, TM1_ADDR + MATCH1_TIMER);

  /* Attach and enable the timer interrupt */

  irq_attach(IRQ_SYSTIMER, (xcpt_t)moxart_timerisr, NULL);
  up_enable_irq(IRQ_SYSTIMER);
  ftintc010_set_trig_mode(IRQ_SYSTIMER, 1);
  ftintc010_set_trig_level(IRQ_SYSTIMER, 0);

  /* Unmask IRQ */

  tmp = getreg32(TM1_ADDR + INTR_MASK_TIMER);
  tmp &= ~TM1_MATCH1;
  putreg32(tmp, TM1_ADDR + INTR_MASK_TIMER);

  tmp = getreg32(TM1_ADDR + CNTL_TIMER);
  tmp |= TM1_CLOCK | TM1_ENABLE;
  putreg32(tmp, TM1_ADDR + CNTL_TIMER);
}
