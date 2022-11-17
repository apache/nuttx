/****************************************************************************
 * arch/x86/src/qemu/qemu_timerisr.c
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

/*   Based on Bran's kernel development tutorials. Rewritten for JamesM's
 *   kernel development tutorials.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/io.h>
#include <arch/board/board.h>

#include "clock/clock.h"
#include "x86_internal.h"
#include "chip.h"
#include "qemu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Programmable interval timer (PIT)
 *
 *   Fpit = Fin / divisor
 *   divisor = Fin / divisor
 *
 * Where:
 *
 *   Fpit = The desired interrupt frequency.
 *   Fin  = PIT input frequency (PIT_CLOCK provided in board.h)
 *
 * The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 */

#define PIT_DIVISOR  ((uint32_t)PIT_CLOCK/(uint32_t)CLK_TCK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: qemu_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int qemu_timerisr(int irq, uint32_t *regs, void *arg)
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
  /* uint32_t to avoid compile time overflow errors */

  uint32_t divisor = PIT_DIVISOR;
  DEBUGASSERT(divisor <= 0xffff);

  /* Attach to the timer interrupt handler */

  irq_attach(IRQ0, (xcpt_t)qemu_timerisr, NULL);

  /* Send the command byte to configure counter 0 */

  outb(PIT_OCW_MODE_SQUARE | PIT_OCW_RL_DATA |
       PIT_OCW_COUNTER_0, PIT_REG_COMMAND);

  /* Set the PIT input frequency divisor */

  outb((uint8_t)(divisor & 0xff),  PIT_REG_COUNTER0);
  outb((uint8_t)((divisor >> 8) & 0xff), PIT_REG_COUNTER0);

  /* And enable IRQ0 */

  up_enable_irq(IRQ0);
}
