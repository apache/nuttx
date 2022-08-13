/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_timerisr.c
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

#include <debug.h>
#include <stdint.h>
#include <time.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "clock/clock.h"
#include "xtensa.h"
#include "xtensa_counter.h"

/****************************************************************************
 * Private data
 ****************************************************************************/

static uint32_t g_tick_divisor;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  esp32s2_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 *   Xtensa timers work by comparing a cycle counter with a preset value.
 *   Once the match occurs an interrupt is generated, and the handler has to
 *   set a new cycle count into the comparator.  To avoid clock drift due to
 *   interrupt latency, the new cycle count is computed from the old, not the
 *   time the interrupt was serviced. However if a timer interrupt is ever
 *   serviced more than one tick late, it is necessary to process multiple
 *   ticks until the new cycle count is in the future, otherwise the next
 *   timer interrupt would not occur until after the cycle counter had
 *   wrapped (2^32 cycles later).
 *
 ****************************************************************************/

static int esp32s2_timerisr(int irq, uint32_t *regs, void *arg)
{
  uint32_t divisor;
  uint32_t compare;
  uint32_t diff;

  divisor = g_tick_divisor;
  do
    {
      /* Increment the compare register for the next tick */

      compare = xtensa_getcompare();
      xtensa_setcompare(compare + divisor);

      /* Process one timer tick */

      nxsched_process_timer();

      /* Check if we are falling behind and need to process multiple timer
       * interrupts.
       */

      diff = xtensa_getcount() - compare;
    }
  while (diff >= divisor);

  return OK;
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
  uint32_t divisor;
  uint32_t count;

  /* Configured the timer0 as the system timer.
   *
   *   divisor = BOARD_CLOCK_FREQUENCY / ticks_per_sec
   */

  divisor = BOARD_CLOCK_FREQUENCY / CLOCKS_PER_SEC;
  g_tick_divisor = divisor;

  /* Set up periodic timer */

  count = xtensa_getcount();
  xtensa_setcompare(count + divisor);

  /* NOTE: Timer 0 is an internal interrupt source so we do not need to
   * attach any peripheral ID to the dedicated CPU interrupt.
   */

  /* Attach the timer interrupt */

  irq_attach(XTENSA_IRQ_TIMER0, (xcpt_t)esp32s2_timerisr, NULL);

  /* Enable the timer 0 CPU interrupt. */

  up_enable_irq(XTENSA_IRQ_TIMER0);
}
