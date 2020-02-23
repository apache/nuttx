/****************************************************************************
 * arch/xtensa/src/esp32/esp32_timerisr.c
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <arch/xtensa/xtensa_specregs.h>
#include <arch/board/board.h>

#include "clock/clock.h"
#include "xtensa_timer.h"
#include "xtensa.h"

/****************************************************************************
 * Private data
 ****************************************************************************/

static uint32_t g_tick_divisor;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  xtensa_getcount, xtensa_getcompare, and xtensa_setcompare
 *
 * Description:
 *   Lower level operations on Xtensa special registers.
 *
 ****************************************************************************/

/* Return the current value of the cycle count register */

static inline uint32_t xtensa_getcount(void)
{
  uint32_t count;

  __asm__ __volatile__
  (
    "rsr %0, CCOUNT"  : "=r"(count)
  );

  return count;
}

/* Return the old value of the compare register */

static inline uint32_t xtensa_getcompare(void)
{
  uint32_t compare;

  __asm__ __volatile__
  (
    "rsr %0, %1"  : "=r"(compare) : "I"(XT_CCOMPARE)
  );

  return compare;
}

/* Set the value of the compare register */

static inline void xtensa_setcompare(uint32_t compare)
{
  __asm__ __volatile__
  (
    "wsr %0, %1" : : "r"(compare), "I"(XT_CCOMPARE)
  );
}

/****************************************************************************
 * Function:  esp32_timerisr
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

static int esp32_timerisr(int irq, uint32_t *regs, FAR void *arg)
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

  irq_attach(XTENSA_IRQ_TIMER0, (xcpt_t)esp32_timerisr, NULL);

  /* Enable the timer 0 CPU interrupt. */

  up_enable_irq(ESP32_CPUINT_TIMER0);
}
