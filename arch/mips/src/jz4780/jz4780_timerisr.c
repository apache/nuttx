/****************************************************************************
 * arch/mips/src/jz4780/jz4780_timerisr.c
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

#include "clock/clock.h"
#include "mips_internal.h"

#include "mips_internal.h"
#include "jz4780_lowinit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void reboot_s(int sec)
{
  putreg8(0, WD_TCER);
  putreg16(0, WD_TCNT);
  putreg16(32*sec, WD_TDR);
  putreg16(TCSR_PRESCALE(5) | TCSR_RTC_EN, WD_TCSR);
  putreg8(TCER_TCEN, WD_TCER);

  for (; ; )
    {
    }
}

/****************************************************************************
 * Function:  jz_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int jz_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear the pending timer interrupt */

  mips_clrpend_irq(JZ4780_IRQ_TCU0);

  putreg32(TFCR_OSTFCL, TFCR);

  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  mips_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  putreg32(0, OSTCNTL);
  putreg32(0, OSTCNTH);

  putreg16(TCSR_PRESCALE(TCSR_DIV_16) | TCSR_EXT_EN, OSTCSR);

  putreg32(TICKS_PER_MS * CONFIG_USEC_PER_TICK / 1000, OSTDR);

  putreg32(TESR_OSTEN, TESR);
  putreg32(TMCR_OSTMCL, TMCR);

  /* Configure the timer interrupt */

  mips_clrpend_irq(JZ4780_IRQ_TCU0);

  /* Attach the timer interrupt vector */

  (void)irq_attach(JZ4780_IRQ_TCU0, (xcpt_t)jz_timerisr, NULL);

  /* And enable the timer interrupt */

  up_enable_irq(JZ4780_IRQ_TCU0);
}

#ifdef CONFIG_BOARDCTL_RESET

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  Support for this function is required by board-level
 *   logic if CONFIG_BOARDCTL_RESET is selected.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *            meaning of this status information is board-specific.  If not
 *            used by a board, the value zero may be provided in calls to
 *            board_reset().
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

int board_reset(int status)
{
  reboot_s(1);
  return 0;
}

#endif /* CONFIG_BOARDCTL_RESET */
