/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_timerisr.c
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "ab32vg1.h"
#include "hardware/ab32vg1_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TMR0 counts system clock cycles and wraps at TMR0PR */

#define TICK_RELOAD  (AB32VG1_SYSCLK_FREQUENCY / CLK_TCK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_timerisr
 ****************************************************************************/

static int ab32vg1_timerisr(int irq, void *context, void *arg)
{
  putreg32(TMR_CPND_TPCLR, AB32VG1_TMR0CPND);

  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  modifyreg32(AB32VG1_CLKGAT0, 0, CLKGAT0_TMR0);

  putreg32(0, AB32VG1_TMR0CON);
  putreg32(0, AB32VG1_TMR0CNT);
  putreg32(TICK_RELOAD - 1, AB32VG1_TMR0PR);
  putreg32(TMR_CPND_TPCLR, AB32VG1_TMR0CPND);

  irq_attach(AB32VG1_IRQ_TMR0, ab32vg1_timerisr, NULL);

  putreg32(TMR_CON_INCSEL_SYSCLK | TMR_CON_TIE | TMR_CON_TMREN,
           AB32VG1_TMR0CON);

  up_enable_irq(AB32VG1_IRQ_TMR0);
}
