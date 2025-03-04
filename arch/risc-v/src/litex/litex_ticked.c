/****************************************************************************
 * arch/risc-v/src/litex/litex_ticked.c
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
#include <stdbool.h>

#include "chip.h"

#include "riscv_internal.h"

#include "litex.h"
#include "litex_clockconfig.h"
#include "hardware/litex_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TICK_COUNT (litex_get_hfclk() / TICK_PER_SEC)

#if !defined(CONFIG_SCHED_TICKLESS)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int litex_timerisr(int irq, void *context, void *arg)
{
  /* Clear timer interrupt */

  putreg32(1 << LITEX_TIMER0_TIMEOUT_EV_OFFSET, LITEX_TIMER0_EV_PENDING);

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
  /* Disable the timer and clear any pending interrupt */

  putreg32(0, LITEX_TIMER0_EN);
  putreg32(1 << LITEX_TIMER0_TIMEOUT_EV_OFFSET, LITEX_TIMER0_EV_PENDING);

  /* Set the timer period */

  putreg32(TICK_COUNT, LITEX_TIMER0_LOAD);
  putreg32(TICK_COUNT, LITEX_TIMER0_RELOAD);

  /* Attach timer interrupt handler */

  irq_attach(LITEX_IRQ_TIMER0, litex_timerisr, NULL);

  /* Enable the timer */

  putreg32(LITEX_TIMER0_ENABLE_BIT, LITEX_TIMER0_EN);

  /* And enable the timer interrupt */

  putreg32(1 << LITEX_TIMER0_TIMEOUT_EV_OFFSET, LITEX_TIMER0_EV_ENABLE);
  up_enable_irq(LITEX_IRQ_TIMER0);
}

#endif /* !defind(CONFIG_SCHED_TICKLESS) */
