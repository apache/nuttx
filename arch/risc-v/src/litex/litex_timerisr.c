/****************************************************************************
 * arch/risc-v/src/litex/litex_timerisr.c
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
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "litex.h"
#include "hardware/litex_timer.h"
#include "litex_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TICK_COUNT (litex_get_hfclk() / TICK_PER_SEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  litex_timerisr
 ****************************************************************************/

static int litex_timerisr(int irq, void *context, FAR void *arg)
{
  putreg32(1, LITEX_TIMER0_EV_PENDING);
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
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  putreg32(0, LITEX_TIMER0_EN);
  putreg32(0, LITEX_TIMER0_LOAD);
  putreg32(TICK_COUNT, LITEX_TIMER0_RELOAD);
  putreg32(1, LITEX_TIMER0_EV_ENABLE);
  putreg32(1, LITEX_TIMER0_EN);
  irq_attach(LITEX_IRQ_TIMER0, litex_timerisr, NULL);
  up_enable_irq(LITEX_IRQ_TIMER0);
}
