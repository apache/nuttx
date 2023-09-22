/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_timerisr.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "chip.h"
#include "mx8mp_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 */

#define SYSTICK_RELOAD(coreclk) (((coreclk) / CLK_TCK) - 1)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mx8mp_timerisr(int irq, uint32_t *regs, void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  mx8mp_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int mx8mp_timerisr(int irq, uint32_t *regs, void *arg)
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
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;
  uint32_t coreclk;
  uint32_t reload;

  /* Get the reload value */

  coreclk = mx8mp_ccm_get_clock(ARM_M7_CLK_ROOT);
  reload = SYSTICK_RELOAD(coreclk);

  /* The size of the reload field is 24 bits. */

  DEBUGASSERT(reload <= 0x00ffffff);

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(reload, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Attach the timer interrupt vector */

  irq_attach(MX8MP_IRQ_SYSTICK, (xcpt_t)mx8mp_timerisr, NULL);

  /* Enable SysTick interrupts */

  regval = (NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE);
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(MX8MP_IRQ_SYSTICK);
}
