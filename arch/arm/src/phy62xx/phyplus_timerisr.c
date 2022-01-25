/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_timerisr.c
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

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"

/* #include "irq.h" */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  phyplus_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int phyplus_timerisr(int irq, uint32_t *regs, void *arg)
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
  /* putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
   *  putreg32(0, NVIC_SYSTICK_CURRENT);
   */

  irq_attach(PHY62XX_IRQ_SYSTICK, (xcpt_t)phyplus_timerisr, NULL);

  /* Enable SysTick interrupts:
   *
   *   NVIC_SYSTICK_CTRL_CLKSOURCE=0 : Use the implementation defined clock
   *                                   source which, for the STM32F7, will be
   *                                   HCLK/8
   *   NVIC_SYSTICK_CTRL_TICKINT=1   : Generate interrupts
   *   NVIC_SYSTICK_CTRL_ENABLE      : Enable the counter
   */

  /* regval = (NVIC_SYSTICK_CTRL_TICKINT | NVIC_SYSTICK_CTRL_ENABLE);
   * putreg32(regval, NVIC_SYSTICK_CTRL);
   */

  up_enable_irq(PHY62XX_IRQ_SYSTICK);
}
