/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_timerisr.c
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
#include <nuttx/irq.h>
#include <bsp_api.h>
#include "gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use the MPCore Private Timer as the system tick source.
 *
 * The Private Timer interrupt (IRQn = -3 in the Renesas BSP) maps to
 * GIC interrupt ID 29 (INTID = IRQn + 32), which corresponds to
 * GIC_IRQ_STM in the NuttX GIC definitions.
 *
 * The required control bits are redefined locally to follow NuttX
 * naming conventions.
 */

#define RZV2H_PTCTLR_TE       (1 << 0)  /* Timer Enable */
#define RZV2H_PTCTLR_AR       (1 << 1)  /* Auto-Reload */
#define RZV2H_PTCTLR_IRQE     (1 << 2)  /* Interrupt Enable */

#define RZV2H_PTISR_CLR       (1 << 0)  /* Write 1 to clear the event flag */

/* The MPCore Private Timer clock is I6CLK / 4.
 * Use the BSP-defined clock value instead of a hardcoded frequency.
 */

#define RZV2H_PTIMER_CLOCK    (BSP_CFG_CLOCK_I6CLK_HZ / 4)

#define RZV2H_PTIMER_RELOAD   ((RZV2H_PTIMER_CLOCK / CLK_TCK) - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  rzv2h_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int rzv2h_timerisr(int irq, void *regs, void *arg)
{
  /* Clear the interrupt flag */

  R_PRIVATE_TIMER->PTISR = RZV2H_PTISR_CLR;

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
  /* Attach the timer interrupt vector */

  irq_attach(GIC_IRQ_STM, rzv2h_timerisr, NULL);

  /* Configure the Private Timer to interrupt at the requested rate */

  R_PRIVATE_TIMER->PTLR = RZV2H_PTIMER_RELOAD;

  R_PRIVATE_TIMER->PTCTLR = RZV2H_PTCTLR_TE | RZV2H_PTCTLR_AR |
                            RZV2H_PTCTLR_IRQE;

  /* And enable the timer interrupt */

  up_enable_irq(GIC_IRQ_STM);
}
