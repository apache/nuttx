/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_timerisr.c
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
#include "cxd56_powermgr.h"
#include "cxd56_timerisr.h"
#include "cxd56_clock.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The Clock Source: Either the internal CCLK or external STCLK (P3.26) clock
 * as the source in the STCTRL register.  This file alwyays configures the
 * timer to use CCLK as its source.
 */

static uint32_t g_systrvr;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cxd56_changeclock(uint8_t id);
static int cxd56_timerisr(int irq, uint32_t *regs, void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cxd56_changeclock(uint8_t id)
{
  irqstate_t flags;
  uint32_t systcsr;
  uint32_t current;

  if (id == CXD56_PM_CALLBACK_ID_CLK_CHG_START)
    {
      flags = enter_critical_section();
        {
          systcsr = getreg32(NVIC_SYSTICK_CTRL);
          systcsr &= ~NVIC_SYSTICK_CTRL_ENABLE;
          putreg32(systcsr, NVIC_SYSTICK_CTRL);
        }

      leave_critical_section(flags);
    }
  else if ((id == CXD56_PM_CALLBACK_ID_CLK_CHG_END) ||
           (id == CXD56_PM_CALLBACK_ID_HOT_BOOT))
    {
      current = (cxd56_get_cpu_baseclk() / CLK_TCK) - 1;

      flags = enter_critical_section();
        {
          if (g_systrvr != current)
            {
              putreg32(current, NVIC_SYSTICK_RELOAD);
              g_systrvr = current;
              putreg32(0, NVIC_SYSTICK_CURRENT);
            }

          if (id == CXD56_PM_CALLBACK_ID_CLK_CHG_END)
            {
              systcsr = getreg32(NVIC_SYSTICK_CTRL);
              systcsr |= NVIC_SYSTICK_CTRL_ENABLE;
              putreg32(systcsr, NVIC_SYSTICK_CTRL);
            }
        }

      leave_critical_section(flags);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  cxd56_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int cxd56_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

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
  uint32_t regval;

  /* Set the SysTick interrupt to the default priority */

  regval = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (CXD56M4_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

  /* Make sure that the SYSTICK clock source is set to use the CXD56xx CCLK */

  regval = getreg32(NVIC_SYSTICK_CTRL);
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* Configure SysTick to interrupt at the requested rate */

  g_systrvr = (cxd56_get_cpu_baseclk() / CLK_TCK) - 1;
  putreg32(g_systrvr, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(CXD56_IRQ_SYSTICK, (xcpt_t)cxd56_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(CXD56_IRQ_SYSTICK);
}

int cxd56_timerisr_initialize(void)
{
  cxd56_pm_register_callback(PM_CLOCK_APP_CPU, cxd56_changeclock);

  return 0;
}
