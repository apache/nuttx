/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_timer_isr.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <debug.h>
#include <assert.h>

#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "tlsr82_timer.h"

#include "hardware/tlsr82_clock.h"
#include "hardware/tlsr82_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_ISR_CAPT    (CONFIG_TLSR82_CPU_CLK_MHZ * CONFIG_USEC_PER_TICK)

#if TIMER_ISR_CAPT > UINT32_MAX
#  error "The tick period is too large."
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* timer is 10ms tick part */

static void tlsr82_timer_setup(void)
{
  /* Disable timer0 */

  TIMER_CTRL_REG &= ~TIMER_CTRL_T0_ENABLE;

  /* Config timer0 */

  TIMER_TICK0_REG = 0;
  TIMER_CAPT0_REG = TIMER_ISR_CAPT;

  /* Enable timer0 */

  TIMER_CTRL_REG |= TIMER_CTRL_T0_ENABLE;
}

static int tlsr82_os_timer_isr(int irq, void *context, void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  /* Ack interrupt */

  TIMER_STATUS_REG |= TIMER_STATUS_T0_CLR;

  nxsched_process_timer();

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_initialize(void)
{
  int ret;
  up_disable_irq(NR_TIMER0_IRQ);

  ret = irq_attach(NR_TIMER0_IRQ, tlsr82_os_timer_isr, NULL);
  if (ret == OK)
    {
      tlsr82_timer_setup();
      up_enable_irq(NR_TIMER0_IRQ);
    }
  else
    {
      _err("timer irq attach fail, ret=%d\n", ret);
      PANIC();
    }
}

