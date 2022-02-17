/****************************************************************************
 * arch/arm/src/phy62xx/systick.c
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

#include "rom_sym_def.h"
#include "timer.h"
#include "error.h"
#include "clock.h"
#include "pwrmgr.h"
#include "nvic.h"
#include "arm_arch.h"
#include "jump_function.h"
#include <arch/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int systic_timerisr(int irq, uint32_t *regs, FAR void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

void up_timer_initialize(void)
{
  irq_attach(PHY62XX_IRQ_SYSTICK, (xcpt_t)systic_timerisr, NULL);

  putreg32((sysclk_get_clk() / 100 - 1), ARMV6M_SYSTICK_RVR); /* 10ms tick */
  putreg32((SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE |
      SYSTICK_CSR_CLKSOURCE), ARMV6M_SYSTICK_CSR);

  /* And enable the timer interrupt */

  up_enable_irq(PHY62XX_IRQ_SYSTICK);
}
