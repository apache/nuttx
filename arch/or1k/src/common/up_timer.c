/****************************************************************************
 * arch/or1k/src/common/up_timer.c
 *
 *   Copyright (C) 2018 Extent3D. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <arch/spr.h>

#include "clock/clock.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TTMR_MATCH (CONFIG_OR1K_CPU_FREQUENCY/100)
#define TTMR_LOAD (TTMR_MATCH | SPR_TTMR_IE | SPR_TTMR_M)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: or1k_timer_isr
 *
 * Description:
 *   Tick Timer interrupt handler
 *
 ****************************************************************************/

static int or1k_timer_isr(int irq, uint32_t *regs, void *arg)
{
  uint32_t ttmr = TTMR_LOAD;

  /* Clear the TTMR interrupt */

  mtspr(SPR_TICK_TTMR, ttmr);

  nxsched_process_timer();

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Initialize the OpenRISC Tick Timer unit
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t ttmr = TTMR_LOAD;

  irq_attach(OR1K_IRQ_TICK, (xcpt_t)or1k_timer_isr, NULL);

  /* Clear TTCR */

  mtspr(SPR_TICK_TTCR, 0);

  /* Write TTMR */

  mtspr(SPR_TICK_TTMR, ttmr);
}
