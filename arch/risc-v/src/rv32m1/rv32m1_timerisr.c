/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_timerisr.c
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

#include "riscv_internal.h"
#include "rv32m1.h"
#include "hardware/rv32m1_lptmr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Data Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  rv32m1_timerisr
 ****************************************************************************/

LOCATE_ITCM
static int rv32m1_timerisr(int irq, void *context, void *arg)
{
  /* Write '1' to clear the pending flag */

  uint32_t regval = getreg32(RV32M1_LPTMR_CSR);
  regval |= LPTMR_CSR_TCF;
  putreg32(regval,  RV32M1_LPTMR_CSR);

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
  uint32_t regaddr;
  uint32_t regval;

  /* Stop the timer and clear the pending flag */

  regaddr = RV32M1_LPTMR_CSR;
  regval = getreg32(regaddr);
  regval &= ~LPTMR_CSR_TEN;
  putreg32(regval, regaddr);

  /* Counter mode,
   * Reset counter when the compare value is matched,
   * No DMA request
   */

  regval &= ~(LPTMR_CSR_TMS  | LPTMR_CSR_TFC |
              LPTMR_CSR_TDRE | LPTMR_CSR_TPS_MASK);
  regval |= LPTMR_CSR_TPS0;

  putreg32(regval, regaddr);

  regaddr = RV32M1_LPTMR_PSR;
  regval = LPTMR_PSR_PCS_SIRCDIV3 | LPTMR_PSR_PBYP;
  putreg32(regval, regaddr);

  /* Attach timer interrupt handler */

  irq_attach(RV32M1_IRQ_LPTMR, rv32m1_timerisr, NULL);

  /* Open the timer interrupt gate */

  up_enable_irq(RV32M1_IRQ_LPTMR);

  /* Set ticks to compare */

  regval = rv32m1_clockfreq(CLK_SIRCDIV3) / TICK_PER_SEC;
  if (regval > 0)
    {
      /* Fine tune the ticks */

      --regval;
    }

  putreg32(regval, RV32M1_LPTMR_CMR);

  /* Start the timer with interrupt enabled */

  regval = getreg32(RV32M1_LPTMR_CSR);
  regval &= ~LPTMR_CSR_TCF;
  regval |= LPTMR_CSR_TEN | LPTMR_CSR_TIE;
  putreg32(regval, RV32M1_LPTMR_CSR);
}
