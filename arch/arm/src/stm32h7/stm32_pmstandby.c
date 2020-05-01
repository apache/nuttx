/****************************************************************************
 * arch/arm/src/stm32h7/stm32_pmstandby.c
 *
 *   Copyright (C) 2018 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#include <stdbool.h>

#include "arm_arch.h"
#include "nvic.h"
#include "stm32_rcc.h"
#include "stm32_pwr.h"
#include "stm32_pm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pmstandby
 *
 * Description:
 *   Enter STANDBY mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pmstandby(void)
{
  uint32_t regval;

  /* Clear the wake-up flags before reseting. */

  modifyreg32(STM32_PWR_CPUCR, 0, STM32_PWR_CPUCR_CSSF);
  modifyreg32(STM32_PWR_WKUPCR, 0, STM32_PWR_WKUPC1 | STM32_PWR_WKUPC2 |
                                   STM32_PWR_WKUPC3 | STM32_PWR_WKUPC4 |
                                   STM32_PWR_WKUPC5 | STM32_PWR_WKUPC6);

  /* Clear reset flags. */

  modifyreg32(STM32_RCC_CSR, 0, RCC_RSR_RMVF);

  /* Set the domain Power Down Deep Sleep (PDDS) bits in the power control
   * register so that D1, D2, and D3 will go into the DStop state.
   */

  modifyreg32(STM32_PWR_CPUCR, 0, STM32_PWR_CPUCR_PDDS_D1 |
                                  STM32_PWR_CPUCR_PDDS_D2 |
                                  STM32_PWR_CPUCR_PDDS_D3);

  /* Set SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  /* Sleep until the wakeup reset occurs */

  asm("wfi");
}
