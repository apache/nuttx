/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pmlpr.c
 *
 *   Copyright (C) 2012, 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include "up_arch.h"
#include "nvic.h"
#include "stm32l4_pwr.h"
#include "stm32l4_pm.h"
#include "stm32l4_rcc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_pmlpr
 *
 * Description:
 *   Enter Low-Power Run (LPR) mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero means that LPR was successfully entered. Otherwise, LPR mode was not
 *   entered and a negated errno value is returned to indicate the cause of the
 *   failure.
 *
 ****************************************************************************/

int stm32l4_pmlpr(void)
{
  uint32_t regval;

  /* Enable MSI clock */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_MSION;

  /* Set MSI clock to 2 MHz */

  regval &= ~RCC_CR_MSIRANGE_MASK;
  regval |= RCC_CR_MSIRANGE_2M; /* 2 MHz */
  regval |= RCC_CR_MSIRGSEL;    /* Select new MSIRANGE */
  putreg32(regval, STM32L4_RCC_CR);

  /* Select MSI clock as system clock source */

  regval  = getreg32(STM32L4_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= RCC_CFGR_SW_MSI;
  putreg32(regval, STM32L4_RCC_CFGR);

  /* Wait until the MSI source is used as the system clock source */

  while ((getreg32(STM32L4_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_MSI)
    {
    }

  /* Enable Low-Power Run */

  regval  = getreg32(STM32L4_PWR_CR1);
  regval |= PWR_CR1_LPR;
  putreg32(regval, STM32L4_PWR_CR1);

  return OK;
}
