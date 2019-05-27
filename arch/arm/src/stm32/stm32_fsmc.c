/************************************************************************************
 * arch/arm/src/stm32/stm32_fsmc.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <sirmanlypowers@gmail.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "stm32.h"

#if defined(CONFIG_STM32_FSMC)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: stm32_fsmc_enable
 *
 * Description:
 *   Enable clocking to the FSMC.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F10XX)

void stm32_fsmc_enable(void)
{
  modifyreg32(STM32_RCC_AHBENR, 0, RCC_AHBENR_FSMCEN);
}

#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)

void stm32_fsmc_enable(void)
{
  modifyreg32(STM32_RCC_AHB3ENR, 0, RCC_AHB3ENR_FSMCEN);
}

#endif

/****************************************************************************
 * Name: stm32_fsmc_disable
 *
 * Description:
 *   Disable clocking to the FSMC.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F10XX)

void stm32_fsmc_disable(void)
{
  modifyreg32(STM32_RCC_AHBENR, RCC_AHBENR_FSMCEN, 0);
}

#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)

void stm32_fsmc_disable(void)
{
  modifyreg32(STM32_RCC_AHB3ENR, RCC_AHB3ENR_FSMCEN, 0);
}

#endif

#endif /* CONFIG_STM32_FSMC */
