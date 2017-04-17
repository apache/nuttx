/****************************************************************************
 * arch/arm/src/stm32f0/stm32f0_clockconfig.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "stm32f0_rcc.h"
#include "stm32f0_clockconfig.h"
#include "chip/stm32f0_syscfg.h"
#include "chip/stm32f0_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32f0_clockconfig
 *
 * Description:
 *   Called to initialize the STM32F0xx.  This does whatever setup is needed
 *   to put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void stm32f0_clockconfig(void)
{
  int regval;

  /*  Verify if PLL is already setup, if so define to use HSI mode */

  if ((getreg32(STM32F0_RCC_CFGR) & RCC_CFGR_SWS_MASK) == RCC_CFGR_SWS_PLL)
    {
      /* Select HSI mode */

      regval  = getreg32(STM32F0_RCC_CFGR);
      regval &= (uint32_t) (~RCC_CFGR_SW_MASK);
      putreg32(regval, STM32F0_RCC_CFGR);

      while ((getreg32(STM32F0_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI);
    }

  /* Disable the PLL */

  regval  = getreg32(STM32F0_RCC_CR);
  regval &= (uint32_t)(~RCC_CR_PLLON);
  putreg32(regval, STM32F0_RCC_CR);
  while ((getreg32(STM32F0_RCC_CR) & RCC_CR_PLLRDY) != 0);

  /* Configure the PLL. Multiple x6 to get 48MHz */

  regval  = getreg32(STM32F0_RCC_CFGR);
  regval &= ~RCC_CFGR_PLLMUL_MASK;
  regval |= RCC_CFGR_PLLMUL_CLKx6;
  putreg32(regval, STM32F0_RCC_CFGR);

  /* Enable the PLL */

  regval  = getreg32(STM32F0_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32F0_RCC_CR);
  while ((getreg32(STM32F0_RCC_CR) & RCC_CR_PLLRDY) == 0);

  /* Configure to use the PLL */

  regval  = getreg32(STM32F0_RCC_CFGR);
  regval |= (uint32_t)(RCC_CFGR_SW_PLL);
  putreg32(regval, STM32F0_RCC_CFGR);
  while ((getreg32(STM32F0_RCC_CFGR) & RCC_CFGR_SW_MASK) != RCC_CFGR_SW_PLL);
}
