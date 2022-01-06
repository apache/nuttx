/****************************************************************************
 * arch/arm/src/stm32/stm32_fsmc.c
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

#include "stm32.h"

#if defined(CONFIG_STM32_FSMC)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
