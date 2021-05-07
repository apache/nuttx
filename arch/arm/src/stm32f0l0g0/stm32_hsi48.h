/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_hsi48.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_STM32_HSI48_H
#define __ARCH_ARM_SRC_STM32F0L0G0_STM32_HSI48_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32F0L0G0_HAVE_HSI48

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum syncsrc_e
{
  SYNCSRC_NONE = 0, /* No SYNC signal */
  SYNCSRC_GPIO,     /* GPIO selected as SYNC signal source */
  SYNCSRC_LSE,      /* LSE selected as SYNC signal source */
  SYNCSRC_USB,      /* USB SOF selected as SYNC signal source */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_enable_hsi48
 *
 * Description:
 *   On STM32F04x, STM32F07x and STM32F09x devices only, the HSI48 clock
 *   signal is generated from an internal 48 MHz RC oscillator and can be
 *   used directly as a system clock or divided and be used as PLL input.
 *
 *   The internal 48MHz RC oscillator is mainly dedicated to provide a high
 *   precision clock to the USB peripheral by means of a special Clock
 *   Recovery System (CRS) circuitry, which could use the USB SOF signal or
 *   the LSE or an external signal to automatically adjust the oscillator
 *   frequency on-fly, in a very small steps. This oscillator can also be
 *   used as a system clock source when the system is in run mode; it will
 *   be disabled as soon as the system enters in Stop or Standby mode. When
 *   the CRS is not used, the HSI48 RC oscillator runs on its default
 *   frequency which is subject to manufacturing process variations.
 *
 * Input Parameters:
 *   Identifies the syncrhonization source for the HSI48.  When used as the
 *   USB source clock, this must be set to SYNCSRC_USB.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_enable_hsi48(enum syncsrc_e syncsrc);

/****************************************************************************
 * Name: stm32_disable_hsi48
 *
 * Description:
 *   Disable the HSI48 clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_disable_hsi48(void);

#endif /* CONFIG_STM32F0L0G0_HAVE_HSI48 */
#endif /* __ARCH_ARM_SRC_STM32F0L0G0_STM32_HSI48_H */
