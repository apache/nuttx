/************************************************************************************
 * arch/arm/src/stm32f0/stm32f0_rcc.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0_STM32F0_HSI48_H
#define __ARCH_ARM_SRC_STM32F0_STM32F0_HSI48_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32F0_HAVE_HSI48

/************************************************************************************
 * Public Types
 ************************************************************************************/

enum syncsrc_e
{
  SYNCSRC_GPIO = 0, /* GPIO selected as SYNC signal source */
  SYNCSRC_LSE,      /* LSE selected as SYNC signal source */
  SYNCSRC_USB,      /* USB SOF selected as SYNC signal source */
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: stm32f0_enable_hsi48
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

void stm32f0_enable_hsi48(enum syncsrc_e syncsrc);

/****************************************************************************
 * Name: stm32f0_disable_hsi48
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

void stm32f0_disable_hsi48(void);

#endif /* CONFIG_STM32F0_HAVE_HSI48 */
#endif /* __ARCH_ARM_SRC_STM32F0_STM32F0_HSI48_H */
