/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32g0_pwr.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Daniel Pereira Volpato <dpo@certi.org.br>
 *
 *   Based on: arch/arm/src/stm32f0l0g0/stm32f0l0_pwr.c
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_arch.h"
#include "stm32_pwr.h"

#if defined(CONFIG_STM32F0L0G0_PWR)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t stm32_pwr_getreg32(uint8_t offset)
{
  return getreg32(STM32_PWR_BASE + (uint32_t)offset);
}

static inline void stm32_pwr_putreg32(uint8_t offset, uint32_t value)
{
  putreg32(value, STM32_PWR_BASE + (uint32_t)offset);
}

static inline void stm32_pwr_modifyreg32(uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits)
{
  modifyreg32(STM32_PWR_BASE + (uint32_t)offset, clearbits, setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_pwr_setvos(uint16_t vos)
{
  uint16_t regval;

  /* The following sequence is required to program the voltage regulator
   * ranges:
   * 1. Wait until VOSF flag is cleared in Power Status register 2 (PWR_SR2).
   * 2. Configure the voltage scaling range by setting the VOS bits in the
   *    PWR_CR1 register.
   * 3. Wait until VOSF flag is cleared in Power Status register 2 (PWR_SR2).
   *
   * No checking is performed to ensure the VOS value to be set is within the
   * valid range.
   */

  while ((stm32_pwr_getreg32(STM32_PWR_SR2_OFFSET) & PWR_SR2_VOSF) != 0)
    {
    }

  regval  = stm32_pwr_getreg32(STM32_PWR_CR1_OFFSET);
  regval &= ~PWR_CR1_VOS_MASK;
  regval |= (vos & PWR_CR1_VOS_MASK);
  stm32_pwr_putreg32(STM32_PWR_CR1_OFFSET, regval);

  while ((stm32_pwr_getreg32(STM32_PWR_SR2_OFFSET) & PWR_SR2_VOSF) != 0)
    {
    }
}

/* TODO Other stm32_pwr_* functions need to be implemented */

#endif /* CONFIG_STM32F0L0G0_PWR */
