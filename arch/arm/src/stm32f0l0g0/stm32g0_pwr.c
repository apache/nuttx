/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32g0_pwr.c
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
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
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
