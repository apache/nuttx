/****************************************************************************
 * arch/arm/src/stm32wl5/stm32wl5_pwr.c
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "hardware/stm32wl5_pwr.h"
#include "arm_internal.h"
#include "stm32wl5_pwr.h"
#include "stm32wl5_rcc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t stm32wl5_pwr_getreg(uint8_t offset)
{
  return (uint16_t)getreg32(STM32WL5_PWR_BASE + (uint32_t)offset);
}

static inline void stm32wl5_pwr_putreg(uint8_t offset, uint16_t value)
{
  putreg32((uint32_t)value, STM32WL5_PWR_BASE + (uint32_t)offset);
}

static inline void stm32wl5_pwr_modifyreg(uint8_t offset, uint16_t clearbits,
                                         uint16_t setbits)
{
  modifyreg32(STM32WL5_PWR_BASE + (uint32_t)offset, (uint32_t)clearbits,
              (uint32_t)setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data
 *   registers and backup SRAM).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   True: The backup domain was previously writable.
 *
 ****************************************************************************/

bool stm32wl5_pwr_enablebkp(bool writable)
{
  uint16_t regval;
  bool waswritable;

  /* Get the current state of the STM32WL5 PWR control register 1 */

  regval      = stm32wl5_pwr_getreg(STM32WL5_PWR_CR1_OFFSET);
  waswritable = ((regval & PWR_CR1_DBP) != 0);

  /* Enable or disable the ability to write */

  if (waswritable && !writable)
    {
      /* Disable backup domain access */

      regval &= ~PWR_CR1_DBP;
      stm32wl5_pwr_putreg(STM32WL5_PWR_CR1_OFFSET, regval);
    }
  else if (!waswritable && writable)
    {
      /* Enable backup domain access */

      regval |= PWR_CR1_DBP;
      stm32wl5_pwr_putreg(STM32WL5_PWR_CR1_OFFSET, regval);

      /* Enable does not happen right away */

      up_udelay(4);
    }

  return waswritable;
}

/****************************************************************************
 * Name: stm32wl5_pwr_boot_c2
 *
 * Description:
 *   Boots up CPU2 (cortex-m0) after reset or wakeup from stop or standby
 *   modes.
 *
 ****************************************************************************/

void stm32wl5_pwr_boot_c2(void)
{
  modifyreg32(STM32WL5_PWR_CR4, 0, PWR_CR4_C2BOOT);
}
