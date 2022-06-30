/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_pwr.c
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

#include "arm_internal.h"
#include "chip.h"
#include "stm32wb_pwr.h"
#include "stm32wb_rcc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t stm32wb_pwr_getreg(uint8_t offset)
{
  return (uint16_t)getreg32(STM32WB_PWR_BASE + (uint32_t)offset);
}

static inline void stm32wb_pwr_putreg(uint8_t offset, uint16_t value)
{
  putreg32((uint32_t)value, STM32WB_PWR_BASE + (uint32_t)offset);
}

static inline void stm32wb_pwr_modifyreg(uint8_t offset, uint16_t clearbits,
                                         uint16_t setbits)
{
  modifyreg32(STM32WB_PWR_BASE + (uint32_t)offset,
              (uint32_t)clearbits, (uint32_t)setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers and backup
 *   registers).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   True: The backup domain was previously writable.
 *
 ****************************************************************************/

bool stm32wb_pwr_enablebkp(bool writable)
{
  uint16_t regval;
  bool waswritable;

  /* Get the current state of the STM32WB PWR control register 1 */

  regval      = stm32wb_pwr_getreg(STM32WB_PWR_CR1_OFFSET);
  waswritable = ((regval & PWR_CR1_DBP) != 0);

  /* Enable or disable the ability to write */

  if (waswritable && !writable)
    {
      /* Disable backup domain access */

      regval &= ~PWR_CR1_DBP;
      stm32wb_pwr_putreg(STM32WB_PWR_CR1_OFFSET, regval);
    }
  else if (!waswritable && writable)
    {
      /* Enable backup domain access */

      regval |= PWR_CR1_DBP;
      stm32wb_pwr_putreg(STM32WB_PWR_CR1_OFFSET, regval);

      /* Enable does not happen right away */

      up_udelay(4);
    }

  return waswritable;
}

/****************************************************************************
 * Name: stm32wb_pwr_enableusv
 *
 * Description:
 *   Enables or disables the USB Supply Valid monitoring.  Setting this bit
 *   is mandatory to use the USB OTG FS peripheral.
 *
 * Input Parameters:
 *   set - True: Vddusb is valid; False: Vddusb is not present. Logical and
 *         electrical isolation is applied to ignore this supply.
 *
 * Returned Value:
 *   True: The bit was previously set.
 *
 ****************************************************************************/

bool stm32wb_pwr_enableusv(bool set)
{
  uint32_t regval;
  bool was_set;

  /* Get the current state of the STM32WB PWR control register 2 */

  regval  = stm32wb_pwr_getreg(STM32WB_PWR_CR2_OFFSET);
  was_set = ((regval & PWR_CR2_USV) != 0);

  /* Enable or disable the ability to write */

  if (was_set && !set)
    {
      /* Disable the Vddusb monitoring */

      regval &= ~PWR_CR2_USV;
      stm32wb_pwr_putreg(STM32WB_PWR_CR2_OFFSET, regval);
    }
  else if (!was_set && set)
    {
      /* Enable the Vddusb monitoring */

      regval |= PWR_CR2_USV;
      stm32wb_pwr_putreg(STM32WB_PWR_CR2_OFFSET, regval);
    }

  return was_set;
}

/****************************************************************************
 * Name: stm32_pwr_setvos
 *
 * Description:
 *   Set voltage scaling for Vcore
 *
 * Input Parameters:
 *   vos - Either 1 or 2, to set to Range 1 or 2, respectively
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.
 *   If used for any other purpose that protection to assure that its
 *   operation is atomic will be required.
 *
 ****************************************************************************/

void stm32_pwr_setvos(int vos)
{
  uint32_t regval;

  if (vos != 1 && vos != 2)
    {
      return;
    }

  regval  = getreg32(STM32WB_PWR_CR1);
  regval &= ~PWR_CR1_VOS_MASK;

  if (vos == 1)
    {
      regval |= PWR_CR1_VOS_RANGE1;
    }
  else
    {
      regval |= PWR_CR1_VOS_RANGE2;
    }

  putreg32(regval, STM32WB_PWR_CR1);
}
