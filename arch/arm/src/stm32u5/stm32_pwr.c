/****************************************************************************
 * arch/arm/src/stm32u5/stm32_pwr.c
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

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_arch.h"
#include "stm32_pwr.h"
#include "stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PWR_TIMEOUT (10 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t stm32_pwr_getreg(uint8_t offset)
{
  return (uint16_t)getreg32(STM32_PWR_BASE + (uint32_t)offset);
}

static inline void stm32_pwr_putreg(uint8_t offset, uint16_t value)
{
  putreg32((uint32_t)value, STM32_PWR_BASE + (uint32_t)offset);
}

static inline void stm32_pwr_modifyreg(uint8_t offset, uint16_t clearbits,
                                         uint16_t setbits)
{
  modifyreg32(STM32_PWR_BASE + (uint32_t)offset, (uint32_t)clearbits,
              (uint32_t)setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enableclk
 *
 * Description:
 *   Enable/disable the clock to the power control peripheral.  Enabling must
 *   be done after the APB1 clock is validly configured, and prior to using
 *   any functionality controlled by the PWR block (i.e. much of anything
 *   else provided by this module).
 *
 * Input Parameters:
 *   enable - True: enable the clock to the Power control (PWR) block.
 *
 * Returned Value:
 *   True:  the PWR block was previously enabled.
 *
 ****************************************************************************/

bool stm32_pwr_enableclk(bool enable)
{
  uint32_t regval;
  bool wasenabled;

  regval = getreg32(STM32_RCC_AHB3ENR);
  wasenabled = ((regval & RCC_AHB3ENR_PWREN) != 0);

  /* Power interface clock enable. */

  if (wasenabled && !enable)
    {
      /* Disable power interface clock */

      regval &= ~RCC_AHB3ENR_PWREN;
      putreg32(regval, STM32_RCC_AHB3ENR);
    }
  else if (!wasenabled && enable)
    {
      /* Enable power interface clock */

      regval |= RCC_AHB3ENR_PWREN;
      putreg32(regval, STM32_RCC_AHB3ENR);
    }

  return wasenabled;
}

/****************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data
 *   registers and backup SRAM).
 *
 * Input Parameters:
 *   writable  True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   True: The backup domain was previously writable.
 *
 ****************************************************************************/

bool stm32_pwr_enablebkp(bool writable)
{
  uint16_t regval;
  bool waswritable;

  /* Get the current state of the PWR disable Backup domain register */

  regval      = stm32_pwr_getreg(STM32_PWR_DBPR_OFFSET);
  waswritable = ((regval & PWR_DBPR_DBP) != 0);

  /* Enable or disable the ability to write */

  if (waswritable && !writable)
    {
      /* Disable backup domain access */

      regval &= ~PWR_DBPR_DBP;
      stm32_pwr_putreg(STM32_PWR_DBPR_OFFSET, regval);
    }
  else if (!waswritable && writable)
    {
      /* Enable backup domain access */

      regval |= PWR_DBPR_DBP;
      stm32_pwr_putreg(STM32_PWR_DBPR_OFFSET, regval);

      /* Enable does not happen right away */

      up_udelay(4);
    }

  return waswritable;
}

/****************************************************************************
 * Name stm32_pwr_adjustvcore
 *
 * Description:
 *   Adjusts the voltage used for digital peripherals (V_CORE) before
 *   raising or after decreasing the system clock frequency.  Compare
 *   [RM0456], section 10.5.4 Dynamic voltage scaling management.
 *
 * Input Parameters:
 *   sysclock - The frequency in Hertz the system clock will or has been set
 *              to.
 *
 ****************************************************************************/

void stm32_pwr_adjustvcore(unsigned sysclock)
{
  volatile int timeout;
  uint32_t vos_range;

  /* Select the applicable V_CORE voltage range depending on the new system
   * clock frequency.
   */

  DEBUGASSERT(sysclock <= 160000000);

  if (sysclock > 110000000)
    {
      vos_range = PWR_VOSR_VOS_RANGE1 | PWR_VOSR_BOOSTEN;
    }
  else if (sysclock > 55000000)
    {
      vos_range = PWR_VOSR_VOS_RANGE2 | PWR_VOSR_BOOSTEN;
    }
  else if (sysclock > 25000000)
    {
      vos_range = PWR_VOSR_VOS_RANGE3;
    }
  else
    {
      vos_range = PWR_VOSR_VOS_RANGE4;
    }

  modreg32(vos_range, PWR_VOSR_VOS_MASK | PWR_VOSR_BOOSTEN, STM32_PWR_VOSR);

  /* Wait until the new V_CORE voltage range has been applied. */

  for (timeout = PWR_TIMEOUT; timeout; timeout--)
    {
      if (getreg32(STM32_PWR_VOSR) & PWR_VOSR_VOSRDY)
        {
          break;
        }
    }

  DEBUGASSERT(timeout > 0);

  /* Wait until the voltage level for the currently used VOS is ready. */

  for (timeout = PWR_TIMEOUT; timeout; timeout--)
    {
      if (getreg32(STM32_PWR_SVMSR) & PWR_SVMSR_ACTVOSRDY)
        {
          break;
        }
    }

  DEBUGASSERT(timeout > 0);
#if 0
  /* Wait until the embedded power distribution (EPOD) booster has been
   * enabled, if applicable.
   */

  DEBUGASSERT(timeout > 0);

  if (vos_range & PWR_VOSR_BOOSTEN)
    {
      for (timeout = PWR_TIMEOUT; timeout; timeout--)
        {
          if (getreg32(STM32_PWR_VOSR) & PWR_VOSR_BOOSTRDY)
            {
              break;
            }
        }
    }
#endif

  DEBUGASSERT(timeout > 0);
}

/****************************************************************************
 * Name stm32_pwr_enable_smps
 *
 * Description:
 *   Select between the Low-Drop Out (LDO) or Switched Mode Power Suppy
 *   (SMPS) regulator.  Compare [RM0456], section 10.5.1 SMPS and LDO
 *   embedded regulators.
 *
 * Input Parameters:
 *   enable - If true, the SMPS regulator will be enabled, otherwise the LDO,
 *
 ****************************************************************************/

void stm32_pwr_enablesmps(bool enable)
{
  volatile int timeout;
  uint32_t regsel = enable ? PWR_CR3_REGSEL_SMPS : PWR_CR3_REGSEL_LDO;

  /* Select the respective regulator. */

  modreg32(regsel, PWR_CR3_REGSEL, STM32_PWR_CR3);

  /* Wait until the respective regulator has been activated. */

  for (timeout = PWR_TIMEOUT; timeout; timeout--)
    {
      if (enable)
        {
          if ((getreg32(STM32_PWR_SVMSR) & PWR_SVMSR_REGS) ==
              PWR_SVMSR_REGS_SMPS)
            {
              break;
            }
        }
      else
        {
          if ((getreg32(STM32_PWR_SVMSR) & PWR_SVMSR_REGS) ==
              PWR_SVMSR_REGS_LDO)
            {
              break;
            }
        }
    }

  DEBUGASSERT(timeout > 0);
}
