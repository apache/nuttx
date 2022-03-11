/****************************************************************************
 * arch/arm/src/stm32f7/stm32_pwr.c
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
#include <assert.h>
#include <errno.h>

#include "arm_internal.h"
#include "stm32_pwr.h"

#if defined(CONFIG_STM32F7_PWR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_bkp_writable_counter = 0;

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
  modifyreg32(STM32_PWR_BASE + (uint32_t)offset,
              (uint32_t)clearbits,
              (uint32_t)setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwr_initbkp
 *
 * Description:
 *   Insures the referenced count access to the backup domain (RTC
 *   registers, RTC backup data registers and backup SRAM is consistent with
 *   the HW state without relying on a variable.
 *
 *   NOTE: This function should only be called by SoC Start up code.
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_initbkp(bool writable)
{
  uint16_t regval;

  /* Make the HW not writable */

  regval = stm32_pwr_getreg(STM32_PWR_CR1_OFFSET);
  regval &= ~PWR_CR1_DBP;
  stm32_pwr_putreg(STM32_PWR_CR1_OFFSET, regval);

  /* Make the reference count agree */

  g_bkp_writable_counter =  0;
  stm32_pwr_enablebkp(writable);
}

/****************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data
 *   registers and backup SRAM).
 *
 *   NOTE: Reference counting is used in order to supported nested calls to
 *   this function.  As a consequence, every call to
 *   stm32_pwr_enablebkp(true) must be followed by a matching call to
 *   stm32_pwr_enablebkp(false).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_enablebkp(bool writable)
{
  irqstate_t flags;
  uint16_t regval;
  bool waswritable;
  bool wait = false;

  flags = enter_critical_section();

  /* Get the current state of the STM32 PWR control register */

  regval      = stm32_pwr_getreg(STM32_PWR_CR1_OFFSET);
  waswritable = ((regval & PWR_CR1_DBP) != 0);

  if (writable)
    {
      DEBUGASSERT(g_bkp_writable_counter < UINT16_MAX);
      g_bkp_writable_counter++;
    }
  else if (g_bkp_writable_counter > 0)
    {
      g_bkp_writable_counter--;
    }

  /* Enable or disable the ability to write */

  if (waswritable && g_bkp_writable_counter == 0)
    {
      /* Disable backup domain access */

      regval &= ~PWR_CR1_DBP;
      stm32_pwr_putreg(STM32_PWR_CR1_OFFSET, regval);
    }
  else if (!waswritable && g_bkp_writable_counter > 0)
    {
      /* Enable backup domain access */

      regval |= PWR_CR1_DBP;
      stm32_pwr_putreg(STM32_PWR_CR1_OFFSET, regval);

      wait = true;
    }

  leave_critical_section(flags);

  if (wait)
    {
      /* Enable does not happen right away */

      up_udelay(4);
    }
}

/****************************************************************************
 * Name: stm32_pwr_enablebreg
 *
 * Description:
 *   Enables the Backup regulator, the Backup regulator (used to maintain
 *   backup SRAM content in Standby and VBAT modes) is enabled. If BRE is
 *   reset, the backup regulator is switched off. The backup SRAM can still
 *   be used but its content will be lost in the Standby and VBAT modes.
 *   Once set, the application must wait until the Backup Regulator Ready
 *   flag (BRR) is set to indicate that the data written into the RAM will
 *   be maintained in the Standby and VBAT modes.
 *
 * Input Parameters:
 *   region - state to set it to
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_enablebreg(bool region)
{
  uint16_t regval;

  regval  = stm32_pwr_getreg(STM32_PWR_CSR1_OFFSET);
  regval &= ~PWR_CSR1_BRE;
  regval |= region ? PWR_CSR1_BRE : 0;
  stm32_pwr_putreg(STM32_PWR_CSR1_OFFSET, regval);

  if (region)
    {
      while ((stm32_pwr_getreg(STM32_PWR_CSR1_OFFSET) & PWR_CSR1_BRR) == 0);
    }
}

/****************************************************************************
 * Name: stm32_pwr_setvos
 *
 * Description:
 *   Set voltage scaling.
 *
 * Input Parameters:
 *   vos - Properly aligned voltage scaling select bits for the PWR_CR
 *   register.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.  If
 *   used for any other purpose that protection to assure that its operation
 *   is atomic will be required.
 *
 ****************************************************************************/

void stm32_pwr_setvos(uint16_t vos)
{
  uint16_t regval;

  /* The following sequence is required to program the voltage regulator
   * ranges:
   * 1. Check VDD to identify which ranges are allowed...
   * 2. Configure the voltage scaling range by setting the VOS bits in the
   *    PWR_CR1 register.
   */

  regval  = stm32_pwr_getreg(STM32_PWR_CR1_OFFSET);
  regval &= ~PWR_CR1_VOS_MASK;
  regval |= (vos & PWR_CR1_VOS_MASK);
  stm32_pwr_putreg(STM32_PWR_CR1_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32_pwr_setpvd
 *
 * Description:
 *   Sets power voltage detector
 *
 * Input Parameters:
 *   pls - PVD level
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.  If
 *   used for any other purpose that protection to assure that its operation
 *   is atomic will be required.
 *
 ****************************************************************************/

void stm32_pwr_setpvd(uint16_t pls)
{
  uint16_t regval;

  /* Set PLS */

  regval = stm32_pwr_getreg(STM32_PWR_CR1_OFFSET);
  regval &= ~PWR_CR1_PLS_MASK;
  regval |= (pls & PWR_CR1_PLS_MASK);

  /* Write value to register */

  stm32_pwr_putreg(STM32_PWR_CR1_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32_pwr_enablepvd
 *
 * Description:
 *   Enable the Programmable Voltage Detector
 *
 ****************************************************************************/

void stm32_pwr_enablepvd(void)
{
  /* Enable PVD by setting the PVDE bit in PWR_CR register. */

  stm32_pwr_modifyreg(STM32_PWR_CR1_OFFSET, 0, PWR_CR1_PVDE);
}

/****************************************************************************
 * Name: stm32_pwr_disablepvd
 *
 * Description:
 *   Disable the Programmable Voltage Detector
 *
 ****************************************************************************/

void stm32_pwr_disablepvd(void)
{
  /* Disable PVD by clearing the PVDE bit in PWR_CR register. */

  stm32_pwr_modifyreg(STM32_PWR_CR1_OFFSET, PWR_CR1_PVDE, 0);
}

#endif /* CONFIG_STM32_PWR */
