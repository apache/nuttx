/****************************************************************************
 * arch/arm/src/stm32h7/stm32_pwr.c
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

#include "barriers.h"
#include "arm_internal.h"
#include "stm32_pwr.h"
#include "stm32_gpio.h"

#if defined(CONFIG_STM32H7_PWR)

#define BREG_WAIT_USTIMEOUT 1000 /* uS to wait for regulator to come ready */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_bkp_writable_counter = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t stm32_pwr_getreg(uint32_t offset)
{
  return getreg32(STM32_PWR_BASE + offset);
}

static inline void stm32_pwr_putreg(uint32_t offset, uint32_t value)
{
  putreg32(value, STM32_PWR_BASE + offset);
}

static inline void stm32_pwr_modifyreg(uint32_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(STM32_PWR_BASE + offset, clearbits, setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwr_initbkp
 *
 * Description:
 *   Insures the referenced count access to the backup domain (RTC registers,
 *   RTC backup data registers and backup SRAM is consistent with the HW
 *   state without relying on a variable.
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
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Make the HW not writable */

  regval = stm32_pwr_getreg(STM32_PWR_CR1_OFFSET);
  regval &= ~PWR_CR1_DBP;
  stm32_pwr_putreg(STM32_PWR_CR1_OFFSET, regval);

  /* Make the reference count agree */

  g_bkp_writable_counter =  0;

  leave_critical_section(flags);

  stm32_pwr_enablebkp(writable);
}

/****************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data
 *   registers and backup SRAM).
 *
 *   NOTE:
 *   Reference counting is used in order to supported nested calls to this
 *   function.  As a consequence, every call to stm32_pwr_enablebkp(true)
 *   must be followed by a matching call to stm32_pwr_enablebkp(false).
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
  uint32_t regval;
  bool waswritable;
  bool wait = false;

  flags = enter_critical_section();
  ARM_DSB();

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

  ARM_DSB();
  leave_critical_section(flags);

  if (wait)
    {
      /* Enable does not happen right away */

      up_udelay(4);
    }
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
 *   At present, this function is called only from initialization logic.
 *   If used for any other purpose that protection to assure that its
 *   operation is atomic will be required.
 *
 ****************************************************************************/

void stm32_pwr_setpvd(uint32_t pls)
{
  uint32_t regval;

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

/****************************************************************************
 * Name: stm32_pwr_enablebreg
 *
 * Description:
 *   Enables the Backup regulator, the Backup regulator (used to maintain
 *   backup SRAM content in Standby and VBAT modes) is enabled. If BRE is
 *   reset, the backup regulator is switched off. The backup SRAM can still
 *   be used but its content will be lost in the Standby and VBAT modes.
 *   Once set, the application must wait that the Backup Regulator Ready
 *   flag (BRR) is set to indicate that the data written into the RAM will
 *   be maintained in the Standby and VBAT modes.
 *
 *   This function needs to be called after stm32_pwr_enablebkp(true) has
 *   been called.
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
  irqstate_t flags;
  uint32_t regval;
  uint32_t reg_wait = 0;

  flags = enter_critical_section();

  regval      = stm32_pwr_getreg(STM32_PWR_CR2_OFFSET);

  if (region)
    {
      /* Request to turn on, if it was off we have wait */

      reg_wait = BREG_WAIT_USTIMEOUT;
      regval     |= PWR_CR2_BREN;
    }
  else
    {
      regval     &= ~PWR_CR2_BREN;
    }

  stm32_pwr_putreg(STM32_PWR_CR2_OFFSET, regval);

  while (reg_wait-- &&
         (stm32_pwr_getreg(STM32_PWR_CR2_OFFSET) & PWR_CR2_BREN) == 0)
    {
      up_udelay(1);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_pwr_configurewkup
 *
 * Description:
 *   Configures the external wakeup (WKUP) signals for wakeup from standby
 *   mode. Sets rising/falling edge sensitivity and pull state.
 *
 *
 * Input Parameters:
 *   pin    - WKUP pin number (0-5) to work on
 *   en     - Enables the specified WKUP pin if true
 *   rising - If true, wakeup is triggered on rising edge, otherwise,
 *            it is triggered on the falling edge.
 *   pull   - Specifies the WKUP pin pull resistor configuration
 *            (GPIO_FLOAT, GPIO_PULLUP, or GPIO_PULLDOWN)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_configurewkup(uint32_t pin, bool en,
                             bool rising, uint32_t pull)
{
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(pin < 6);

  flags = enter_critical_section();

  regval      = stm32_pwr_getreg(STM32_PWR_WKUPEPR_OFFSET);

  if (en)
    {
      regval     |= STM32_PWR_WKUPEN(pin);
    }
  else
    {
      regval     &= ~STM32_PWR_WKUPEN(pin);
    }

  if (rising)
    {
      regval     &= ~STM32_PWR_WKUPP(pin);
    }
  else
    {
      regval     |= STM32_PWR_WKUPP(pin);
    }

  /* Set to the no pull-up state by default */

  regval &= ~ (STM32_PWR_WKUPPUPD_MASK << STM32_PWR_WKUPPUPD_SHIFT(pin));

  if (pull == GPIO_PULLUP)
    {
      regval     |= STM32_PWR_WKUPPUPD_PULLUP <<
                    STM32_PWR_WKUPPUPD_SHIFT(pin);
    }
  else if (pull == GPIO_PULLDOWN)
    {
      regval     |= STM32_PWR_WKUPPUPD_PULLDN <<
                    STM32_PWR_WKUPPUPD_SHIFT(pin);
    }

  stm32_pwr_putreg(STM32_PWR_WKUPEPR_OFFSET, regval);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_pwr_setvbatcharge
 *
 * Description:
 *   Configures the internal charge resistor to charge a battery attached to
 *   the VBAT pin.
 *
 *
 * Input Parameters:
 *   enable    - Enables the charge resistor if true, disables it if false
 *   resistor  - Sets charge resistor to 1.5 KOhm if true,
 *               sets it to 5 KOhm if false.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_setvbatcharge(bool enable, bool resistor)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  regval      = stm32_pwr_getreg(STM32_PWR_CR3_OFFSET);

  if (enable)
    {
      regval |= STM32_PWR_CR3_VBE;
    }
  else
    {
      regval &= ~STM32_PWR_CR3_VBE;
    }

  if (resistor)
    {
      regval |= STM32_PWR_CR3_VBRS;
    }
  else
    {
      regval &= ~STM32_PWR_CR3_VBRS;
    }

  stm32_pwr_putreg(STM32_PWR_CR3_OFFSET, regval);

  leave_critical_section(flags);
}
#endif /* CONFIG_STM32_PWR */
