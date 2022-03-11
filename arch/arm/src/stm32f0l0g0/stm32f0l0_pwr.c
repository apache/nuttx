/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32f0l0_pwr.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "stm32_pwr.h"

#if defined(CONFIG_STM32F0L0G0_PWR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Parts only support a single Wake-up pin do not include the numeric suffix
 * in the naming.
 */

#ifndef PWR_CSR_EWUP1
#  define PWR_CSR_EWUP1 PWR_CSR_EWUP
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_bkp_writable_counter = 0;

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
  uint16_t regval;

  /* Make the HW not writable */

  regval = stm32_pwr_getreg32(STM32_PWR_CR_OFFSET);
  regval &= ~PWR_CR_DBP;
  stm32_pwr_putreg32(STM32_PWR_CR_OFFSET, regval);

  /* Make the reference count agree */

  g_bkp_writable_counter = 0;
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
 *   this function.  As a consequence, every call to stm32_pwr_enablebkp
 *  (true) must be followed by a matching call to stm32_pwr_enablebkp(false).
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

  regval      = stm32_pwr_getreg32(STM32_PWR_CR_OFFSET);
  waswritable = ((regval & PWR_CR_DBP) != 0);

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

      regval &= ~PWR_CR_DBP;
      stm32_pwr_putreg32(STM32_PWR_CR_OFFSET, regval);
    }
  else if (!waswritable && g_bkp_writable_counter > 0)
    {
      /* Enable backup domain access */

      regval |= PWR_CR_DBP;
      stm32_pwr_putreg32(STM32_PWR_CR_OFFSET, regval);

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
 * Name: stm32_pwr_enablewkup
 *
 * Description:
 *   Enables the WKUP pin.
 *
 * Input Parameters:
 *   wupin - Selects the WKUP pin to enable/disable
 *   wupon - state to set it to
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  The only cause of failure is if the selected MCU does not
 *   support the requested wakeup pin.
 *
 ****************************************************************************/

int stm32_pwr_enablewkup(enum stm32_pwr_wupin_e wupin, bool wupon)
{
  uint16_t pinmask;

  /* Select the PWR_CSR bit associated with the requested wakeup pin */

  switch (wupin)
    {
      case PWR_WUPIN_1:    /* Wake-up pin 1 (all parts) */
        pinmask = PWR_CSR_EWUP1;
        break;

#ifdef HAVE_PWR_WKUP2
      case PWR_WUPIN_2:    /* Wake-up pin 2 */
        pinmask = PWR_CSR_EWUP2;
        break;
#endif

#ifdef HAVE_PWR_WKUP3
      case PWR_WUPIN_3:     /* Wake-up pin 3 */
        pinmask = PWR_CSR_EWUP3;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Set/clear the the wakeup pin enable bit in the CSR.  This must be done
   * within a critical section because the CSR is shared with other functions
   * that may be running concurrently on another thread.
   */

  if (wupon)
    {
      /* Enable the wakeup pin by setting the bit in the CSR. */

      stm32_pwr_modifyreg32(STM32_PWR_CSR_OFFSET, 0, pinmask);
    }
  else
    {
      /* Disable the wakeup pin by clearing the bit in the CSR. */

      stm32_pwr_modifyreg32(STM32_PWR_CSR_OFFSET, pinmask, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_pwr_getsbf
 *
 * Description:
 *   Return the standby flag.
 *
 ****************************************************************************/

bool stm32_pwr_getsbf(void)
{
  return (stm32_pwr_getreg32(STM32_PWR_CSR_OFFSET) & PWR_CSR_SBF) != 0;
}

/****************************************************************************
 * Name: stm32_pwr_getwuf
 *
 * Description:
 *   Return the wakeup flag.
 *
 ****************************************************************************/

bool stm32_pwr_getwuf(void)
{
  return (stm32_pwr_getreg32(STM32_PWR_CSR_OFFSET) & PWR_CSR_WUF) != 0;
}

/****************************************************************************
 * Name: stm32_pwr_setvos
 *
 * Description:
 *   Set voltage scaling for EnergyLite devices.
 *
 * Input Parameters:
 *   vos - Properly aligned voltage scaling select bits for the PWR_CR
 *         register.
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

#ifdef CONFIG_STM32F0L0G0_ENERGYLITE
void stm32_pwr_setvos(uint16_t vos)
{
  uint16_t regval;

  /* The following sequence is required to program the voltage regulator
   * ranges:
   * 1. Check VDD to identify which ranges are allowed...
   * 2. Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0.
   * 3. Configure the voltage scaling range by setting the VOS bits in the
   *    PWR_CR register.
   * 4. Poll VOSF bit of in PWR_CSR register. Wait until it is reset to 0.
   */

  while ((stm32_pwr_getreg32(STM32_PWR_CSR_OFFSET) & PWR_CSR_VOSF) != 0)
    {
    }

  regval  = stm32_pwr_getreg32(STM32_PWR_CR_OFFSET);
  regval &= ~PWR_CR_VOS_MASK;
  regval |= (vos & PWR_CR_VOS_MASK);
  stm32_pwr_putreg32(STM32_PWR_CR_OFFSET, regval);

  while ((stm32_pwr_getreg32(STM32_PWR_CSR_OFFSET) & PWR_CSR_VOSF) != 0)
    {
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
 *   At present, this function is called only from initialization logic.  If
 *   used for any other purpose that protection to assure that its operation
 *   is atomic will be required.
 *
 ****************************************************************************/

void stm32_pwr_setpvd(uint16_t pls)
{
  uint16_t regval;

  /* Set PLS */

  regval  = stm32_pwr_getreg32(STM32_PWR_CR_OFFSET);
  regval &= ~PWR_CR_PLS_MASK;
  regval |= (pls & PWR_CR_PLS_MASK);

  /* Write value to register */

  stm32_pwr_putreg32(STM32_PWR_CR_OFFSET, regval);
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

  stm32_pwr_modifyreg32(STM32_PWR_CR_OFFSET, 0, PWR_CR_PVDE);
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

  stm32_pwr_modifyreg32(STM32_PWR_CR_OFFSET, PWR_CR_PVDE, 0);
}

#endif /* CONFIG_STM32F0L0G0_ENERGYLITE */

#endif /* CONFIG_STM32F0L0G0_PWR */
