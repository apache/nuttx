/************************************************************************************
 * arch/arm/src/stm32f7/stm32_pwr.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2013, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "up_arch.h"
#include "stm32_pwr.h"

#if defined(CONFIG_STM32F7_PWR)

/************************************************************************************
 * Private Data
 ************************************************************************************/

static uint16_t g_bkp_writable_counter = 0;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static inline uint16_t stm32_pwr_getreg(uint8_t offset)
{
  return (uint16_t)getreg32(STM32_PWR_BASE + (uint32_t)offset);
}

static inline void stm32_pwr_putreg(uint8_t offset, uint16_t value)
{
  putreg32((uint32_t)value, STM32_PWR_BASE + (uint32_t)offset);
}

static inline void stm32_pwr_modifyreg(uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
  modifyreg32(STM32_PWR_BASE + (uint32_t)offset, (uint32_t)clearbits, (uint32_t)setbits);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_pwr_initbkp
 *
 * Description:
 *   Insures the referenced count access to the backup domain (RTC registers,
 *   RTC backup data registers and backup SRAM is consistent with the HW state
 *   without relying on a variable.
 *
 *   NOTE: This function should only be called by SoC Start up code.
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

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

/************************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data registers
 *   and backup SRAM).
 *
 *   NOTE: Reference counting is used in order to supported nested calls to this
 *   function.  As a consequence, every call to stm32_pwr_enablebkp(true) must
 *   be followed by a matching call to stm32_pwr_enablebkp(false).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

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

/************************************************************************************
 * Name: stm32_pwr_enablebreg
 *
 * Description:
 *   Enables the Backup regulator, the Backup regulator (used to maintain backup
 *   SRAM content in Standby and VBAT modes) is enabled. If BRE is reset, the backup
 *   regulator is switched off. The backup SRAM can still be used but its content will
 *   be lost in the Standby and VBAT modes. Once set, the application must wait that
 *   the Backup Regulator Ready flag (BRR) is set to indicate that the data written
 *   into the RAM will be maintained in the Standby and VBAT modes.
 *
 * Input Parameters:
 *   regon - state to set it to
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void stm32_pwr_enablebreg(bool regon)
{
  uint16_t regval;

  regval  = stm32_pwr_getreg(STM32_PWR_CSR1_OFFSET);
  regval &= ~PWR_CSR1_BRE;
  regval |= regon ? PWR_CSR1_BRE : 0;
  stm32_pwr_putreg(STM32_PWR_CSR1_OFFSET, regval);

  if (regon)
    {
      while ((stm32_pwr_getreg(STM32_PWR_CSR1_OFFSET) & PWR_CSR1_BRR) == 0);
    }
}

/************************************************************************************
 * Name: stm32_pwr_setvos
 *
 * Description:
 *   Set voltage scaling.
 *
 * Input Parameters:
 *   vos - Properly aligned voltage scaling select bits for the PWR_CR register.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.  If used
 *   for any other purpose that protection to assure that its operation is atomic
 *   will be required.
 *
 ************************************************************************************/

void stm32_pwr_setvos(uint16_t vos)
{
  uint16_t regval;

  /* The following sequence is required to program the voltage regulator ranges:
   * 1. Check VDD to identify which ranges are allowed...
   * 2. Configure the voltage scaling range by setting the VOS bits in the PWR_CR1
   *    register.
   */

  regval  = stm32_pwr_getreg(STM32_PWR_CR1_OFFSET);
  regval &= ~PWR_CR1_VOS_MASK;
  regval |= (vos & PWR_CR1_VOS_MASK);
  stm32_pwr_putreg(STM32_PWR_CR1_OFFSET, regval);
}

/************************************************************************************
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
 *   At present, this function is called only from initialization logic.  If used
 *   for any other purpose that protection to assure that its operation is atomic
 *   will be required.
 *
 ************************************************************************************/

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

/************************************************************************************
 * Name: stm32_pwr_enablepvd
 *
 * Description:
 *   Enable the Programmable Voltage Detector
 *
 ************************************************************************************/

void stm32_pwr_enablepvd(void)
{
  /* Enable PVD by setting the PVDE bit in PWR_CR register. */

  stm32_pwr_modifyreg(STM32_PWR_CR1_OFFSET, 0, PWR_CR1_PVDE);
}

/************************************************************************************
 * Name: stm32_pwr_disablepvd
 *
 * Description:
 *   Disable the Programmable Voltage Detector
 *
 ************************************************************************************/

void stm32_pwr_disablepvd(void)
{
  /* Disable PVD by clearing the PVDE bit in PWR_CR register. */

  stm32_pwr_modifyreg(STM32_PWR_CR1_OFFSET, PWR_CR1_PVDE, 0);
}

#endif /* CONFIG_STM32_PWR */
