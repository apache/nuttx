/************************************************************************************
 * arch/arm/src/stm32/stm32_pwr.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Gregory Nutt <gnutt@nuttx.org>
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
#include <errno.h>

#include "up_arch.h"
#include "stm32_pwr.h"

#if defined(CONFIG_STM32_PWR)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

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
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data registers
 *   and backup SRAM).
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   None
 *
 ************************************************************************************/

void stm32_pwr_enablebkp(void)
{
  stm32_pwr_modifyreg(STM32_PWR_CR_OFFSET, 0, PWR_CR_DBP);
}

/************************************************************************************
 * Name: stm32_pwr_setvos
 *
 * Description:
 *   Set voltage scaling for EneryLite devices.
 *
 * Input Parameters:
 *   vos - Properly aligned voltage scaling select bits for the PWR_CR register.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.  If used
 *   for any other purpose that protection to assure that its operation is atomic
 *   will be required.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_ENERGYLITE
void stm32_pwr_setvos(uint16_t vos)
{
  uint16_t regval;

  /* The following sequence is required to program the voltage regulator ranges:
   * 1. Check VDD to identify which ranges are allowed...
   * 2. Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0.
   * 3. Configure the voltage scaling range by setting the VOS bits in the PWR_CR
   *    register.
   * 4. Poll VOSF bit of in PWR_CSR register. Wait until it is reset to 0.
   */

  while ((stm32_pwr_getreg(STM32_PWR_CSR_OFFSET) & PWR_CSR_VOSF) != 0);

  regval  = stm32_pwr_getreg(STM32_PWR_CR_OFFSET);
  regval &= ~PWR_CR_VOS_MASK;
  regval |= (vos & PWR_CR_VOS_MASK);
  stm32_pwr_putreg(STM32_PWR_CR_OFFSET, regval);

  while ((stm32_pwr_getreg(STM32_PWR_CSR_OFFSET) & PWR_CSR_VOSF) != 0);
}
#endif

#endif /* CONFIG_STM32_PWR */
