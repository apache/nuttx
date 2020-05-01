/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pwr.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2013, 2015-2016 Gregory Nutt. All rights reserved.
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Gregory Nutt <gnutt@nuttx.org>
 *            dev@ziggurat29.com
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "arm_arch.h"
#include "stm32l4_pwr.h"
#include "stm32l4_rcc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t stm32l4_pwr_getreg(uint8_t offset)
{
  return (uint16_t)getreg32(STM32L4_PWR_BASE + (uint32_t)offset);
}

static inline void stm32l4_pwr_putreg(uint8_t offset, uint16_t value)
{
  putreg32((uint32_t)value, STM32L4_PWR_BASE + (uint32_t)offset);
}

static inline void stm32l4_pwr_modifyreg(uint8_t offset, uint16_t clearbits,
                                         uint16_t setbits)
{
  modifyreg32(STM32L4_PWR_BASE + (uint32_t)offset,
              (uint32_t)clearbits, (uint32_t)setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enableclk
 *
 * Description:
 *   Enable/disable the clock to the power control peripheral.  Enabling
 *   must be done after the APB1 clock is validly configured, and prior to
 *   using any functionality controlled by the PWR block (i.e. much of
 *   anything else provided by this module).
 *
 * Input Parameters:
 *   enable - True: enable the clock to the Power control (PWR) block.
 *
 * Returned Value:
 *   True:  the PWR block was previously enabled.
 *
 ****************************************************************************/

bool stm32l4_pwr_enableclk(bool enable)
{
  uint32_t regval;
  bool wasenabled;

  regval = getreg32(STM32L4_RCC_APB1ENR1);
  wasenabled = ((regval & RCC_APB1ENR1_PWREN) != 0);

  /* Power interface clock enable. */

  if (wasenabled && !enable)
    {
      /* Disable power interface clock */

      regval &= ~RCC_APB1ENR1_PWREN;
      putreg32(regval, STM32L4_RCC_APB1ENR1);
    }
  else if (!wasenabled && enable)
    {
      /* Enable power interface clock */

      regval |= RCC_APB1ENR1_PWREN;
      putreg32(regval, STM32L4_RCC_APB1ENR1);
    }

  return wasenabled;
}

/****************************************************************************
 * Name: stm32l4_pwr_enablebkp
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

bool stm32l4_pwr_enablebkp(bool writable)
{
  uint16_t regval;
  bool waswritable;

  /* Get the current state of the STM32L4 PWR control register 1 */

  regval      = stm32l4_pwr_getreg(STM32L4_PWR_CR1_OFFSET);
  waswritable = ((regval & PWR_CR1_DBP) != 0);

  /* Enable or disable the ability to write */

  if (waswritable && !writable)
    {
      /* Disable backup domain access */

      regval &= ~PWR_CR1_DBP;
      stm32l4_pwr_putreg(STM32L4_PWR_CR1_OFFSET, regval);
    }
  else if (!waswritable && writable)
    {
      /* Enable backup domain access */

      regval |= PWR_CR1_DBP;
      stm32l4_pwr_putreg(STM32L4_PWR_CR1_OFFSET, regval);

      /* Enable does not happen right away */

      up_udelay(4);
    }

  return waswritable;
}

/****************************************************************************
 * Name: stm32l4_pwr_enableusv
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

bool stm32l4_pwr_enableusv(bool set)
{
  uint32_t regval;
  bool was_set;
  bool was_clk_enabled;

  regval = getreg32(STM32L4_RCC_APB1ENR1);
  was_clk_enabled = ((regval & RCC_APB1ENR1_PWREN) != 0);

  if (!was_clk_enabled)
    {
      stm32l4_pwr_enableclk(true);
    }

  /* Get the current state of the STM32L4 PWR control register 2 */

  regval  = stm32l4_pwr_getreg(STM32L4_PWR_CR2_OFFSET);
  was_set = ((regval & PWR_CR2_USV) != 0);

  /* Enable or disable the ability to write */

  if (was_set && !set)
    {
      /* Disable the Vddusb monitoring */

      regval &= ~PWR_CR2_USV;
      stm32l4_pwr_putreg(STM32L4_PWR_CR2_OFFSET, regval);
    }
  else if (!was_set && set)
    {
      /* Enable the Vddusb monitoring */

      regval |= PWR_CR2_USV;
      stm32l4_pwr_putreg(STM32L4_PWR_CR2_OFFSET, regval);
    }

  if (!was_clk_enabled)
    {
      stm32l4_pwr_enableclk(false);
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

  regval  = getreg32(STM32L4_PWR_CR1);
  regval &= ~PWR_CR1_VOS_MASK;

  if (vos == 1)
    {
      regval |= PWR_CR1_VOS_RANGE1;
    }
  else
    {
      regval |= PWR_CR1_VOS_RANGE2;
    }

  putreg32(regval, STM32L4_PWR_CR1);
}
