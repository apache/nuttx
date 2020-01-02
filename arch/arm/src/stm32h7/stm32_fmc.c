/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fmc.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <sirmanlypowers@gmail.com>
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

#include "stm32.h"

#if defined(CONFIG_STM32H7_FMC)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_fmc_enable
 *
 * Description:
 *   Enable clocking to the FMC.
 *
 ****************************************************************************/

void stm32_fmc_enable_clk(void)
{
  modifyreg32(STM32_RCC_AHB3ENR, 0, RCC_AHB3ENR_FMCEN);
}

/****************************************************************************
 * Name: stm32_fmc_disable
 *
 * Description:
 *   Disable clocking to the FMC.
 *
 ****************************************************************************/

void stm32_fmc_disable(void)
{
  modifyreg32(STM32_RCC_AHB3ENR, RCC_AHB3ENR_FMCEN, 0);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_write_protect
 *
 * Description:
 *   Enable/Disable writes to an SDRAM.
 *
 ****************************************************************************/

void stm32_fmc_sdram_write_protect(int bank, bool state)
{
  uint32_t val;
  uint32_t sdcr;

  DEBUGASSERT(bank == 1 || bank == 2);
  sdcr = (bank == 1) ? STM32_FMC_SDCR1 : STM32_FMC_SDCR2;

  val = getreg32(sdcr);
  if (state)
    {
      val |= FMC_SDRAM_CR_WRITE_PROTECT;       /* wp == 1 */
    }
  else
    {
      val &= ~FMC_SDRAM_CR_WRITE_PROTECT;      /* wp == 0 */
    }

  putreg32(val, sdcr);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_set_refresh_rate
 *
 * Description:
 *   Set the SDRAM refresh rate.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_refresh_rate(int count)
{
  uint32_t val;

  DEBUGASSERT(count <= 0x1fff && count >= 0x29);
  putreg32(count << 1, STM32_FMC_SDRTR);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_set_timing
 *
 * Description:
 *   Set the SDRAM timing parameters.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_timing(int bank, uint32_t timing)
{
  uint32_t val;
  uint32_t sdtr;

  DEBUGASSERT((bank == 1) || (bank == 2));
  DEBUGASSERT((timing & FMC_SDRAM_TR_RESERVED) == 0);

  sdtr = (bank == 1) ? STM32_FMC_SDTR1 : STM32_FMC_SDTR2;
  val  = getreg32(sdtr);
  val &= FMC_SDRAM_TR_RESERVED;     /* preserve reserved bits */
  val |= timing;
  putreg32(val, sdtr);
}

/****************************************************************************
 * Name: stm32_fmc_enable
 *
 * Description:
 *   Enable FMC SDRAM. Do this after issue refresh rate.
 *
 ****************************************************************************/

void stm32_fmc_sdram_enable(void)
{
  uint32_t val;
  val = FMC_BCR_FMCEN | getreg32(STM32_FMC_BCR1);
  putreg32(val, STM32_FMC_BCR1);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_set_control
 *
 * Description:
 *   Set the SDRAM control parameters.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_control(int bank, uint32_t ctrl)
{
  uint32_t val;
  uint32_t sdcr;

  DEBUGASSERT((bank == 1) || (bank == 2));
  DEBUGASSERT((ctrl & FMC_SDRAM_CR_RESERVED) == 0);

  sdcr = (bank == 1) ? STM32_FMC_SDCR1 : STM32_FMC_SDCR2;
  val  = getreg32(sdcr);
  val &= FMC_SDRAM_CR_RESERVED;     /* preserve reserved bits */
  val |= ctrl;
  putreg32(val, sdcr);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_command
 *
 * Description:
 *   Send a command to the SDRAM.
 *
 ****************************************************************************/

void stm32_fmc_sdram_command(uint32_t cmd)
{
  DEBUGASSERT((cmd & FMC_SDRAM_CMD_RESERVED) == 0);
  putreg32(cmd, STM32_FMC_SDCMR);
}

#endif /* CONFIG_STM32H7_FMC */
