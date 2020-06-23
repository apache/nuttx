/****************************************************************************
 * arch/arm/src/stm32f7/stm32_fmc.c
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

#if defined(CONFIG_STM32F7_FMC)

#include <debug.h>

#include <nuttx/arch.h>

#include "chip.h"
#include "arm_arch.h"
#include "stm32_fmc.h"
#include "stm32_rcc.h"

/****************************************************************************
 * Private data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_fmc_sdram_wait
 *
 * Description:
 *   Wait for the SDRAM controller to be ready.
 *
 ****************************************************************************/

static void stm32_fmc_sdram_wait(void)
{
  int timeout = 0xffff;
  while (timeout > 0)
    {
      if ((getreg32(STM32_FMC_SDSR) & FMC_SDSR_BUSY) == 0)
        {
          break;
        }

      timeout--;
    }

  DEBUGASSERT(timeout > 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_fmc_init
 *
 * Description:
 *   Initialize the FMC peripheral.
 *
 ****************************************************************************/

void stm32_fmc_init(void)
{
  uint32_t regval;

  /* Reset the FMC on the AHB3 bus */

  regval  = getreg32(STM32_RCC_AHB3RSTR);
  regval |= RCC_AHB3RSTR_FMCRST;
  putreg32(regval, STM32_RCC_AHB3RSTR);

  /* Leave reset state */

  regval &= ~RCC_AHB3RSTR_FMCRST;
  putreg32(regval, STM32_RCC_AHB3RSTR);
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
      val |= FMC_SDCR_WP;       /* wp == 1 */
    }
  else
    {
      val &= ~FMC_SDCR_WP;      /* wp == 0 */
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

  stm32_fmc_sdram_wait();

  val  = getreg32(STM32_FMC_SDRTR);
  val &= ~(0x1fff << 1);        /* preserve non-count bits */
  val |= (count << 1);
  putreg32(val, STM32_FMC_SDRTR);
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
  DEBUGASSERT((timing & FMC_SDTR_RESERVED) == 0);

  sdtr = (bank == 1) ? STM32_FMC_SDTR1 : STM32_FMC_SDTR2;
  val  = getreg32(sdtr);
  val &= FMC_SDTR_RESERVED;     /* preserve reserved bits */
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
  DEBUGASSERT((ctrl & FMC_SDCR_RESERVED) == 0);

  sdcr = (bank == 1) ? STM32_FMC_SDCR1 : STM32_FMC_SDCR2;
  val  = getreg32(sdcr);
  val &= FMC_SDCR_RESERVED;     /* preserve reserved bits */
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
  uint32_t val;

  DEBUGASSERT((cmd & FMC_SDCMR_RESERVED) == 0);

  /* Wait for the controller to be ready */

  stm32_fmc_sdram_wait();

  val  = getreg32(STM32_FMC_SDCMR);
  val &= FMC_SDCMR_RESERVED;    /* Preserve reserved bits */
  val |= cmd;
  putreg32(val, STM32_FMC_SDCMR);
}

#endif /* CONFIG_STM32F7_FMC */
