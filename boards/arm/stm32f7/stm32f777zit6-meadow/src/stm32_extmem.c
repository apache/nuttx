/****************************************************************************
 * boards/arm/stm32f7/stm32f777zit6-meadow/src/stm32_extmem.c
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
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_fmc.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "stm32f777zit6-meadow.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32F7_FMC
#  warning "FMC is not enabled"
#endif

#if STM32F7_NGPIO < 6
#  error "Required GPIO ports not enabled"
#endif

#define STM32_FMC_NADDRCONFIGS 23
#define STM32_FMC_NDATACONFIGS 16

#define STM32_SDRAM_CLKEN     FMC_SDCMR_CTB1 | FMC_SDCMR_MODE_CLK_ENABLE
#define STM32_SDRAM_PALL      FMC_SDCMR_CTB1 | FMC_SDCMR_MODE_PALL
#define STM32_SDRAM_REFRESH   FMC_SDCMR_CTB1 | FMC_SDCMR_MODE_AUTO_REFRESH |\
                                FMC_SDCMR_NRFS(8)
#define STM32_SDRAM_MODEREG   FMC_SDCMR_CTB1 | FMC_SDCMR_MODE_LOAD_MODE |\
                                FMC_SDCMR_MRD_BURST_LENGTH_1 | \
                                FMC_SDCMR_MRD_BURST_TYPE_SEQUENTIAL |\
                                FMC_SDCMR_MRD_CAS_LATENCY_3 |\
                                FMC_SDCMR_MRD_OPERATING_MODE_STANDARD |\
                                FMC_SDCMR_MRD_WRITEBURST_MODE_SINGLE

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* GPIO configurations common to most external memories */

static const uint32_t g_addressconfig[STM32_FMC_NADDRCONFIGS] =
{
  GPIO_FMC_A0,  GPIO_FMC_A1,  GPIO_FMC_A2,  GPIO_FMC_A3,  GPIO_FMC_A4,
  GPIO_FMC_A5,  GPIO_FMC_A6,  GPIO_FMC_A7,  GPIO_FMC_A8,  GPIO_FMC_A9,
  GPIO_FMC_A10, GPIO_FMC_A11, GPIO_FMC_A12,

  GPIO_FMC_SDCKE0,   GPIO_FMC_SDNE0,   GPIO_FMC_SDNWE, GPIO_FMC_NBL0,
  GPIO_FMC_SDNRAS,   GPIO_FMC_NBL1,    GPIO_FMC_BA0,     GPIO_FMC_BA1,
  GPIO_FMC_SDCLK,    GPIO_FMC_SDNCAS
};

static const uint32_t g_dataconfig[STM32_FMC_NDATACONFIGS] =
{
  GPIO_FMC_D0,  GPIO_FMC_D1,  GPIO_FMC_D2,  GPIO_FMC_D3,  GPIO_FMC_D4,
  GPIO_FMC_D5,  GPIO_FMC_D6,  GPIO_FMC_D7,  GPIO_FMC_D8,  GPIO_FMC_D9,
  GPIO_FMC_D10, GPIO_FMC_D11, GPIO_FMC_D12, GPIO_FMC_D13, GPIO_FMC_D14,
  GPIO_FMC_D15
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage.
 *
 ****************************************************************************/

static void stm32_extmemgpios(const uint32_t *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < ngpios; i++)
    {
      stm32_configgpio(gpios[i]);
    }
}

/****************************************************************************
 * Name: stm32_sdramcommand
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 */

static void stm32_sdramcommand(uint32_t command)
{
  uint32_t  regval;
  volatile  uint32_t timeout = 0xffff;

  regval = getreg32(STM32_FMC_SDSR) & 0x00000020;
  while ((regval != 0) && timeout-- > 0)
    {
      regval = getreg32(STM32_FMC_SDSR) & 0x00000020;
    }

  putreg32(command, STM32_FMC_SDCMR);
  timeout = 0xffff;
  regval = getreg32(STM32_FMC_SDSR) & 0x00000020;
  while ((regval != 0) && timeout-- > 0)
    {
      regval = getreg32(STM32_FMC_SDSR) & 0x00000020;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdram_initialize
 *
 * Description:
 *   Called from stm32_bringup to initialize external SDRAM access.
 *
 ****************************************************************************/

void stm32_sdram_initialize(void)
{
  uint32_t regval;
  volatile int count;

  /* Enable GPIOs as FMC / memory pins */

  stm32_extmemgpios(g_addressconfig, STM32_FMC_NADDRCONFIGS);
  stm32_extmemgpios(g_dataconfig, STM32_FMC_NDATACONFIGS);

  /* Initialize the FMC peripheral */

  stm32_fmc_init();

  /* Configure and enable the SDRAM bank1
   *
   *   FMC clock = 180MHz/2 = 90MHz
   *   90MHz = 11,11 ns
   *   All timings from the datasheet for Speedgrade -7 (=7ns)
   */

  putreg32(FMC_SDCR_RPIPE_1 |
           FMC_SDCR_SDCLK_2X |
           FMC_SDCR_CASLAT_3 |
           FMC_SDCR_BANKS_4 |
           FMC_SDCR_WIDTH_16 |
           FMC_SDCR_ROWBITS_13 |
           FMC_SDCR_COLBITS_9,
      STM32_FMC_SDCR1);

  putreg32(FMC_SDCR_RPIPE_1 |
           FMC_SDCR_SDCLK_2X |
           FMC_SDCR_CASLAT_3 |
           FMC_SDCR_BANKS_4 |
           FMC_SDCR_WIDTH_16 |
           FMC_SDCR_ROWBITS_13 |
           FMC_SDCR_COLBITS_9,
      STM32_FMC_SDCR2);

  putreg32((2 << FMC_SDTR_TRCD_SHIFT) |  /* tRCD min = 15ns */
           (2 << FMC_SDTR_TRP_SHIFT) |   /* tRP  min = 15ns */
           (2 << FMC_SDTR_TWR_SHIFT) |   /* tWR      = 2CLK */
           (7 << FMC_SDTR_TRC_SHIFT) |   /* tRC  min = 63ns */
           (4 << FMC_SDTR_TRAS_SHIFT) |  /* tRAS min = 42ns */
           (7 << FMC_SDTR_TXSR_SHIFT) |  /* tXSR min = 70ns */
           (2 << FMC_SDTR_TMRD_SHIFT),   /* tMRD     = 2CLK */
      STM32_FMC_SDTR1);

  /* SDRAM Initialization sequence */

  stm32_sdramcommand(STM32_SDRAM_CLKEN);      /* Clock enable command */
  for (count = 0; count < 10000; count++);    /* Delay */
  stm32_sdramcommand(STM32_SDRAM_PALL);       /* Precharge ALL command */
  stm32_sdramcommand(STM32_SDRAM_REFRESH);    /* Auto refresh command */
  stm32_sdramcommand(STM32_SDRAM_MODEREG);    /* Mode Register program */

  /* Set refresh count
   *
   * FMC_CLK = 90MHz
   * Refresh_Rate = 7.81us
   * Counter = (FMC_CLK * Refresh_Rate) - 20
   */

  putreg32(683 << 1, STM32_FMC_SDRTR);

  /* Disable write protection */

  regval = getreg32(STM32_FMC_SDCR1);
  putreg32(regval & 0xfffffdff, STM32_FMC_SDCR1);
}

/****************************************************************************
 * Name: stm32_disablefmc
 *
 * Description:
 *  enable clocking to the FMC module
 *
 ****************************************************************************/

void stm32_disablefmc(void)
{
  uint32_t regval;

  /* Disable AHB clocking to the FMC */

  regval  = getreg32(STM32_RCC_AHB3ENR);
  regval &= ~RCC_AHB3ENR_FMCEN;
  putreg32(regval, STM32_RCC_AHB3ENR);
}
