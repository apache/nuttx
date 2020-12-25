/****************************************************************************
 * boards/arm/stm32f7/stm32f769i-disco/src/stm32_extmem.c
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
#include "arm_arch.h"

#include "stm32_fmc.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "stm32f769i-disco.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32F7_FMC
#  warning "FMC is not enabled"
#endif

#if STM32F7_NGPIO < 8
#  error "Required GPIO ports not enabled"
#endif

#define STM32_FMC_NADDRCONFIGS 25
#define STM32_FMC_NDATACONFIGS 32

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

  GPIO_FMC_NBL0, GPIO_FMC_NBL1, GPIO_FMC_NBL2, GPIO_FMC_NBL3, GPIO_FMC_BA0,
  GPIO_FMC_BA1, GPIO_FMC_SDNWE_3, GPIO_FMC_SDNCAS, GPIO_FMC_SDNRAS,
  GPIO_FMC_SDNE0_3, GPIO_FMC_SDCKE0_3, GPIO_FMC_SDCLK
};

static const uint32_t g_dataconfig[STM32_FMC_NDATACONFIGS] =
{
  GPIO_FMC_D0, GPIO_FMC_D1, GPIO_FMC_D2, GPIO_FMC_D3, GPIO_FMC_D4,
  GPIO_FMC_D5, GPIO_FMC_D6, GPIO_FMC_D7, GPIO_FMC_D8, GPIO_FMC_D9,
  GPIO_FMC_D10, GPIO_FMC_D11, GPIO_FMC_D12, GPIO_FMC_D13, GPIO_FMC_D14,
  GPIO_FMC_D15, GPIO_FMC_D16, GPIO_FMC_D17, GPIO_FMC_D18, GPIO_FMC_D19,
  GPIO_FMC_D20, GPIO_FMC_D21, GPIO_FMC_D22, GPIO_FMC_D23, GPIO_FMC_D24,
  GPIO_FMC_D25, GPIO_FMC_D26, GPIO_FMC_D27, GPIO_FMC_D28, GPIO_FMC_D29,
  GPIO_FMC_D30, GPIO_FMC_D31
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
  uint32_t val;
  volatile int count;

  /* Enable GPIOs as FMC / memory pins */

  stm32_extmemgpios(g_addressconfig, STM32_FMC_NADDRCONFIGS);
  stm32_extmemgpios(g_dataconfig, STM32_FMC_NDATACONFIGS);

  /* Initialize the FMC peripheral */

  stm32_fmc_init();

  /* Configure and enable the SDRAM bank1
   *
   *   FMC clock = 216MHz/2 = 108MHz
   *   108MHz = 9,26 ns
   *   All timings from the datasheet for Speedgrade -6A (=6ns)
   */

  val = FMC_SDCR_RPIPE_0 |      /* rpipe = 0 hclk */
    FMC_SDCR_BURST_READ |       /* enable burst read */
    FMC_SDCR_SDCLK_2X |         /* sdclk = 2 hclk */
    FMC_SDCR_CASLAT_3 |         /* cas latency = 3 cycles */
    FMC_SDCR_BANKS_4 |          /* 4 internal banks */
    FMC_SDCR_WIDTH_32 |         /* width = 32 bits */
    FMC_SDCR_ROWBITS_12 |       /* numrows = 12 bits */
    FMC_SDCR_COLBITS_8;         /* numcols = 8 bits */
  stm32_fmc_sdram_set_control(1, val);

  val = FMC_SDTR_TRCD(2) |      /* tRCD min = 18ns */
    FMC_SDTR_TRP(2) |           /* tRP  min = 18ns */
    FMC_SDTR_TWR(3) |           /* tWR      = 3CLK */
    FMC_SDTR_TRC(7) |           /* tRC  min = 64ns */
    FMC_SDTR_TRAS(4) |          /* tRAS min = 37ns */
    FMC_SDTR_TXSR(7) |          /* tXSR min = 64ns */
    FMC_SDTR_TMRD(2);           /* tMRD     = 2CLK */
  stm32_fmc_sdram_set_timing(1, val);

  /* SDRAM Initialization sequence */

  stm32_fmc_sdram_command(STM32_SDRAM_CLKEN);      /* Clock enable command */

  for (count = 0; count < 10000; count++);         /* Delay */

  stm32_fmc_sdram_command(STM32_SDRAM_PALL);       /* Precharge ALL command */
  stm32_fmc_sdram_command(STM32_SDRAM_REFRESH);    /* Auto refresh command */
  stm32_fmc_sdram_command(STM32_SDRAM_MODEREG);    /* Mode Register program */

  /* Set refresh count
   *
   * FMC_CLK = 108MHz
   * Refresh_Rate = 64ms / 4096 rows = 15.63us
   * Counter = (FMC_CLK * Refresh_Rate) - 20
   */

  stm32_fmc_sdram_set_refresh_rate(1668);
}
