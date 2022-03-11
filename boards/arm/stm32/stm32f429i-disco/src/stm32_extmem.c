/****************************************************************************
 * boards/arm/stm32/stm32f429i-disco/src/stm32_extmem.c
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
#include "stm32.h"
#include "stm32f429i-disco.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_FMC
#warning "FMC is not enabled"
#endif

#if STM32_NGPIO_PORTS < 6
#error "Required GPIO ports not enabled"
#endif

#define STM32_SDRAM_CLKEN     FMC_SDCMR_CMD_CLK_ENABLE | FMC_SDCMR_BANK_2

#define STM32_SDRAM_PALL      FMC_SDCMR_CMD_PALL | FMC_SDCMR_BANK_2

#define STM32_SDRAM_REFRESH   FMC_SDCMR_CMD_AUTO_REFRESH | FMC_SDCMR_BANK_2 |\
                                FMC_SDCMR_NRFS(4)

#define STM32_SDRAM_MODEREG   FMC_SDCMR_CMD_LOAD_MODE | FMC_SDCMR_BANK_2 |\
                                FMC_SDCMR_MDR_BURST_LENGTH_2 | \
                                FMC_SDCMR_MDR_BURST_TYPE_SEQUENTIAL |\
                                FMC_SDCMR_MDR_CAS_LATENCY_3 |\
                                FMC_SDCMR_MDR_WBL_SINGLE

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPIO configurations common to most external memories */

static const uint32_t g_sdram_config[] =
{
  /* 16 data lines */

  GPIO_FMC_D0, GPIO_FMC_D1, GPIO_FMC_D2, GPIO_FMC_D3,
  GPIO_FMC_D4, GPIO_FMC_D5, GPIO_FMC_D6, GPIO_FMC_D7,
  GPIO_FMC_D8, GPIO_FMC_D9, GPIO_FMC_D10, GPIO_FMC_D11,
  GPIO_FMC_D12, GPIO_FMC_D13, GPIO_FMC_D14, GPIO_FMC_D15,

  /* 12 address lines */

  GPIO_FMC_A0, GPIO_FMC_A1, GPIO_FMC_A2, GPIO_FMC_A3,
  GPIO_FMC_A4, GPIO_FMC_A5, GPIO_FMC_A6, GPIO_FMC_A7,
  GPIO_FMC_A8, GPIO_FMC_A9, GPIO_FMC_A10, GPIO_FMC_A11,

  /* control lines */

  GPIO_FMC_SDCKE1, GPIO_FMC_SDNE1, GPIO_FMC_SDNWE, GPIO_FMC_NBL0,
  GPIO_FMC_SDNRAS, GPIO_FMC_NBL1, GPIO_FMC_BA0, GPIO_FMC_BA1,
  GPIO_FMC_SDCLK, GPIO_FMC_SDNCAS,
};

#define NUM_SDRAM_GPIOS (sizeof(g_sdram_config) / sizeof(uint32_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  int i;
  volatile int count;

  /* Enable GPIOs as FMC / memory pins */

  for (i = 0; i < NUM_SDRAM_GPIOS; i++)
    {
      stm32_configgpio(g_sdram_config[i]);
    }

  /* Enable AHB clocking to the FMC */

  stm32_fmc_enable();

  /* Configure and enable the SDRAM bank1
   *
   *   FMC clock = 180MHz/2 = 90MHz
   *   90MHz = 11,11 ns
   *   All timings from the datasheet for Speedgrade -7 (=7ns)
   */

  val = FMC_SDCR_RPIPE_1 |      /* rpipe = 1 hclk */
    FMC_SDCR_SDCLK_2X |         /* sdclk = 2 hclk */
    FMC_SDCR_CAS_LATENCY_3 |    /* cas latency = 3 cycles */
    FMC_SDCR_NBANKS_4 |         /* 4 internal banks */
    FMC_SDCR_WIDTH_16 |         /* width = 16 bits */
    FMC_SDCR_ROWS_12 |          /* numrows = 12 */
    FMC_SDCR_COLS_8;            /* numcols = 8 bits */
  stm32_fmc_sdram_set_control(1, val);
  stm32_fmc_sdram_set_control(2, val);

  val = FMC_SDTR_TRCD(3) |      /* tRCD min = 15ns */
    FMC_SDTR_TRP(3) |           /* tRP  min = 15ns */
    FMC_SDTR_TWR(3) |           /* tWR      = 2CLK */
    FMC_SDTR_TRC(8) |           /* tRC  min = 63ns */
    FMC_SDTR_TRAS(5) |          /* tRAS min = 42ns */
    FMC_SDTR_TXSR(8) |          /* tXSR min = 70ns */
    FMC_SDTR_TMRD(3);           /* tMRD     = 2CLK */
  stm32_fmc_sdram_set_timing(2, val);

  /* SDRAM Initialization sequence */

  stm32_fmc_sdram_command(STM32_SDRAM_CLKEN);   /* Clock enable command */
  for (count = 0; count < 10000; count++);      /* Delay */
  stm32_fmc_sdram_command(STM32_SDRAM_PALL);    /* Precharge ALL command */
  stm32_fmc_sdram_command(STM32_SDRAM_REFRESH); /* Auto refresh command */
  stm32_fmc_sdram_command(STM32_SDRAM_MODEREG); /* Mode Register program */

  /* Set refresh count
   *
   * FMC_CLK = 90MHz
   * Refresh_Rate = 7.81us
   * Counter = (FMC_CLK * Refresh_Rate) - 20
   */

  stm32_fmc_sdram_set_refresh_rate(683);

  /* Disable write protection */

  /* stm32_fmc_sdram_write_protect(2, false); */
}
