/****************************************************************************
 * boards/arm/stm32/axoloti/src/stm32_extmem.c
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
#include "axoloti.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_FMC
#warning "FMC is not enabled"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Axoloti SDRAM GPIO configuration */

static const uint32_t g_sdram_config[] =
{
  /* Data lines */

  GPIO_FMC_D0, GPIO_FMC_D1, GPIO_FMC_D2, GPIO_FMC_D3,
  GPIO_FMC_D4, GPIO_FMC_D5, GPIO_FMC_D6, GPIO_FMC_D7,
  GPIO_FMC_D8, GPIO_FMC_D9, GPIO_FMC_D10, GPIO_FMC_D11,
  GPIO_FMC_D12, GPIO_FMC_D13, GPIO_FMC_D14, GPIO_FMC_D15,

  /* Address lines */

  GPIO_FMC_A0, GPIO_FMC_A1, GPIO_FMC_A2, GPIO_FMC_A3,
  GPIO_FMC_A4, GPIO_FMC_A5, GPIO_FMC_A6, GPIO_FMC_A7,
  GPIO_FMC_A8, GPIO_FMC_A9, GPIO_FMC_A10, GPIO_FMC_A11,
  GPIO_FMC_A12,

  /* Control lines */

  GPIO_FMC_BA0,                 /* ba0 */
  GPIO_FMC_BA1,                 /* ba1 */
  GPIO_FMC_NBL0,                /* ldqm */
  GPIO_FMC_NBL1,                /* udqm */
  GPIO_FMC_SDCLK,               /* clk */
  GPIO_FMC_SDCKE0_1,            /* cke */
  GPIO_FMC_SDNWE_2,             /* we */
  GPIO_FMC_SDNCAS,              /* cas */
  GPIO_FMC_SDNRAS,              /* ras */
  GPIO_FMC_SDNE0_1,             /* cs0 */
  GPIO_FMC_SDNE1_2,             /* cs1 */
};

#define NUM_SDRAM_GPIOS (sizeof(g_sdram_config) / sizeof(uint32_t))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdram_memtest
 *
 * Description:
 *  Test the SDRAM.
 *
 ****************************************************************************/

#define RAND_A 22695477
#define RAND_C 1
#define TEST_ITERATIONS 16

int stm32_sdram_memtest(void *base, uint32_t size)
{
  volatile int iter;
  volatile int i;

  /* Linear write with linear congruential generator values */

  for (iter = 0; iter < TEST_ITERATIONS; iter++)
    {
      uint32_t x = iter;

      /* Write */

      for (i = 0; i < size / 4; i++)
        {
          x = (RAND_A * x) + RAND_C;
          ((volatile uint32_t *)base)[i] = x;
        }

      /* Read/verify */

      x = iter;
      for (i = 0; i < size / 4; i++)
        {
          x = (RAND_A * x) + RAND_C;
          if (((volatile uint32_t *)base)[i] != x)
            {
              return -1;
            }
        }
    }

  /* Scattered byte write at linear congruential generator addresses */

  for (iter = 0; iter < TEST_ITERATIONS; iter++)
    {
      uint32_t x = iter;

      /* Write */

      for (i = 0; i < 1024 * 1024; i++)
        {
          x = (RAND_A * x) + RAND_C;
          ((volatile uint8_t *)base)[x & (size - 1)] = (uint8_t) i;
        }

      /* Read/verify */

      x = iter;
      for (i = 0; i < 1024 * 1024; i++)
        {
          x = (RAND_A * x) + RAND_C;
          if (((volatile uint8_t *)base)[x & (size - 1)] != (uint8_t) i)
            {
              return -1;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_sdram_initialize
 *
 * Description:
 *   Called from stm32_bringup to initialize external SDRAM access.
 *   The Axoloti uses an Alliance Memory AS4C4M16SA SDRAM.
 *
 ****************************************************************************/

int stm32_sdram_initialize(void)
{
  uint32_t val;
  int i;

  /* Configure SDRAM GPIOs */

  for (i = 0; i < NUM_SDRAM_GPIOS; i++)
    {
      stm32_configgpio(g_sdram_config[i]);
    }

  /* Enable the FMC */

  stm32_fmc_enable();

  /* Go through the SDRAM initialization steps per the reference manual.
   * The sdclk period is set to 2 x hclk. That is: 168 /2 = 84 MHz
   * This gives a clock period of about 11.9 ns
   */

  /* Step 1:
   * Program the memory device features into the FMC_SDCRx register. The
   * SDRAM clock frequency, RBURST and RPIPE must be programmed in the
   * FMC_SDCR1 register.
   */

  val = FMC_SDCR_RPIPE_1 |      /* rpipe = 1 hclk */
    FMC_SDCR_READBURST |        /* read burst enabled */
    FMC_SDCR_SDCLK_2X |         /* sdclk = 2 hclk */
    FMC_SDCR_CAS_LATENCY_2 |    /* cas latency = 2 cycles */
    FMC_SDCR_NBANKS_4 |         /* 4 internal banks */
    FMC_SDCR_WIDTH_16 |         /* width = 16 bits */
    FMC_SDCR_ROWS_12 |          /* numrows = 12 */
    FMC_SDCR_COLS_8;            /* numcols = 8 bits */
  stm32_fmc_sdram_set_control(1, val);

  /* Step 2:
   * Program the memory device timing into the FMC_SDTRx register.  The
   * TRP and TRC timings must be programmed in the FMC_SDTR1 register.
   */

  val = FMC_SDTR_TRCD(2) |      /* ras to cas delay 21ns => 2x11.90ns */
    FMC_SDTR_TRP(2) |           /* row precharge 21ns => 2x11.90ns */
    FMC_SDTR_TRC(6) |           /* row cycle time 63ns => 6x11.9ns */
    FMC_SDTR_TRAS(4) |          /* row active time 42ns = >4x11.9ns */
    FMC_SDTR_TWR(4) |           /* write to precharge 42ns => 4x11.9ns */
    FMC_SDTR_TXSR(6) |          /* exit self refresh 65ns => 6x11.9ns */
    FMC_SDTR_TMRD(2);           /* load mode register to active 2 clks */
  stm32_fmc_sdram_set_timing(1, val);

  /* Step 3:
   * Set MODE bits to ‘001’ and configure the Target Bank bits (CTB1
   * and/or CTB2) in the FMC_SDCMR register to start delivering the clock
   * to the memory (SDCKE is driven high).
   */

  val = FMC_SDCMR_BANK_1 | FMC_SDCMR_CMD_CLK_ENABLE;
  stm32_fmc_sdram_command(val);

  /* Step 4:
   * Wait during the prescribed delay period. Typical delay is around 100
   * μs (refer to the SDRAM datasheet for the required delay after
   * power-up).
   */

  nxsig_usleep(1000);

  /* Step 5:
   * Set MODE bits to ‘010’ and configure the Target Bank bits (CTB1
   * and/or CTB2) in the FMC_SDCMR register to issue a “Precharge All”
   * command.
   */

  val = FMC_SDCMR_BANK_1 | FMC_SDCMR_CMD_PALL;
  stm32_fmc_sdram_command(val);

  /* Step 6:
   * Set MODE bits to ‘011’, and configure the Target Bank bits (CTB1
   * and/or CTB2) as well as the number of consecutive Auto-refresh
   * commands (NRFS) in the FMC_SDCMR register. Refer to the SDRAM
   * datasheet for the number of Auto-refresh commands that should be
   * issued. Typical number is 8.
   */

  val = FMC_SDCMR_NRFS(5) | FMC_SDCMR_BANK_1 | FMC_SDCMR_CMD_AUTO_REFRESH;
  stm32_fmc_sdram_command(val);

  /* Step 7:
   * Configure the MRD field according to your SDRAM device, set the MODE
   * bits to '100', and configure the Target Bank bits (CTB1 and/or CTB2)
   * in the FMC_SDCMR register to issue a "Load Mode Register" command in
   * order to program the SDRAM. In particular:
   * a) The CAS latency must be selected following configured value in
   *    FMC_SDCR1/2 registers
   * b) The Burst Length (BL) of 1 must be selected by configuring the
   *    M[2:0] bits to 000 in the mode register (refer to the SDRAM
   *    datasheet). If the Mode Register is not the same for both SDRAM
   *    banks, this step has to be repeated twice, once for each bank,
   *    and the Target Bank bits set accordingly.
   */

  val = FMC_SDCMR_MDR_BURST_LENGTH_2 |
    FMC_SDCMR_MDR_BURST_TYPE_SEQUENTIAL |
    FMC_SDCMR_MDR_CAS_LATENCY_2 |
    FMC_SDCMR_MDR_MODE_NORMAL |
    FMC_SDCMR_MDR_WBL_SINGLE | FMC_SDCMR_BANK_1 | FMC_SDCMR_CMD_LOAD_MODE;
  stm32_fmc_sdram_command(val);

  /* Step 8:
   * Program the refresh rate in the FMC_SDRTR register
   * The refresh rate corresponds to the delay between refresh cycles. Its
   * value must be adapted to SDRAM devices.
   */

  stm32_fmc_sdram_set_refresh_rate(1292);       /* (64ms/4096rows) x 84MHz) - 20 */

  /* Step 9:
   * For mobile SDRAM devices, to program the extended mode register it
   * should be done once the SDRAM device is initialized: First, a dummy
   * read access should be performed while BA1=1 and BA=0 (refer to SDRAM
   * address mapping section for BA[1:0] address mapping) in order to select
   * the extended mode register instead of Load mode register and then
   * program the needed value.
   */

  /* Setting EMRS is optional and we're not bothering ... */

  /* Enable memory writes for bank 1 */

  stm32_fmc_sdram_write_protect(1, false);

  /* Wait for the controller to be ready */

  stm32_fmc_sdram_wait();
  return OK;
}
