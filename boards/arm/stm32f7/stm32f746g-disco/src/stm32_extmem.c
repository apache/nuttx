/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_extmem.c
 *
 *   Copyright (C) 2018 Marcin Wyrwas. All rights reserved.
 *   Author: Marcin Wyrwas <mvp1@wp.pl>
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "arm_arch.h"

#include "stm32_fmc.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "stm32f746g-disco.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32F7_FMC
#  warning "FMC is not enabled"
#endif

#if STM32F7_NGPIO < 7
#  error "Required GPIO ports not enabled"
#endif

#define STM32_FMC_NADDRCONFIGS 22
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
  GPIO_FMC_A10, GPIO_FMC_A11,

  GPIO_FMC_SDCKE0_1, GPIO_FMC_SDNE0_3, GPIO_FMC_SDNWE_3, GPIO_FMC_NBL0,
  GPIO_FMC_SDNRAS, GPIO_FMC_NBL1,  GPIO_FMC_BA0,   GPIO_FMC_BA1,
  GPIO_FMC_SDCLK,  GPIO_FMC_SDNCAS
};

static const uint32_t g_dataconfig[STM32_FMC_NDATACONFIGS] =
{
  GPIO_FMC_D0, GPIO_FMC_D1, GPIO_FMC_D2, GPIO_FMC_D3, GPIO_FMC_D4,
  GPIO_FMC_D5, GPIO_FMC_D6, GPIO_FMC_D7, GPIO_FMC_D8, GPIO_FMC_D9,
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage
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
 *
 ****************************************************************************/

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
 * Name: stm32_enablefmc
 *
 * Description:
 *  enable clocking to the FMC module
 *
 ****************************************************************************/

void stm32_enablefmc(void)
{
  uint32_t regval;
  volatile int count;

  /* Enable GPIOs as FMC / memory pins */

  stm32_extmemgpios(g_addressconfig, STM32_FMC_NADDRCONFIGS);
  stm32_extmemgpios(g_dataconfig, STM32_FMC_NDATACONFIGS);

  /* Enable AHB clocking to the FMC */

  regval  = getreg32(STM32_RCC_AHB3ENR);
  regval |= RCC_AHB3ENR_FMCEN;
  putreg32(regval, STM32_RCC_AHB3ENR);

  /* Configure and enable the SDRAM bank1
   *
   *   FMC clock = 216MHz/2 = 108MHz
   *   108MHz = 9,26 ns
   *   All timings from the datasheet for Speedgrade -6A (=6ns)
   */

  putreg32(FMC_SDCR_RPIPE_0 |
           FMC_SDCR_BURST_READ |
           FMC_SDCR_SDCLK_2X |
           FMC_SDCR_CASLAT_3 |
           FMC_SDCR_BANKS_4 |
           FMC_SDCR_WIDTH_16 |
           FMC_SDCR_ROWBITS_12 |
           FMC_SDCR_COLBITS_8,
      STM32_FMC_SDCR1);

  putreg32(FMC_SDTR_TRCD(2) |          /* tRCD min = 18ns */
           FMC_SDTR_TRP(2) |           /* tRP  min = 18ns */
           FMC_SDTR_TWR(2) |           /* tWR      = 2CLK */
           FMC_SDTR_TRC(7) |           /* tRC  min = 64ns */
           FMC_SDTR_TRAS(5) |          /* tRAS min = 46ns */
           FMC_SDTR_TXSR(8) |          /* tXSR min = 74ns */
           FMC_SDTR_TMRD(2),           /* tMRD     = 2CLK */
      STM32_FMC_SDTR1);

  /* SDRAM Initialization sequence */

  stm32_sdramcommand(STM32_SDRAM_CLKEN);      /* Clock enable command */
  for (count = 0; count < 10000; count++) ;   /* Delay */
  stm32_sdramcommand(STM32_SDRAM_PALL);       /* Precharge ALL command */
  stm32_sdramcommand(STM32_SDRAM_REFRESH);    /* Auto refresh command */
  stm32_sdramcommand(STM32_SDRAM_MODEREG);    /* Mode Register program */

  /* Set refresh count
   *
   * FMC_CLK = 108MHz
   * Refresh_Rate = 64ms / 4096 rows = 15.63us
   * Counter = (FMC_CLK * Refresh_Rate) - 20
   */

  putreg32(1668 << 1, STM32_FMC_SDRTR);
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
  regval &= ~(uint32_t)RCC_AHB3ENR_FMCEN;
  putreg32(regval, STM32_RCC_AHB3ENR);
}
