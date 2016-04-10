/************************************************************************************
 * configs/stm32f429i-disco/src/stm32_extmem.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32_fsmc.h"
#include "stm32_gpio.h"
#include "stm32.h"
#include "stm32f429i-disco.h"

#include <arch/board/board.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_STM32_FSMC
#  warning "FSMC is not enabled"
#endif

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

#define STM32_FSMC_NADDRCONFIGS 22
#define STM32_FSMC_NDATACONFIGS 16

#define STM32_SDRAM_CLKEN     FSMC_SDRAM_MODE_CMD_CLK_ENABLE | FSMC_SDRAM_CMD_BANK_2
#define STM32_SDRAM_PALL      FSMC_SDRAM_MODE_CMD_PALL | FSMC_SDRAM_CMD_BANK_2
#define STM32_SDRAM_REFRESH   FSMC_SDRAM_MODE_CMD_AUTO_REFRESH | FSMC_SDRAM_CMD_BANK_2 |\
                                (3 << FSMC_SDRAM_AUTO_REFRESH_SHIFT)
#define STM32_SDRAM_MODEREG   FSMC_SDRAM_MODE_CMD_LOAD_MODE | FSMC_SDRAM_CMD_BANK_2 |\
                                FSMC_SDRAM_MODEREG_BURST_LENGTH_2 | \
                                FSMC_SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL |\
                                FSMC_SDRAM_MODEREG_CAS_LATENCY_3 |\
                                FSMC_SDRAM_MODEREG_WRITEBURST_MODE_SINGLE


/************************************************************************************
 * Public Data
 ************************************************************************************/

/* GPIO configurations common to most external memories */

static const uint32_t g_addressconfig[STM32_FSMC_NADDRCONFIGS] =
{
  GPIO_FSMC_A0,  GPIO_FSMC_A1 , GPIO_FSMC_A2,  GPIO_FSMC_A3,  GPIO_FSMC_A4 , GPIO_FSMC_A5,
  GPIO_FSMC_A6,  GPIO_FSMC_A7,  GPIO_FSMC_A8,  GPIO_FSMC_A9,  GPIO_FSMC_A10, GPIO_FSMC_A11,

  GPIO_FSMC_SDCKE1, GPIO_FSMC_SDNE1, GPIO_FSMC_SDNWE, GPIO_FSMC_NBL0,
  GPIO_FSMC_SDNRAS, GPIO_FSMC_NBL1,  GPIO_FSMC_BA0,   GPIO_FSMC_BA1,
  GPIO_FSMC_SDCLK,  GPIO_FSMC_SDNCAS
};

static const uint32_t g_dataconfig[STM32_FSMC_NDATACONFIGS] =
{
  GPIO_FSMC_D0,  GPIO_FSMC_D1 , GPIO_FSMC_D2,  GPIO_FSMC_D3,  GPIO_FSMC_D4 , GPIO_FSMC_D5,
  GPIO_FSMC_D6,  GPIO_FSMC_D7,  GPIO_FSMC_D8,  GPIO_FSMC_D9,  GPIO_FSMC_D10, GPIO_FSMC_D11,
  GPIO_FSMC_D12, GPIO_FSMC_D13, GPIO_FSMC_D14, GPIO_FSMC_D15
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage
 *
 ************************************************************************************/

static void stm32_extmemgpios(const uint32_t *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < ngpios; i++)
    {
      stm32_configgpio(gpios[i]);
    }
}

/************************************************************************************
 * Name: stm32_sdramcommand
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 *
 ************************************************************************************/

static void stm32_sdramcommand(uint32_t command)
{
  uint32_t  regval;
  volatile  uint32_t timeout = 0xFFFF;

  regval = getreg32( STM32_FSMC_SDSR ) & 0x00000020;
  while ((regval != 0) && timeout-- > 0)
    {
      regval = getreg32( STM32_FSMC_SDSR ) & 0x00000020;
    }
  putreg32(command, STM32_FSMC_SDCMR);
  timeout = 0xFFFF;
  regval = getreg32( STM32_FSMC_SDSR ) & 0x00000020;
  while ((regval != 0) && timeout-- > 0)
    {
      regval = getreg32( STM32_FSMC_SDSR ) & 0x00000020;
    }
}

/************************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

void stm32_enablefsmc(void)
{
  uint32_t regval;
  volatile int count;

  /* Enable GPIOs as FSMC / memory pins */

  stm32_extmemgpios(g_addressconfig, STM32_FSMC_NADDRCONFIGS);
  stm32_extmemgpios(g_dataconfig, STM32_FSMC_NDATACONFIGS);

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHB3ENR);
  regval |= RCC_AHB3ENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHB3ENR);

  /* Configure and enable the SDRAM bank1
   *
   *   FMC clock = 180MHz/2 = 90MHz
   *   90MHz = 11,11 ns
   *   All timings from the datasheet for Speedgrade -7 (=7ns)
   */

  putreg32(FSMC_SDRAM_CR_RPIPE_1 |
           FSMC_SDRAM_CR_SDCLK_2X |
           FSMC_SDRAM_CR_CASLAT_3 |
           FSMC_SDRAM_CR_BANKS_4 |
           FSMC_SDRAM_CR_WIDTH_16 |
           FSMC_SDRAM_CR_ROWBITS_12 |
           FSMC_SDRAM_CR_COLBITS_8,
      STM32_FSMC_SDCR1);

  putreg32(FSMC_SDRAM_CR_RPIPE_1 |
           FSMC_SDRAM_CR_SDCLK_2X |
           FSMC_SDRAM_CR_CASLAT_3 |
           FSMC_SDRAM_CR_BANKS_4 |
           FSMC_SDRAM_CR_WIDTH_16 |
           FSMC_SDRAM_CR_ROWBITS_12 |
           FSMC_SDRAM_CR_COLBITS_8,
      STM32_FSMC_SDCR2);

  putreg32((2 << FSMC_SDRAM_TR_TRCD_SHIFT) |  /* tRCD min = 15ns */
           (2 << FSMC_SDRAM_TR_TRP_SHIFT) |   /* tRP  min = 15ns */
           (2 << FSMC_SDRAM_TR_TWR_SHIFT) |   /* tWR      = 2CLK */
           (7 << FSMC_SDRAM_TR_TRC_SHIFT) |   /* tRC  min = 63ns */
           (4 << FSMC_SDRAM_TR_TRAS_SHIFT) |  /* tRAS min = 42ns */
           (7 << FSMC_SDRAM_TR_TXSR_SHIFT) |  /* tXSR min = 70ns */
           (2 << FSMC_SDRAM_TR_TMRD_SHIFT),   /* tMRD     = 2CLK */
      STM32_FSMC_SDTR2);

  /* SDRAM Initialization sequence */

  stm32_sdramcommand(STM32_SDRAM_CLKEN);      /* Clock enable command */
  for (count = 0; count < 10000; count++) ;    /* Delay */
  stm32_sdramcommand(STM32_SDRAM_PALL);       /* Precharge ALL command */
  stm32_sdramcommand(STM32_SDRAM_REFRESH);    /* Auto refresh command */
  stm32_sdramcommand(STM32_SDRAM_MODEREG);    /* Mode Register program */

  /* Set refresh count
   *
   * FMC_CLK = 90MHz
   * Refresh_Rate = 7.81us
   * Counter = (FMC_CLK * Refresh_Rate) - 20
   */

  putreg32(683 << 1, STM32_FSMC_SDRTR);

  /* Disable write protection */

//  regval = getreg32(STM32_FSMC_SDCR2);
//  putreg32(regval & 0xFFFFFDFF, STM32_FSMC_SDCR2);
}

/************************************************************************************
 * Name: stm32_disablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

void stm32_disablefsmc(void)
{
  uint32_t regval;

  /* Disable AHB clocking to the FSMC */

  regval  = getreg32(STM32_RCC_AHB3ENR);
  regval &= ~RCC_AHB3ENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHB3ENR);
}
