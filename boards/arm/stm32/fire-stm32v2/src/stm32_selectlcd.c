/****************************************************************************
 * boards/arm/stm32/fire-stm32v2/src/stm32_selectlcd.c
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

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"

#include "stm32_gpio.h"
#include "stm32.h"
#include "fire-stm32v2.h"

#ifdef CONFIG_STM32_FSMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_FSMC
#  warning "FSMC is not enabled"
#endif

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* 2.4" TFT + Touchscreen.  FSMC Bank1
 *
 * --- ------ -------------- ------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- ------------------------------------------------
 *
 * 30  PA5    PA5-SPI1-SCK   2.4" TFT + Touchscreen, 10Mbit ENC28J60,
 *                                            SPI 2M FLASH
 * 31  PA6    PA6-SPI1-MISO  2.4" TFT + Touchscreen, 10Mbit ENC28J60,
 *                                            SPI 2M FLASH
 * 32  PA7    PA7-SPI1-MOSI  2.4" TFT + Touchscreen, 10Mbit ENC28J60,
 *                                            SPI 2M FLASH
 * 92  PB6    PB6-I2C1-SCL   2.4" TFT + Touchscreen, AT24C02
 * 93  PB7    PB7-I2C1-SDA   2.4" TFT + Touchscreen, AT24C02
 * 81  PD0    PD0-FSMC_D2    2.4" TFT + Touchscreen
 * 82  PD1    PD1-FSMC_D3    2.4" TFT + Touchscreen
 * 85  PD4    PD4-FSMC_NOE   2.4" TFT + Touchscreen
 * 86  PD5    PD5-FSMC_NWE   2.4" TFT + Touchscreen
 * 88  PD7    PD7-FSMC_NE1   2.4" TFT + Touchscreen
 * 55  PD8    PD8-FSMC_D13   2.4" TFT + Touchscreen
 * 56  PD9    PD9-FSMC_D14   2.4" TFT + Touchscreen
 * 57  PD10   PD10-FSMC_D15  2.4" TFT + Touchscreen
 * 58  PD11   PD11-FSMC_A16  2.4" TFT + Touchscreen
 * 60  PD13   PD13-LCD/LIGHT 2.4" TFT + Touchscreen
 * 61  PD14   PD14-FSMC_D0   2.4" TFT + Touchscreen
 * 62  PD15   PD15-FSMC_D1   2.4" TFT + Touchscreen
 * 98  PE1    PE1-FSMC_NBL1  2.4" TFT + Touchscreen
 * 38  PE7    PE7-FSMC_D4    2.4" TFT + Touchscreen
 * 39  PE8    PE8-FSMC_D5    2.4" TFT + Touchscreen
 * 40  PE9    PE9-FSMC_D6    2.4" TFT + Touchscreen
 * 41  PE10   PE10-FSMC_D7   2.4" TFT + Touchscreen
 * 42  PE11   PE11-FSMC_D8   2.4" TFT + Touchscreen
 * 43  PE12   PE12-FSMC_D9   2.4" TFT + Touchscreen
 * 44  PE13   PE13-FSMC_D10  2.4" TFT + Touchscreen
 * 45  PE14   PE14-FSMC_D11  2.4" TFT + Touchscreen
 * 46  PE15   PE15-FSMC_D12  2.4" TFT + Touchscreen
 *
 * NOTE:
 * SPI and I2C pin configuration is controlled in the SPI and I2C drivers,
 * respectively.
 */

static const uint16_t g_lcdconfig[NCOMMON_CONFIG] =
{
  /* Address Lines:  A16 only */

  GPIO_NPS_A16,

  /* Data Lines: D0... D15 */

  GPIO_NPS_D0,  GPIO_NPS_D1,  GPIO_NPS_D2,  GPIO_NPS_D3,
  GPIO_NPS_D4,  GPIO_NPS_D5,  GPIO_NPS_D6,  GPIO_NPS_D7,
  GPIO_NPS_D8,  GPIO_NPS_D9,  GPIO_NPS_D10, GPIO_NPS_D11,
  GPIO_NPS_D12, GPIO_NPS_D13, GPIO_NPS_D14, GPIO_NPS_D15,

  /* NOE, NWE, NE1, NBL1 */

  GPIO_NPS_NOE, GPIO_NPS_NWE, GPIO_NPS_NE1, GPIO_NPS_NBL1,

  /* Backlight GPIO */

  GPIO_LCD_BACKLIGHT
};
#define NLCD_CONFIG (sizeof(g_lcdconfig) / sizeof(uint16_t))

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
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD pin configuration.
 *
 ****************************************************************************/

void stm32_selectlcd(void)
{
  irqstate_t flags;
  int i;

  /* Configure LCD GPIO pis */

  flags = enter_critical_section();
  for (i = 0; i < NLCD_GPIOS; i++)
    {
      stm32_configgpio(g_lcdconfig[i]);
    }

  /* Enable AHB clocking to the FSMC */

  stm32_fsmc_enable();

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(1) | FSMC_BTR_ADDHLD(0) |
           FSMC_BTR_DATAST(2) | FSMC_BTR_BUSTURN(0) |
           FSMC_BTR_CLKDIV(0) | FSMC_BTR_DATLAT(0) |
           FSMC_BTR_ACCMODA, STM32_FSMC_BTR1);

  putreg32(0xffffffff, STM32_FSMC_BWTR4);

  /* Enable the bank by setting the MBKEN bit */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM |
           FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);
  leave_critical_section(flags);
}

#endif /* CONFIG_STM32_FSMC */
