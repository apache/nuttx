/****************************************************************************
 * boards/arm/stm32/stm3220g-eval/src/stm32_selectlcd.c
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
#include <debug.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "stm3220g-eval.h"

#ifdef CONFIG_STM32_FSMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/* SRAM pin definitions */

#define LCD_NADDRLINES   1
#define LCD_NDATALINES   16

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/*   Pin Usage (per schematic)
 *                         SRAM       LCD
 *   D[0..15]              [0..15]    [0..15]
 *   A[0..25]              [0..22]    [0] RS
 *   FSMC_NBL0  PE0   OUT  ---        ---
 *   FSMC_NBL1  PE1   OUT  ---        ---
 *   FSMC_NE2   PG9   OUT  ---        ---
 *   FSMC_NE3   PG10  OUT  ---        ~CS
 *   FSMC_NE4   PG12  OUT  ---        ---
 *   FSMC_NWE   PD5   OUT  ---        ~WR/SCL
 *   FSMC_NOE   PD4   OUT  ---        ~RD
 *   FSMC_NWAIT PD6   IN   ---        ---
 *   FSMC_INT2  PG6*  IN   ---        ---
 *   FSMC_INT3
 *   FSMC_INTR
 *   FSMC_CD
 *   FSMC_CLK
 *   FSMC_NCE2
 *   FSMC_NCE3
 *   FSMC_NCE4_1
 *   FSMC_NCE4_2
 *   FSMC_NIORD
 *   FSMC_NIOWR
 *   FSMC_NL
 *   FSMC_NREG
 */

/* GPIO configurations unique to the LCD  */

static const uint32_t g_lcdconfig[] =
{
  /* NOE, NWE, and NE3  */

  GPIO_FSMC_NOE, GPIO_FSMC_NWE, GPIO_FSMC_NE3
};
#define NLCD_CONFIG (sizeof(g_lcdconfig)/sizeof(uint32_t))

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
 *   Initialize to the LCD
 *
 ****************************************************************************/

void stm32_selectlcd(void)
{
  /* Configure new GPIO pins */

  stm32_extmemaddr(LCD_NADDRLINES);             /* Common address lines: A0 */
  stm32_extmemdata(LCD_NDATALINES);             /* Common data lines: D0-D15 */
  stm32_extmemgpios(g_lcdconfig, NLCD_CONFIG);  /* LCD-specific control lines */

  /* Enable AHB clocking to the FSMC */

  stm32_fsmc_enable();

  /* Color LCD configuration (LCD configured as follow):
   *
   *   - Data/Address MUX  = Disable   "FSMC_BCR_MUXEN" just not enable it.
   *   - Extended Mode     = Disable   "FSMC_BCR_EXTMOD"
   *   - Memory Type       = SRAM      "FSMC_BCR_SRAM"
   *   - Data Width        = 16bit     "FSMC_BCR_MWID16"
   *   - Write Operation   = Enable    "FSMC_BCR_WREN"
   *   - Asynchronous Wait = Disable
   */

  /* Bank3 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR3);

  /* Bank3 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(5) | FSMC_BTR_ADDHLD(0) |
           FSMC_BTR_DATAST(9) | FSMC_BTR_BUSTURN(0) |
           FSMC_BTR_CLKDIV(0) | FSMC_BTR_DATLAT(0) |
           FSMC_BTR_ACCMODA, STM32_FSMC_BTR3);

  putreg32(0xffffffff, STM32_FSMC_BWTR3);

  /* Enable the bank by setting the MBKEN bit */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM |
           FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR3);
}

#endif /* CONFIG_STM32_FSMC */
