/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/src/stm32_selectnor.c
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
#include "stm3210e-eval.h"

#ifdef CONFIG_STM32_FSMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin Usage (per schematic)
 *
 *                         FLASH   SRAM    NAND    LCD
 *   D[0..15]              [0..15] [0..15] [0..7]  [0..15]
 *   A[0..23]              [0..22] [0..18] [16,17] [0]
 *   FSMC_NBL0  PE0   OUT  ~BLE    ---     ---     ---
 *   FSMC_NBL1  PE1   OUT  ~BHE    ---     ---     ---
 *   FSMC_NE2   PG9   OUT  ---     ~E      ---     ---
 *   FSMC_NE3   PG10  OUT  ~CE     ---     ---     ---
 *   FSMC_NE4   PG12  OUT  ---     ---     ---     ~CS
 *   FSMC_NWE   PD5   OUT  ~WE     ~W      ~W      ~WR/SCL
 *   FSMC_NOE   PD4   OUT  ~OE     ~G      ~R      ~RD
 *   FSMC_NWAIT PD6   IN   ---     R~B     ---     ---
 *   FSMC_INT2  PG6*  IN   ---     ---     R~B     ---
 *
 *   *JP7 will switch to PD6
 */

/* GPIO configurations unique to NOR Flash  */

static const uint16_t g_norconfig[] =
{
  /* A19... A22 */

  GPIO_NPS_A19, GPIO_NPS_A20, GPIO_NPS_A21, GPIO_NPS_A22,

  /* NE2  */

  GPIO_NPS_NE2
};
#define NNOR_CONFIG (sizeof(g_norconfig)/sizeof(uint16_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_selectnor
 *
 * Description:
 *   Initialize to access NOR flash
 *
 ****************************************************************************/

void stm32_selectnor(void)
{
  /* Configure new GPIO state */

  stm32_extmemgpios(g_commonconfig, NCOMMON_CONFIG);
  stm32_extmemgpios(g_norconfig, NNOR_CONFIG);

  /* Enable AHB clocking to the FSMC */

  stm32_fsmc_enable();

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_NOR | FSMC_BCR_FACCEN |
           FSMC_BCR_MWID16 | FSMC_BCR_WREN,
           STM32_FSMC_BCR2);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(3) | FSMC_BTR_ADDHLD(1) |
           FSMC_BTR_DATAST(6) | FSMC_BTR_BUSTURN(1) |
           FSMC_BTR_CLKDIV(1) | FSMC_BTR_DATLAT(2) |
           FSMC_BTR_ACCMODB, STM32_FSMC_BTR2);

  putreg32(0x0fffffff, STM32_FSMC_BWTR2);

  /* Enable the bank */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_NOR |
           FSMC_BCR_FACCEN | FSMC_BCR_MWID16 |
           FSMC_BCR_WREN, STM32_FSMC_BCR2);
}
#endif /* CONFIG_STM32_FSMC */
