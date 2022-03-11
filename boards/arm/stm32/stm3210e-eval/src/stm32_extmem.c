/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/src/stm32_extmem.c
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
#include "stm32_gpio.h"
#include "stm32.h"
#include "stm3210e-eval.h"

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

/* 512Kx16 SRAM is connected to bank2 of the FSMC interface and both 8- and
 * 16-bit accesses are allowed by BLN0 and BLN1 connected to BLE and BHE of
 * SRAM, respectively.
 *
 * Pin Usage (per schematic)
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

/* It would be much more efficient to brute force these all into the
 * the appropriate registers.  Just a little tricky.
 */

/* GPIO configurations common to SRAM and NOR Flash */

const uint16_t g_commonconfig[NCOMMON_CONFIG] =
{
  /* A0... A18 */

  GPIO_NPS_A0,  GPIO_NPS_A1,  GPIO_NPS_A2,  GPIO_NPS_A3,
  GPIO_NPS_A4,  GPIO_NPS_A5,  GPIO_NPS_A6,  GPIO_NPS_A7,
  GPIO_NPS_A8,  GPIO_NPS_A9,  GPIO_NPS_A10, GPIO_NPS_A11,
  GPIO_NPS_A12, GPIO_NPS_A13, GPIO_NPS_A14, GPIO_NPS_A15,
  GPIO_NPS_A16, GPIO_NPS_A17, GPIO_NPS_A18,

  /* D0... D15 */

  GPIO_NPS_D0,  GPIO_NPS_D1,  GPIO_NPS_D2,  GPIO_NPS_D3,
  GPIO_NPS_D4,  GPIO_NPS_D5,  GPIO_NPS_D6,  GPIO_NPS_D7,
  GPIO_NPS_D8,  GPIO_NPS_D9,  GPIO_NPS_D10, GPIO_NPS_D11,
  GPIO_NPS_D12, GPIO_NPS_D13, GPIO_NPS_D14, GPIO_NPS_D15,

  /* NOE, NWE  */

  GPIO_NPS_NOE, GPIO_NPS_NWE
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
 *   Initialize GPIOs for NOR or SRAM
 *
 ****************************************************************************/

void stm32_extmemgpios(const uint16_t *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < ngpios; i++)
    {
      stm32_configgpio(gpios[i]);
    }
}
