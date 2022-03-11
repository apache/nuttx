/****************************************************************************
 * boards/arm/stm32/stm3240g-eval/src/stm32_extmem.c
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
#include "stm3240g-eval.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_FSMC
#  warning "FSMC is not enabled"
#endif

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

#define STM32_FSMC_NADDRCONFIGS 26
#define STM32_FSMC_NDATACONFIGS 16

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* GPIO configurations common to most external memories */

static const uint32_t g_addressconfig[STM32_FSMC_NADDRCONFIGS] =
{
  GPIO_FSMC_A0,  GPIO_FSMC_A1,  GPIO_FSMC_A2,
  GPIO_FSMC_A3,  GPIO_FSMC_A4,  GPIO_FSMC_A5,
  GPIO_FSMC_A6,  GPIO_FSMC_A7,  GPIO_FSMC_A8,
  GPIO_FSMC_A9,  GPIO_FSMC_A10, GPIO_FSMC_A11,
  GPIO_FSMC_A12, GPIO_FSMC_A13, GPIO_FSMC_A14,
  GPIO_FSMC_A15, GPIO_FSMC_A16, GPIO_FSMC_A17,
  GPIO_FSMC_A18, GPIO_FSMC_A19, GPIO_FSMC_A20,
  GPIO_FSMC_A21, GPIO_FSMC_A22, GPIO_FSMC_A23,
  GPIO_FSMC_A24, GPIO_FSMC_A25
};

static const uint32_t g_dataconfig[STM32_FSMC_NDATACONFIGS] =
{
  GPIO_FSMC_D0,  GPIO_FSMC_D1,  GPIO_FSMC_D2,
  GPIO_FSMC_D3,  GPIO_FSMC_D4,  GPIO_FSMC_D5,
  GPIO_FSMC_D6,  GPIO_FSMC_D7,  GPIO_FSMC_D8,
  GPIO_FSMC_D9,  GPIO_FSMC_D10, GPIO_FSMC_D11,
  GPIO_FSMC_D12, GPIO_FSMC_D13, GPIO_FSMC_D14,
  GPIO_FSMC_D15
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

void stm32_extmemgpios(const uint32_t *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < ngpios; i++)
    {
      stm32_configgpio(gpios[i]);
    }
}

/****************************************************************************
 * Name: stm32_extmemaddr
 *
 * Description:
 *   Initialize address line GPIOs for external memory access
 *
 ****************************************************************************/

void stm32_extmemaddr(int naddrs)
{
  stm32_extmemgpios(g_addressconfig, naddrs);
}

/****************************************************************************
 * Name: stm32_extmemdata
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 *
 ****************************************************************************/

void stm32_extmemdata(int ndata)
{
  stm32_extmemgpios(g_dataconfig, ndata);
}
