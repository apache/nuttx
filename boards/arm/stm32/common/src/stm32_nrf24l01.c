/****************************************************************************
 * boards/arm/stm32/common/src/stm32_nrf24l01.c
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
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wireless/nrf24l01.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_nrf24l01.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf24l01_irq_attach(xcpt_t isr, void *arg);
static void nrf24l01_chip_enable(bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nrf24l01_config_s nrf_cfg =
{
  .irqattach  = nrf24l01_irq_attach,
  .chipenable = nrf24l01_chip_enable,
};

static xcpt_t g_isr;
static void *g_arg;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nrf24l01_irq_attach(xcpt_t isr, void *arg)
{
  wlinfo("Attach IRQ\n");
  g_isr = isr;
  g_arg = arg;
  stm32_gpiosetevent(BOARD_NRF24L01_GPIO_IRQ, false, true, false,
                     g_isr, g_arg);
  return OK;
}

static void nrf24l01_chip_enable(bool enable)
{
  wlinfo("CE:%d\n", enable);
  stm32_gpiowrite(BOARD_NRF24L01_GPIO_CE, enable);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_nrf24l01_initialize
 *
 * Description:
 *   Initialize the NRF24L01 wireless module
 *
 * Input Parameters:
 *   busno - The SPI bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_nrf24l01_initialize(int busno)
{
  struct spi_dev_s *spidev;
  int result;

  /* Setup CE & IRQ line IOs */

  stm32_configgpio(BOARD_NRF24L01_GPIO_CE);
  stm32_configgpio(BOARD_NRF24L01_GPIO_IRQ);

  /* Init SPI bus */

  spidev = stm32_spibus_initialize(busno);
  if (!spidev)
    {
      wlerr("ERROR: Failed to initialize SPI bus\n");
      return -ENODEV;
    }

  result = nrf24l01_register(spidev, &nrf_cfg);
  if (result != OK)
    {
      wlerr("ERROR: Failed to register initialize SPI bus\n");
      return -ENODEV;
    }

  return OK;
}
