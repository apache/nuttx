/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-l073rz/src/stm32_nrf24l01.c
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

#include <nuttx/spi/spi.h>
#include <nuttx/wireless/nrf24l01.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "stm32.h"
#include "nucleo-l073rz.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF24L01_SPI 1

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf24l01_irq_attach(xcpt_t isr, FAR void *arg);
static void nrf24l01_chip_enable(bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nrf24l01_config_s nrf_cfg =
{
  .irqattach = nrf24l01_irq_attach,
  .chipenable = nrf24l01_chip_enable,
};

static xcpt_t g_isr;
static FAR void *g_arg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nrf24l01_irq_attach(xcpt_t isr, FAR void *arg)
{
  wlinfo("Attach IRQ\n");
  g_isr = isr;
  g_arg = arg;
  stm32_gpiosetevent(GPIO_NRF24L01_IRQ, false, true, false, g_isr, g_arg);
  return OK;
}

static void nrf24l01_chip_enable(bool enable)
{
  wlinfo("CE:%d\n", enable);
  stm32_gpiowrite(GPIO_NRF24L01_CE, enable);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_wlinitialize(void)
{
  FAR struct spi_dev_s *spidev;
  int ret = OK;

  syslog(LOG_INFO, "Register the nRF24L01 module\n");

  /* Setup CE & IRQ line IOs */

  stm32_configgpio(GPIO_NRF24L01_CE);
  stm32_configgpio(GPIO_NRF24L01_IRQ);

  /* Init SPI bus */

  spidev = stm32_spibus_initialize(NRF24L01_SPI);
  if (!spidev)
    {
      wlerr("ERROR: Failed to initialize SPI %d bus\n", NRF24L01_SPI);
      ret = -ENODEV;
      goto errout;
    }

  ret = nrf24l01_register(spidev, &nrf_cfg);
  if (ret != OK)
    {
      wlerr("ERROR: Failed to register initialize SPI bus\n");
      goto errout;
    }

errout:
  return ret;
}
