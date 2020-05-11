/****************************************************************************
 * boards/arm/stm32/common/src/stm32_nrf24l01.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <laurent@latil.nom.fr>
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
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wireless/nrf24l01.h>
#include <arch/board/board.h>

#include "arm_arch.h"
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

static int nrf24l01_irq_attach(xcpt_t isr, FAR void *arg);
static void nrf24l01_chip_enable(bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct nrf24l01_config_s nrf_cfg =
{
  .irqattach  = nrf24l01_irq_attach,
  .chipenable = nrf24l01_chip_enable,
};

static xcpt_t g_isr;
static FAR void *g_arg;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nrf24l01_irq_attach(xcpt_t isr, FAR void *arg)
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
  FAR struct spi_dev_s *spidev;
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
