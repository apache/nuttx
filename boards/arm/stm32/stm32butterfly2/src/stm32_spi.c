/*****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_spi.c
 *
 *   Copyright (C) 2016 Michał Łyszczek. All rights reserved.
 *   Author: Michał Łyszczek <michal.lyszczek@gmail.com>
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
 ****************************************************************************/

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include <nuttx/spi/spi.h>

#include "stm32_butterfly2.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 * Note:
 *   Here only CS pins are configured as SPI pins are configured by driver
 *   itself.
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
  spiinfo("INFO: Initializing spi gpio pins\n");

  stm32_configgpio(GPIO_SD_CS);
  stm32_configgpio(GPIO_SD_CD);
}

/*****************************************************************************
 * Name: stm32_spi1select
 *
 * Description:
 *   Function asserts given devid based on select
 ****************************************************************************/

void stm32_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool select)
{
  spiinfo("INFO: Selecting spi dev: %d, state: %d\n", devid, select);

  if (devid == SPIDEV_MMCSD(0))
    {
      stm32_gpiowrite(GPIO_SD_CS, !select);
    }
}

/*****************************************************************************
 * Name: stm32_spi1status
 *
 * Description:
 *   Return status of devid
 ****************************************************************************/

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("INFO: Requesting info from spi dev: %d\n", devid);

  if (devid == SPIDEV_MMCSD(0))
    {
      if (stm32_gpioread(GPIO_SD_CD) == 0)
        {
          return SPI_STATUS_PRESENT;
        }
    }

  return 0;
}
