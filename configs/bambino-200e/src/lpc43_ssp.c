/************************************************************************************
 * configs/bambino-200e/src/lpc43_ssp.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <errno.h>
#include <debug.h>
#include <string.h>

#include "lpc43_spi.h"
#include "lpc43_ssp.h"

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "bambino-200e.h"

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/random.h>

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: lpc43_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Bambino-200e board.
 *
 ****************************************************************************/

void weak_function lpc43_sspdev_initialize(void)
{
#ifdef CONFIG_SENSORS_MAX31855
  lpc43_pin_config(PINCONFIG_MAX31855_CS1);
  int ret = lpc43_gpio_config(GPIO_MAX31855_CS1);
  if (ret < 0)
    {
      snerr("ERROR: Error configuring chip select GPIO pin\n");
    }

  lpc43_pin_config(PINCONFIG_MAX31855_CS2);
  ret = lpc43_gpio_config(GPIO_MAX31855_CS2);
  if (ret < 0)
    {
      snerr("ERROR: Error configuring chip select GPIO pin\n")
    }
#endif
}

/************************************************************************************
 * Name:  lpc43_ssp0select
 *
 * Description:
 *   Perform chip selection using GPIO pins, controlling data flow in SSP0 channel.
 *
 * Input parameters:
 *   devpath  - The full path to the driver.  E.g., "/dev/temp0"
 *   devid    - Minor device number.  E.g., 0, 1, 2, etc.
 *   selected - Logical state of the pin
 *
 ************************************************************************************/

void lpc43_ssp0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_SENSORS_MAX31855)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      lpc43_gpio_write(GPIO_MAX31855_CS1, !selected);
    }

  if (devid == SPIDEV_TEMPERATURE(1))
    {
      lpc43_gpio_write(GPIO_MAX31855_CS2, !selected);
    }
#endif
}

/************************************************************************************
 * Name:  lpc43_ssp0status
 *
 * Description:
 *   Perform status operations in SSP0 channel, using GPIO pins.
 *
 * Input parameters:
 *   devpath  - The full path to the driver.  E.g., "/dev/temp0"
 *   devid    - Minor device number.  E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero
 ************************************************************************************/

uint8_t lpc43_ssp0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/************************************************************************************
 * Name:  lpc43_ssp1select
 *
 * Description:
 *   Perform chip selection using GPIO pins, controlling data flow in SSP1 channel.
 *
 * Input parameters:
 *   devpath  - The full path to the driver.  E.g., "/dev/temp0"
 *   devid    - Minor device number.  E.g., 0, 1, 2, etc.
 *   selected - Logical state of the pin
 *
 ************************************************************************************/

void lpc43_ssp1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_SENSORS_MAX31855)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      lpc43_gpio_write(GPIO_MAX31855_CS1, !selected);
    }

  if (devid == SPIDEV_TEMPERATURE(1))
    {
      lpc43_gpio_write(GPIO_MAX31855_CS2, !selected);
    }
#endif
}

/************************************************************************************
 * Name:  lpc43_ssp1status
 *
 * Description:
 *   Perform status operations in SSP1 channel, using GPIO pins.
 *
 * Input parameters:
 *   devpath  - The full path to the driver.  E.g., "/dev/temp0"
 *   devid    - Minor device number.  E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero
 ************************************************************************************/

uint8_t lpc43_ssp1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
