/****************************************************************************
 * boards/arm/nrf53/thingy53/src/nrf53_spi.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "nrf53_gpio.h"
#include "nrf53_spi.h"

#include "thingy53.h"
#include <arch/board/board.h>

#ifdef CONFIG_NRF53_SPI_MASTER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-144 board.
 *
 ****************************************************************************/

void nrf53_spidev_initialize(void)
{
#ifdef CONFIG_NRF53_SPI1_MASTER
  nrf53_gpio_config(GPIO_NRF21540_CSN);
  nrf53_gpio_write(GPIO_NRF21540_CSN, true);

  nrf53_gpio_config(GPIO_ADXL362_CS);
  nrf53_gpio_write(GPIO_ADXL362_CS, true);

  nrf53_gpio_config(GPIO_BMI270_CS);
  nrf53_gpio_write(GPIO_BMI270_CS, true);
#endif
}

/****************************************************************************
 * Name: nrf53_spi0/1/2/3/select and nrf53_spi0/1/2/3/status
 *
 * Description:
 *   The external functions, nrf53_spi0/1/2/3select and
 *   nrf53_spi0/1/2/3status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including nrf53_spibus_initialize()) are provided
 *   by common NRF53 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in nrf53_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide nrf53_spi0/1/2/3select() and nrf53_spi0/1/2/3status()
 *      functions in your board-specific logic. These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to nrf53_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by nrf53_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_NRF53_SPI0_MASTER
void nrf53_spi0select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf53_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_NRF53_SPI1_MASTER
void nrf53_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");

  switch (devid)
    {
#ifdef CONFIG_SENSORS_BMI270
      case SPIDEV_IMU(0):
        {
          spiinfo("BMI270 device %s\n",
                  selected ? "asserted" : "de-asserted");

          /* Set the GPIO low to select and high to de-select */

          nrf53_gpio_write(GPIO_BMI270_CS, !selected);
          break;
        }
#endif

#ifdef CONFIG_SENSORS_ADXL362
      case SPIDEV_ACCELEROMETER(1):
      {
        spiinfo("ADXL362 device %s\n",
                selected ? "asserted" : "de-asserted");

        /* Set the GPIO low to select and high to de-select */

        nrf53_gpio_write(GPIO_ADXL362_CS, !selected);
        break;
      }
#endif

      default:
        {
          break;
        }
    }
}

uint8_t nrf53_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
#ifdef CONFIG_SENSORS_BMI270
      case SPIDEV_IMU(0):
        {
          status |= SPI_STATUS_PRESENT;
          break;
        }
#endif

#ifdef CONFIG_SENSORS_ADXL362
      case SPIDEV_ACCELEROMETER(0):
        {
          status |= SPI_STATUS_PRESENT;
          break;
        }
#endif

      default:
        {
          break;
        }
    }

  return status;
}
#endif

#ifdef CONFIG_nrf53_SPI2_MASTER
void nrf53_spi2select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf53_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_NRF53_SPI3_MASTER
void nrf53_spi3select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf53_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_NRF53_SPI4_MASTER
void nrf53_spi4select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf53_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#endif /* CONFIG_NRF53_SPI_MASTER */
