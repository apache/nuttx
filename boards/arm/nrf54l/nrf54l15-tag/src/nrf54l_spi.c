/****************************************************************************
 * boards/arm/nrf54l/nrf54l15-tag/src/nrf54l_spi.c
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
#include <nuttx/debug.h>
#include <nuttx/spi/spi.h>

#include "nrf54l_gpio.h"
#include "nrf54l_spi.h"
#include "nrf54l15-tag.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_spidev_initialize
 *
 * Description:
 *   Configure the BMI270 chip select.
 *
 ****************************************************************************/

void nrf54l_spidev_initialize(void)
{
  nrf54l_gpio_config(GPIO_BMI270_CS);
}

/****************************************************************************
 * Name: nrf54l_spi2select
 ****************************************************************************/

void nrf54l_spi2select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");

  switch (devid)
    {
      case SPIDEV_IMU(0):
        nrf54l_gpio_write(GPIO_BMI270_CS, !selected);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: nrf54l_spi2status
 ****************************************************************************/

uint8_t nrf54l_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return devid == SPIDEV_IMU(0) ? SPI_STATUS_PRESENT : 0;
}
