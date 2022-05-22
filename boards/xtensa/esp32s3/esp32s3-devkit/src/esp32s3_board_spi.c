/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_board_spi.c
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

#include <nuttx/spi/spi.h>

#include "esp32s3_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_spi2_status
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2

uint8_t esp32s3_spi2_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  return status;
}

#endif

/****************************************************************************
 * Name: esp32s3_spi2_cmddata
 ****************************************************************************/

#if defined(CONFIG_ESP32S3_SPI2) && defined(CONFIG_SPI_CMDDATA)

int esp32s3_spi2_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      esp32s3_gpiowrite(CONFIG_ESP32S3_SPI2_MISOPIN, !cmd);

      return OK;
    }

  spiinfo("devid: %" PRIu32 " CMD: %s\n", devid, cmd ? "command" :
          "data");

  return -ENODEV;
}

#endif

/****************************************************************************
 * Name: esp32s3_spi3_status
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI3

uint8_t esp32s3_spi3_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  return status;
}

#endif

/****************************************************************************
 * Name: esp32s3_spi3_cmddata
 ****************************************************************************/

#if defined(CONFIG_ESP32S3_SPI3) && defined(CONFIG_SPI_CMDDATA)

int esp32s3_spi3_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      esp32s3_gpiowrite(CONFIG_ESP32S3_SPI3_MISOPIN, !cmd);

      return OK;
    }

  spiinfo("devid: %" PRIu32 " CMD: %s\n", devid, cmd ? "command" :
          "data");

  return -ENODEV;
}

#endif
