/****************************************************************************
 * boards/xtensa/esp32s2/common/src/esp32s2_board_spidev.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi_transfer.h>

#ifdef CONFIG_ESPRESSIF_SPI_PERIPH
#include "esp32s2_spi.h"
#endif

#ifdef CONFIG_ESPRESSIF_SPI_BITBANG
#include "espressif/esp_spi_bitbang.h"
#endif

#include "esp32s2_board_spidev.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_bitbang_driver_init
 *
 * Description:
 *   Initialize SPI bitbang driver and register the /dev/spi device.
 *
 * Input Parameters:
 *   port - The SPI bus number, used to build the device path as /dev/spiN
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI_BITBANG
static int spi_bitbang_driver_init(int port)
{
  int ret;
  struct spi_dev_s *spi;

  syslog(LOG_INFO, "Initializing /dev/spi%d...\n", port);

  /* Initialize SPI device */

  spi = esp_spi_bitbang_init();

  if (spi == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d.\n", port);
      return -ENODEV;
    }

  ret = spi_register(spi, port);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register /dev/spi%d: %d\n", port, ret);
      esp_spi_bitbang_uninitialize(spi);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_driver_init
 *
 * Description:
 *   Initialize SPI driver and register the /dev/spi device.
 *
 * Input Parameters:
 *   port - The SPI bus number, used to build the device path as /dev/spiN
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_SPI_PERIPH
static int spi_driver_init(int port)
{
  int ret = OK;
  struct spi_dev_s *spi;

  syslog(LOG_INFO, "Initializing /dev/spi%d...\n", port);

  /* Initialize SPI device */

  spi = esp32s2_spibus_initialize(port);

  if (spi == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d.\n", port);
      return -ENODEV;
    }

  ret = spi_register(spi, port);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register /dev/spi%d: %d\n", port, ret);
      esp32s2_spibus_uninitialize(spi);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spidev_initialize
 *
 * Description:
 *   Initialize SPI driver and register the /dev/spi device.
 *
 * Input Parameters:
 *   port - The SPI bus number, used to build the device path as /dev/spiN
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_spidev_initialize(int port)
{
  int ret = OK;

  switch (port)
    {
#ifdef CONFIG_ESP32S2_SPI2
      case ESP32S2_SPI2:
        {
          ret = spi_driver_init(ESP32S2_SPI2);
          if (ret != OK)
            {
              return ret;
            }
          break;
        }
#endif

#ifdef CONFIG_ESP32S2_SPI3
      case ESP32S2_SPI3:
        {
          ret = spi_driver_init(ESP32S2_SPI3);
          if (ret != OK)
            {
              return ret;
            }
          break;
        }
#endif

#ifdef CONFIG_ESPRESSIF_SPI_BITBANG
      case ESPRESSIF_SPI_BITBANG:
        {
          ret = spi_bitbang_driver_init(ESPRESSIF_SPI_BITBANG);
          if (ret != OK)
            {
              return ret;
            }
          break;
        }
#endif

    default:
      {
        wderr("ERROR: unsupported SPI %d\n", port);
        return ERROR;
      }
    }

  return ret;
}
