/****************************************************************************
 * boards/risc-v/esp32c3/common/src/esp32c3_board_spidev.c
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

#include "esp32c3_spi.h"

#include "esp32c3_board_spidev.h"

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
 *   bus - The SPI bus number, used to build the device path as /dev/spiN
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_spidev_initialize(int port)
{
  int ret;
  struct spi_dev_s *spi;

  syslog(LOG_INFO, "Initializing /dev/spi%d...\n", port);

  /* Initialize SPI device */

  spi = esp32c3_spibus_initialize(port);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d.\n", port);
      return -ENODEV;
    }

  ret = spi_register(spi, port);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register /dev/spi%d: %d\n", port, ret);

      esp32c3_spibus_uninitialize(spi);
    }

  return ret;
}
