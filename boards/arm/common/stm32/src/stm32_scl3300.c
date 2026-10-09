/****************************************************************************
 * boards/arm/common/stm32/src/stm32_scl3300.c
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

#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/sensors/scl3300.h>
#include <nuttx/spi/spi.h>

#include "stm32_spi.h"
#include "stm32_scl3300.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_scl3300_initialize
 *
 * Description:
 *   Initialize and register the Murata SCL3300 inclinometer with the
 *   default configuration (Mode 1, always on).
 *
 * Input Parameters:
 *   devno     - The uORB topic instance and SPIDEV_ACCELEROMETER() index.
 *   spi_busno - The SPI bus number.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_scl3300_initialize(int devno, int spi_busno)
{
  struct spi_dev_s *spi;
  int ret;

  spi = stm32_spibus_initialize(spi_busno);
  if (spi == NULL)
    {
      snerr("ERROR: Failed to initialize SPI%d\n", spi_busno);
      return -ENODEV;
    }

  ret = scl3300_register(devno, spi, NULL);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register SCL3300 %d on SPI%d: %d\n",
            devno, spi_busno, ret);
    }

  return ret;
}
