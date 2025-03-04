/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_max6675.c
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
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/max6675.h>

#include "rp2040_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_max6675_initialize
 *
 * Description:
 *   Initialize the MAX6675 sensor.
 * Input:
 *   devno - Device number to register, i.e. 0 for /dev/temp0
 *   busno - The SPI controller port used. i.e. 0 for SPI0
 *
 ****************************************************************************/

int board_max6675_initialize(int devno, int busno)
{
  struct spi_dev_s *spi;
  char devpath[12];
  int ret;

  spi = rp2040_spibus_initialize(busno);
  if (spi == NULL)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", busno);
      return -ENODEV;
    }

  /* Then register the temperature sensor */

  snprintf(devpath, sizeof(devpath), "/dev/temp%d", devno);
  ret = max6675_register(devpath, spi);
  if (ret < 0)
    {
      snerr("ERROR: Error registering MAX6675\n");
    }

  return OK;
}

