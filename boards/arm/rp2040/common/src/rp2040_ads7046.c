/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_ads7046.c
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

#include <nuttx/arch.h>
#include <nuttx/analog/ads7046.h>

#include "rp2040_spi.h"
#include "rp2040_ads7046.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ads7046_initialize
 *
 * Description:
 *   Initialize and register the ADS7046 ADC driver.
 *
 * Input Parameters:
 *   spi   - An instance of the SPI interface to use.
 *   devno - The device number, used to build the device path as /dev/adcN.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ads7046_initialize(FAR struct spi_dev_s *spi, int devno)
{
  char devpath[10];
  int ret;

  ainfo("Initializing ADS7046 #%d\n", devno);

  if (spi)
    {
      snprintf(devpath, sizeof(devpath), "/dev/adc%d", devno);
      ret = ads7046_register(devpath, spi, devno);
      if (ret < 0)
        {
          snerr("ERROR: Error registering ADS7046 at /dev/adc%d\n", devno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
