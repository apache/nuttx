/****************************************************************************
 * boards/arm/stm32/common/src/stm32_mt6816.c
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
#include <stdio.h>

#include <nuttx/sensors/qencoder.h>
#include <nuttx/sensors/mt6816.h>
#include <arch/board/board.h>
#include <nuttx/spi/spi.h>

#include "chip.h"
#include "stm32.h"
#include "stm32_spi.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mt6816_initialize
 *
 * Description:
 *   Initialize the MT6816 encoder driver
 *
 ****************************************************************************/

int board_mt6816_initialize(int devno, int spi_busno)
{
  struct spi_dev_s *spi;
  struct qe_lowerhalf_s *dev;
  char qe_path[12];
  int ret = OK;

  spi = stm32_spibus_initialize(spi_busno);
  if (!spi)
    {
      return -ENODEV;
    }

  dev = mt6816_initialize(spi, (uint16_t) devno);
  if (dev == NULL)
    {
      snerr("ERROR: Failed to initialize MT6816 at SPI%d\n", spi_busno);
      ret = -ENODEV;
    }
  else
    {
      snprintf(qe_path, sizeof(qe_path), "/dev/qe%d", devno);
      ret = qe_register(qe_path, dev);
      if (ret < 0)
        {
          snerr("ERROR: Failed to register MT6816 qe%d driver: %d\n",
                devno, ret);
          ret = -ENODEV;
        }
    }

  return ret;
}
