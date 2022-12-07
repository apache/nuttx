/****************************************************************************
 * boards/risc-v/esp32c3/common/src/esp32c3_board_spislavedev.c
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

#include <nuttx/spi/slave.h>

#include "esp32c3_spi.h"

#include "esp32c3_board_spislavedev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spislavedev_initialize
 *
 * Description:
 *   Initialize SPI Slave driver and register the /dev/spislv device.
 *
 * Input Parameters:
 *   bus - The SPI bus number, used to build the device path as /dev/spislvN
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_spislavedev_initialize(int bus)
{
  int ret;

  struct spi_slave_ctrlr_s *ctrlr;

  spiinfo("Initializing /dev/spislv%d...\n", bus);

  /* Initialize SPI Slave controller device */

  ctrlr = esp32c3_spislave_ctrlr_initialize(bus);
  if (ctrlr == NULL)
    {
      spierr("Failed to initialize SPI%d as slave.\n", bus);
      return -ENODEV;
    }

  ret = spi_slave_register(ctrlr, bus);
  if (ret < 0)
    {
      spierr("Failed to register /dev/spislv%d: %d\n", bus, ret);

      esp32c3_spislave_ctrlr_uninitialize(ctrlr);
    }

  return ret;
}
