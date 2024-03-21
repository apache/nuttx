/****************************************************************************
 * boards/risc-v/esp32c3-legacy/common/src/esp32c3_board_ice40.c
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

#include <assert.h>
#include <debug.h>
#include <stdint.h>
#include <sys/types.h>

#include <arch/board/board.h>
#include <arch/esp32c3-legacy/chip.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "nuttx/spi/ice40.h"

#include "esp32c3_ice40.h"
#include "esp32c3_board_ice40.h"

#include "esp32c3_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_ice40_setup
 *
 * Description:
 *  Initialize ICE40 FPGA GPIOs, SPI and register the ICE40 driver.
 *
 ****************************************************************************/

int esp32c3_ice40_setup(void)
{
  struct ice40_dev_s *ice40;

  /* Initialize ICE40 FPGA GPIOs and SPI interface */

  ice40 = esp32c3_ice40_initialize(CONFIG_ESP32C3_ICE40_CDONEPIN,
                                   CONFIG_ESP32C3_ICE40_CRSTPIN,
                                   CONFIG_ESP32C3_ICE40_CSPIN,
                                   CONFIG_ESP32C3_ICE40_SPI_PORT);
  if (ice40 <= 0)
    {
      _err("ERROR: Failed to initialize ICE40 driver\n");
      return -ENODEV;
    }

  /* Register the ICE40 FPGA device at "/dev/ice40-0" */

  int ret = ice40_register("/dev/ice40-0", ice40);
  if (ret < 0)
    {
      _err("ERROR: Failed to register ICE40 driver: %d\n", ret);
      return ret;
    }

  return OK;
}
