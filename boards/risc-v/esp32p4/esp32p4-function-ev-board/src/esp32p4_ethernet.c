/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_ethernet.c
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

#include <debug.h>
#include <errno.h>

#include "espressif/esp_emac.h"
#include "espressif/esp_hr_timer.h"

#include "esp32p4-function-ev-board.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_emac_init
 *
 * Description:
 *   Bring up the ESP32-P4 Ethernet interface (internal EMAC + external
 *   PHY) and register it with the NuttX network stack.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_emac_init(void)
{
  int ret;

  /* esp_eth_driver_install() relies on esp_timer; make sure the timer
   * subsystem is initialised before creating the driver.
   */

  ret = esp_hr_timer_init();
  if (ret < 0)
    {
      nerr("ERROR: esp_hr_timer_init failed: %d\n", ret);
      return ret;
    }

  ret = esp_emac_init();
  if (ret < 0)
    {
      nerr("ERROR: esp_emac_init failed: %d\n", ret);
    }

  return ret;
}
