/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_board_rmt.c
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
#include <debug.h>
#include <stdio.h>

#include "xtensa.h"

#include <nuttx/kmalloc.h>
#include <nuttx/rmt/rmtchar.h>
#ifdef CONFIG_WS2812_NON_SPI_DRIVER
#include <nuttx/leds/ws2812.h>

#include "espressif/esp_ws2812.h"
#endif

#include "espressif/esp_rmt.h"

#ifdef CONFIG_ESP_RMT

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
 * Name: board_rmt_rxinitialize
 *
 * Description:
 *   Initialize the RMT peripheral and register an RX device.
 *
 * Input Parameters:
 *   ch  - The RMT's channel that will be used
 *   pin - The pin used for the RX channel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_rmt_rxinitialize(int ch, int pin)
{
  int ret;

  struct rmt_dev_s *rmt = esp_rmt_rx_init(ch, pin);

  ret = rmtchar_register(rmt);
  if (ret < 0)
    {
      rmterr("ERROR: rmtchar_register failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: board_rmt_txinitialize
 *
 * Description:
 *   Initialize the RMT peripheral and register an TX device.
 *
 * Input Parameters:
 *   ch  - The RMT's channel that will be used
 *   pin - The pin used for the TX channel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_rmt_txinitialize(int ch, int pin)
{
  int ret;
  struct rmt_dev_s *rmt;
#ifdef CONFIG_WS2812_NON_SPI_DRIVER
  struct ws2812_dev_s *led;
#endif

  rmt = esp_rmt_tx_init(ch, pin);
  if (rmt == NULL)
    {
      rmterr("ERROR: esp_rmt_tx_init failed\n");
      return -ENODEV;
    }

  ret = rmtchar_register(rmt);
  if (ret < 0)
    {
      rmterr("ERROR: rmtchar_register failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_WS2812_NON_SPI_DRIVER
  led = esp_ws2812_setup("/dev/leds0", rmt, CONFIG_WS2812_LED_COUNT, false);
  if (led == NULL)
    {
      rmterr("ERROR: esp_ws2812_setup failed\n");
      return -ENODEV;
    }
#endif

  return ret;
}
#endif
