/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-lcd-ev/src/esp32s3_ws2812.c
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
#include "esp32s3_spi.h"

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/leds/ws2812.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APB_PERIOD (12.5)

#define T0H ((uint16_t)(350 / APB_PERIOD))   // ns
#define T0L ((uint16_t)(900 / APB_PERIOD))   // ns
#define T1H ((uint16_t)(900 / APB_PERIOD))   // ns
#define T1L ((uint16_t)(350 / APB_PERIOD))   // ns
#define RES ((uint16_t)(60000 / APB_PERIOD)) // ns

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

#ifndef CONFIG_WS2812_NON_SPI_DRIVER
/****************************************************************************
 * Name: board_ws2812_initialize
 *
 * Description:
 *   Initialize and register the WS2812 LED driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/leddrvN
 *   spino - SPI port number
 *   nleds - number of LEDs
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ws2812_initialize(int devno, int spino, uint16_t nleds)
{
  struct spi_dev_s *spi;
  char devpath[13];
  int ret;

  spi = esp32s3_spibus_initialize(spino);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  /* Register the WS2812 driver at the specified location. */

  snprintf(devpath, sizeof(devpath), "/dev/leds%d", devno);
  ret = ws2812_leds_register(devpath, spi, nleds);
  if (ret < 0)
    {
      lederr("ERROR: ws2812_leds_register(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}
#endif
