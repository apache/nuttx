/****************************************************************************
 * boards/arm/stm32/common/src/stm32_ws2812.c
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

#include <nuttx/spi/spi.h>
#include <nuttx/leds/ws2812.h>

#include "stm32.h"
#include "stm32_spi.h"

#ifdef CONFIG_WS2812

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

  spi = stm32_spibus_initialize(spino);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  /* Register the WS2812 driver at the specified location. */

  snprintf(devpath, 13, "/dev/leddrv%d", devno);
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
