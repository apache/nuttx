/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_max7219_leds.c
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

#include <nuttx/spi/spi.h>
#include <nuttx/leds/max7219.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "stm32f4discovery.h"

#if defined(CONFIG_SPI) && defined(CONFIG_STM32_SPI1) && \
    defined(CONFIG_LEDS_MAX7219)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX7219_SPI_PORTNO 1   /* On SPI1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_max7219init
 *
 * Description:
 *   Initialize the max7219 to control 7-segment numeric display
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/numdisp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_max7219init(const char *devpath)
{
  struct spi_dev_s *spi;
  int ret;

  spi = stm32_spibus_initialize(MAX7219_SPI_PORTNO);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  /* Register the MAX7219 Driver at the specified location. */

  ret = max7219_leds_register(devpath, spi);
  if (ret < 0)
    {
      lederr("ERROR: max7219_leds_register(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_LEDS_MAX7219 */
