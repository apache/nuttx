/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_max7456.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <nuttx/video/max7456.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "omnibusf4.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_max7456_initialize
 *
 * Description:
 *
 *   Initialize and register Omnibus F4's MAX7456 OSD chip. The wiring
 *   isn't configurable, but we use macros anyway because some of the
 *   values are referred to in more than one place. And also, because
 *   that's generally what NuttX does.
 *
 *   In particular, the slave-select pin is defined by us, but
 *   controlled elsewhere as part of the SPI machinery. This is an odd
 *   thing in our case because nothing else is using the SPI port, but
 *   that's not the general presentation so I'm staying consistent
 *   with the pattern.
 *
 ****************************************************************************/

int stm32_max7456_initialize(void)
{
  int port = SPIPORT_MAX7456;
  int minor = SPIMINOR_MAX7456;
  int cs = GPIO_CS_MAX7456;

  stm32_configgpio(GPIO_CS_MAX7456);

  struct mx7_config_s config =
    {
     .spi_devid = minor,
    };

  UNUSED(cs);

  /* Get the spi bus instance. */

  struct spi_dev_s *spi = stm32_spibus_initialize(port);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  config.spi = spi;

  /* Register the chip with the device driver. */

  return max7456_register(DEVNODE_MAX7456, &config);
}
