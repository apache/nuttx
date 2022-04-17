/****************************************************************************
 * boards/arm/stm32/common/src/stm32_l3gd20.c
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
#include <nuttx/sensors/l3gd20.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int l3gd20_attach(struct l3gd20_config_s * cfg, xcpt_t irq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one L3GD20 device on board */

static struct l3gd20_config_s g_l3gd20_config =
{
  .attach = l3gd20_attach,
  .irq = BOARD_L3GD20_IRQ,
  .spi_devid = SPIDEV_ACCELEROMETER(0)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: l3gd20_attach()
 *
 * Description: Attach the l3gd20 interrupt handler to the GPIO interrupt
 *
 ****************************************************************************/

static int l3gd20_attach(struct l3gd20_config_s *cfg, xcpt_t irq)
{
  return stm32_gpiosetevent(BOARD_L3GD20_GPIO_DREADY, true, false,
                            true, irq, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_l3gd20_initialize()
 *
 * Description:
 *   Initialize and register the L3GD20 3 axis gyroscope sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as
 *           /dev/sensor/gyro_uncalN
 *   busno - The SPI bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_l3gd20_initialize(int devno, int busno)
{
  int ret = 0;
  struct spi_dev_s *spi;

  /* Configure DREADY IRQ input */

  stm32_configgpio(BOARD_L3GD20_GPIO_DREADY);

  /* Initialize SPI */

  spi = stm32_spibus_initialize(busno);

  if (!spi)
    {
      ret = -ENODEV;
      goto errout;
    }

  /* Then register the gyro */

  ret = l3gd20_register(devno, spi, &g_l3gd20_config);
  if (ret != OK)
    {
      goto errout;
    }

errout:
  return ret;
}
