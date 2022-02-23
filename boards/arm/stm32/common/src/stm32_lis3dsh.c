/****************************************************************************
 * boards/arm/stm32/common/src/stm32_lis3dsh.c
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
#include <nuttx/sensors/lis3dsh.h>
#include <arch/board/board.h>

#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: attach_disc_lis3dsh
 *
 * Description:
 *   Attach the lis3dsh interrupt handler to PE0/EXT0 on the STM32F4 as wired
 *   on STM32F4Discovery
 *
 * Input Parameters:
 *   config - The lis3dsh instance configuration data containing
 *            the IRQ number, device ID and interrupt handler
 *   interrupt_handler - The interrupt handler to attach
 *   arg -
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int attach_disc_lis3dsh(FAR struct lis3dsh_config_s *config,
                        xcpt_t interrupt_handler)
{
  return stm32_gpiosetevent(BOARD_LIS3DSH_GPIO_EXT0, true, false, false,
                            interrupt_handler, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lis3dsh_initialize
 *
 * Description:
 *   Initialize and register the LIS3DSH 3-axis accelerometer.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/accN
 *   busno - The SPI bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lis3dsh_initialize(int devno, int busno)
{
  static struct lis3dsh_config_s acc0_config;
  char devpath[12];
  struct spi_dev_s *spi;
  int ret;

  sninfo("Initializing LIS3DSH\n");

  acc0_config.irq = 22;
  acc0_config.spi_devid = 0;
  acc0_config.attach = &attach_disc_lis3dsh;

  spi = stm32_spibus_initialize(1);
  if (!spi)
    {
      spiinfo("Failed to initialize SPI port\n");
      ret = -ENODEV;
    }
  else
    {
      snprintf(devpath, 12, "/dev/acc%d", devno);
      ret = lis3dsh_register(devpath, spi, &acc0_config);
    }

  return ret;
}
