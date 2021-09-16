/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_mpu6000.c
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

#include <nuttx/sensors/mpu60x0.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "omnibusf4.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mpu6000_initialize
 *
 * Description:
 *
 *   Initialize and register's Omnibus F4's MPU6000 IMU. The wiring
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

int stm32_mpu6000_initialize(void)
{
  int port = SPIPORT_MPU6000;
  int minor = SPIMINOR_MPU6000;
  int exti = GPIO_EXTI_MPU6000;
  int cs = GPIO_CS_MPU6000;

  stm32_configgpio(GPIO_CS_MPU6000);
  stm32_configgpio(GPIO_EXTI_MPU6000);

  /* Note: the "minor" concept doesn't really apply since we're
   * uniquely-identified by a CS pin, we're the only device on the SPI
   * bus, and because users will refer to us through our device-node
   * pathname; I'm leaving this here for now anyway, in case we decide
   * sometime soon to do things differently.
   *
   * Likewise, we'll probably add things like EXTI, etc. to
   * mpu_config_s as the driver learns to support them.
   */

  struct mpu_config_s config =
  {
    .spi_devid = minor,
  };

  UNUSED(cs);
  UNUSED(exti);

  /* Get the spi bus instance. */

  struct spi_dev_s *spi = stm32_spibus_initialize(port);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  config.spi = spi;

  /* TODO: configure EXTI pin */

  /* Register the chip with the device driver. */

  int ret = mpu60x0_register(DEVNODE_MPU6000, &config);
  return ret;
}
