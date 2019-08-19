/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_mpu6000.c
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Bill Gatliff <bgat@billgatliff.com>
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
