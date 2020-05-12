/****************************************************************************
 * boards/arm/stm32/common/src/stm32_lis3dsh.c
 *
 *   Copyright (C) 2017 Florian Olbrich. All rights reserved.
 *   Author: Florian Olbrich <flox@posteo.de>
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
