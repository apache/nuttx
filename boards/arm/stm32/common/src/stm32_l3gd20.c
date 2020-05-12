/****************************************************************************
 * boards/arm/stm32/common/src/stm32_l3gd20.c
 *
 *   Copyright (C) 2017 Gregory Nutt.  All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

static int l3gd20_attach(FAR struct l3gd20_config_s * cfg, xcpt_t irq);

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

static int l3gd20_attach(FAR struct l3gd20_config_s *cfg, xcpt_t irq)
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
 *   devno - The device number, used to build the device path as /dev/gyroN
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
  char devpath[12];

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

  snprintf(devpath, 12, "/dev/gyro%d", devno);
  ret = l3gd20_register(devpath, spi, &g_l3gd20_config);
  if (ret != OK)
    {
      goto errout;
    }

errout:
  return ret;
}
