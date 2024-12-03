/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/mx8mp_spidev.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi_transfer.h>

#include "arm_internal.h"
#include "chip.h"
#include "mx8mp_ecspi.h"
#include "verdin-mx8mp.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spidev_initialize
 *
 * Description:
 *   Initialize and register spi driver for the specified spi port
 *
 ****************************************************************************/

int board_spidev_initialize(int port)
{
  int ret;
  struct spi_dev_s *spi;

  spiinfo("Initializing /dev/spi%d..\n", port);

  /* Initialize spi device */

  spi = mx8mp_spibus_initialize(port);
  if (!spi)
    {
      i2cerr("ERROR: Failed to initialize spi%d.\n", port);
      return -ENODEV;
    }

  ret = spi_register(spi, port);
  if (ret < 0)
    {
      i2cerr("ERROR: Failed to register spi%d: %d\n", port, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_spidev_initialize
 *
 * Description:
 *   Called to configure all spi drivers
 *
 ****************************************************************************/

int mx8mp_spidev_initialize(void)
{
  int ret = 0;

#ifdef CONFIG_MX8MP_SPI1
  mx8mp_iomuxc_config(IOMUX_SPI1_MISO);
  mx8mp_iomuxc_config(IOMUX_SPI1_MOSI);
  mx8mp_iomuxc_config(IOMUX_SPI1_CLK);
  mx8mp_iomuxc_config(IOMUX_SPI1_CS);

  ret = board_spidev_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C1.\n");
      return ret;
    }
#endif

#ifdef CONFIG_MX8MP_SPI2
  mx8mp_iomuxc_config(IOMUXC_SPI2_MISO);
  mx8mp_iomuxc_config(IOMUXC_SPI2_MOSI);
  mx8mp_iomuxc_config(IOMUXC_SPI2_CLK);
  mx8mp_iomuxc_config(IOMUXC_SPI2_CS);
  mx8mp_gpio_config(GPIO_SPI2_CS);

  ret = board_spidev_initialize(2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C2.\n");
      return ret;
    }
#endif

#ifdef CONFIG_MX8MP_SPI3
  mx8mp_iomuxc_config(IOMUXC_SPI3_MISO);
  mx8mp_iomuxc_config(IOMUXC_SPI3_MOSI);
  mx8mp_iomuxc_config(IOMUXC_SPI3_CLK);
  mx8mp_iomuxc_config(IOMUXC_SPI3_CS);
  mx8mp_gpio_config(GPIO_SPI3_CS);

  ret = board_spidev_initialize(3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C3.\n");
      return ret;
    }
#endif

  return ret;
}
