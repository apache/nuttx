/****************************************************************************
 * boards/risc-v/mpfs/common/src/mpfs_board_spi.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "mpfs_spi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spi_initialize
 *
 * Description:
 *   Initialize and register SPI driver for the defined SPI ports.
 *
 ****************************************************************************/

int mpfs_board_spi_init(void)
{
  int ret = OK;
#if defined(CONFIG_MPFS_SPI0) || defined(CONFIG_MPFS_SPI1)
  struct spi_dev_s *spi;
#ifdef CONFIG_SPI_DRIVER
  int port = 0;
#endif /* CONFIG_SPI_DRIVER */
#endif

  /* Initialize SPI device */

#ifdef CONFIG_MPFS_SPI0
  spi = mpfs_spibus_initialize(0);
  if (spi == NULL)
    {
      spierr("Failed to initialize SPI0\n");
      return -ENODEV;
    }

#ifdef CONFIG_SPI_DRIVER
  ret = spi_register(spi, port++);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register /dev/spi0: %d\n", ret);

      mpfs_spibus_uninitialize(spi);
    }
#endif /* CONFIG_SPI_DRIVER */
#endif /* CONFIG_MPFS_SPI0 */

#ifdef CONFIG_MPFS_SPI1
  spi = mpfs_spibus_initialize(1);
  if (spi == NULL)
    {
      spierr("Failed to initialize SPI1\n");
      return -ENODEV;
    }

#ifdef CONFIG_SPI_DRIVER
  ret = spi_register(spi, port);
  if (ret < 0)
    {
      spierr("Failed to register /dev/spi%d: %d\n", port, ret);

      mpfs_spibus_uninitialize(spi);
    }

#endif /* CONFIG_SPI_DRIVER */
#endif /* CONFIG_MPFS_SPI1 */
  return ret;
}
