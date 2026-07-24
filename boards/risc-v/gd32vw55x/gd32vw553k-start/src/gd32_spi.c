/****************************************************************************
 * boards/risc-v/gd32vw55x/gd32vw553k-start/src/gd32_spi.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "gd32vw55x_gpio.h"
#include "gd32vw553k-start.h"

#ifdef CONFIG_GD32VW55X_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spidev_initialize
 *
 * Description:
 *   Configure the SPI chip select GPIO(s).  The bus itself is brought up by
 *   gd32_spibus_initialize().
 *
 ****************************************************************************/

void gd32_spidev_initialize(void)
{
  /* The SD card chip select on SPI0, idle (de-asserted, high) */

  gd32_gpio_config(GPIO_SPI0_CSPIN);
  gd32_gpio_write(GPIO_SPI0_CSPIN, true);
}

/****************************************************************************
 * Name: gd32_spi0select and gd32_spi0status
 *
 * Description:
 *   The board-specific select and status methods of the SPI interface (see
 *   include/nuttx/spi/spi.h).  Chip select is active low.
 *
 ****************************************************************************/

void gd32_spi0select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %" PRIu32 " CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI0_CSPIN, !selected);
}

uint8_t gd32_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  if (devid == SPIDEV_MMCSD(0))
    {
      /* There is no card-detect line wired, so assume the card is present */

      status |= SPI_STATUS_PRESENT;
    }

  return status;
}

#ifdef CONFIG_SPI_CALLBACK
/****************************************************************************
 * Name: gd32_spi0register
 *
 * Description:
 *   Register a media-change callback.  This board has no card-detect line,
 *   so there is nothing to notify; the slot is reported present by
 *   gd32_spi0status().
 *
 ****************************************************************************/

int gd32_spi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                      void *arg)
{
  return OK;
}
#endif

#endif /* CONFIG_GD32VW55X_SPI */
