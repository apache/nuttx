/****************************************************************************
 * boards/hc/m9s12/ne64badge/src/m9s12_spi.c
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

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "ne64badge.h"

#if defined(CONFIG_HCS12_SPI)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hcs12_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NE64 Badge board.
 *
 ****************************************************************************/

void weak_function hcs12_spidev_initialize(void)
{
}

/****************************************************************************
 * Name:  hcs12_spiselect and hcs12_spistatus
 *
 * Description:
 *   The external functions, hcs12_spiselect and hcs12_spistatus must be
 *   provided by board-specific logic. They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h).
 *   All other methods (including hcs12_spibus_initialize())
 *   are provided by common HCS12 logic. To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in hcs12_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide hcs12_spiselect() and hcs12_spistatus() functions in your
 *      board-specific logic. These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to hcs12_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by hcs12_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void hcs12_spiselect(FAR struct spi_dev_s *dev,
                     uint32_t devid, bool selected)
{
}

uint8_t hcs12_spistatus(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

#endif /* CONFIG_HCS12_SPI */
