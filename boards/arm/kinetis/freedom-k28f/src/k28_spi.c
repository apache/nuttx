/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/src/k28_spi.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "kinetis.h"
#include "freedom-k28f.h"

#include <arch/board/board.h>

#if defined(CONFIG_KINETIS_SPI0) || defined(CONFIG_KINETIS_SPI1) || \
    defined(CONFIG_KINETIS_SPI2)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k28_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the KwikStik-K40
 *   board.
 *
 ****************************************************************************/

void weak_function k28_spidev_initialize(void)
{
# warning "Missing logic"
}

/****************************************************************************
 * Name:  kinetis_spi0/1/2select and kinetis_spi0/1/2status
 *
 * Description:
 *   The external functions, kinetis_spi0/1/2select and
 *   kinetis_spi0/1/2status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including kinetis_spibus_initialize())
 *   are provided by common Kinetis logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in kinetis_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide kinetis_spi0/1/2select() and kinetis_spi0/1/2status()
 *      functions in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to kinetis_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by kinetis_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI0
void kinetis_spi0select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_KINETIS_SPI1
void kinetis_spi1select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_KINETIS_SPI2
void kinetis_spi2select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#endif /* CONFIG_KINETIS_SPI0 || CONFIG_KINETIS_SPI1 || CONFIG_KINETIS_SPI2 */
