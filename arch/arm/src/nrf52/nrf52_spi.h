/****************************************************************************
 * arch/arm/src/nrf52/nrf52_spi.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_SPI_H
#define __ARCH_ARM_SRC_NRF52_NRF52_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *nrf52_spibus_initialize(int port);

/****************************************************************************
 * Name:  nrf52_spi0/1/...select and nrf52_spi0/1/...status
 *
 * Description:
 *   The external functions, nrf52_spi0/1/...select, nrf52_spi0/1/...status,
 *   and nrf52_spi0/1/...cmddata must be provided by board-specific logic.
 *   These are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s (include/nuttx/spi/spi.h).
 *   All other methods (including nrf52_spibus_initialize()) are provided by
 *   common NRF52 logic. To use this common SPI logic on your board:
 *
 *   1. Provide logic in nrf52_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide nrf52_spi0/1/...select() and nrf52_spi0/1/...status()
 *      functions in your board-specific logic. These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide nrf52_spi0/1/...cmddata() functions in your
 *      board-specific logic. These functions will perform cmd/data selection
 *      operations using GPIOs in the way your board is configured.
 *   4. Add a calls to nrf52_spibus_initialize() in your low level
 *      application initialization logic.
 *   5. The handle returned by nrf52_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_NRF52_SPI0_MASTER
void nrf52_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t nrf52_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);
int nrf52_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
void nrf52_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t nrf52_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
int nrf52_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_NRF52_SPI2_MASTER
void nrf52_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t nrf52_spi2status(FAR struct spi_dev_s *dev, uint32_t devid);
int nrf52_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
void nrf52_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t nrf52_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
int nrf52_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

/****************************************************************************
 * Name: nrf52_spi0/1/2/3register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD driver when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s) must
 *   be implemented.  These functions implements the registercallback method
 *   of the SPI interface (see include/nuttx/spi/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_NRF52_SPI0_MASTER
int nrf52_spi0register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
int nrf52_spi1register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_NRF52_SPI2_MASTER
int nrf52_spi2register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
int nrf52_spi3register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif
#endif

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_SPI_H */
