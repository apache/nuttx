/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_rspi.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_RSPI_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_RSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct spi_dev_s;

/****************************************************************************
 * Name: rx65n_rspibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   bus number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *rx65n_rspibus_initialize(int bus);

/****************************************************************************
 * Name:  rx65n_rspi0/1/2 select and rx65n_rspi0/1/2 status
 *
 * Description:
 *   The external functions, rx65n_rspi0/1/2 select, rx65n_rspi0/1/2 status,
 *   and rx65n_rspi0/1/2 cmddata must be provided by board-specific logic.
 *   These are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s
 *   (see include/nuttx/spi/spi.h). All other methods (including
 *   rx65n_rspibus_initialize()) are provided by common R656N logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/...select() and stm32_spi1/2/...status()
 *      functions in your board-specific logic.  These functions will
 *      perform chip selection and status operations using GPIOs in the
 *      way your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide stm32_spi1/2/...cmddata() functions in your
 *      board-specific logic. These functions will perform cmd/data
 *      selection operations using GPIOs in the way your board is
 *      configured.
 *   4. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by stm32_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/
#ifdef CONFIG_RX65N_RSPI0
void rx65n_rspi0select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t rx65n_rspi0status(struct spi_dev_s *dev, uint32_t devid);
int rx65n_rspi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_RX65N_RSPI1
void rx65n_rspi1select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t rx65n_rspi1status(struct spi_dev_s *dev, uint32_t devid);
int rx65n_rspi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_RX65N_RSPI2
void rx65n_rspi2select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t rx65n_rspi2status(struct spi_dev_s *dev, uint32_t devid);
int rx65n_rspi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

/****************************************************************************
 * Name: rx65n_rspi0/1/2register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD driver when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s)
 *   must be implemented.  These functions implements the registercallback
 *   method of the SPI interface (see include/nuttx/spi/spi.h for details)
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
#ifdef CONFIG_RX65N_RSPI0
int rx65n_rspi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                        void *arg);
#endif

#ifdef CONFIG_RX65N_RSPI1
int rx65n_rspi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                        void *arg);
#endif

#ifdef CONFIG_RX65N_RSPI2
int rx65n_rspi2register(struct spi_dev_s *dev, spi_mediachange_t callback,
                        void *arg);
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_RSPI_H */
