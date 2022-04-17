/****************************************************************************
 * arch/arm/src/rp2040/rp2040_spi.h
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

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_SPI_H
#define __ARCH_ARM_SRC_RP2040_RP2040_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include "hardware/rp2040_spi.h"
#ifdef CONFIG_RP2040_SPI_DMA
#include "rp2040_dmac.h"
#endif

#if defined(CONFIG_RP2040_SPI0) || defined(CONFIG_RP2040_SPI1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This header file defines interfaces to common SPI logic.
 * To use this common SPI logic on your board:
 *
 * 1. Provide logic in rp2040_boardinitialize() to configure SPI chip select
 *    pins.
 * 2. Provide rp2040_spi0/1select() and rp2040_spi0/1status() functions in
 *    your board-specific logic.  These functions will perform chip selection
 *    and status operations using GPIOs in the way your board is configured.
 * 3. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *    rp2040_spi0/1cmddata() functions in your board-specific logic.  These
 *    functions will perform cmd/data selection operations using GPIOs in the
 *    way your board is configured.
 * 4. Your low level board initialization logic should call
 *    rp2040_spibus_initialize.
 * 5. The handle returned by rp2040_spibus_initialize() may then be used to
 *    bind the SPI driver to higher level logic
 *    (e.g., calling  mmcsd_spislotinitialize(), for example, will bind the
 *    SPI driver to the SPI MMC/SD driver).
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   port - Port number
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *rp2040_spibus_initialize(int port);

/****************************************************************************
 * Name:  rp2040_spiXselect, rp2040_spiXstatus, and rp2040_spiXcmddata
 *
 * Description:
 *   These functions must be provided in your board-specific logic.
 *   The rp2040_spi0/1select functions will perform chip selection and the
 *   rp2040_spi0/1status will perform status operations using GPIOs in
 *   the way your board is configured.
 *
 *   If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, then
 *   rp2040_spi0/1cmddata must also be provided.
 *   This functions performs cmd/data selection operations using GPIOs in
 *   the way your board is configured.
 *
 ****************************************************************************/

#ifdef CONFIG_RP2040_SPI0
void  rp2040_spi0select(struct spi_dev_s *dev,
                        uint32_t devid,
                        bool selected);
uint8_t rp2040_spi0status(struct spi_dev_s *dev,
                          uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int rp2040_spi0cmddata(struct spi_dev_s *dev,
                       uint32_t devid,
                       bool cmd);
#endif
#endif

#ifdef CONFIG_RP2040_SPI1
void  rp2040_spi1select(struct spi_dev_s *dev,
                        uint32_t devid,
                        bool selected);
uint8_t rp2040_spi1status(struct spi_dev_s *dev,
                          uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int rp2040_spi1cmddata(struct spi_dev_s *dev,
                       uint32_t devid,
                       bool cmd);
#endif
#endif

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from spi0/1select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spi_flush(struct spi_dev_s *dev);

/****************************************************************************
 * Name: rp2040_spiXregister
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD driver when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s) must
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
#ifdef CONFIG_RP2040_SPI0
int rp2040_spi0register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif

#ifdef CONFIG_RP2040_SPI1
int rp2040_spi1register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RP2040_SPI0/1 */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_SPI_H */
