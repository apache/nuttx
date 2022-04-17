/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_spi.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_SPI_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include "hardware/cxd56_spi.h"
#ifdef CONFIG_CXD56_DMAC
#include "cxd56_dmac.h"
#endif

#if defined(CONFIG_CXD56_SPI0) || defined(CONFIG_CXD56_SPI3) || \
    defined(CONFIG_CXD56_SPI4) || defined(CONFIG_CXD56_SPI5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This header file defines interfaces to common SPI logic.
 * To use this common SPI logic on your board:
 *
 * 1. Provide logic in cxd56_boardinitialize() to configure SPI chip select
 *    pins.
 * 2. Provide cxd56_spi0/1select() and cxd56_spi0/1status() functions in your
 *    board-specific logic.  These functions will perform chip selection
 *    and status operations using GPIOs in the way your board is configured.
 * 3. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *    cxd56_spi0/1cmddata() functions in your board-specific logic.  These
 *    functions will perform cmd/data selection operations using GPIOs in the
 *    way your board is configured.
 * 4. Your low level board initialization logic should call
 *    cxd56_spibus_initialize.
 * 5. The handle returned by cxd56_spibus_initialize() may then be used to
 *    bind the SPI driver to higher level logic
 *    (e.g., calling  mmcsd_spislotinitialize(), for example, will bind the
 *    SPI driver to the SPI MMC/SD driver).
 */

#define CXD56_SPI_DMAC_CHTYPE_TX (0)
#define CXD56_SPI_DMAC_CHTYPE_RX (1)

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
 * Name: cxd56_spibus_initialize
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

struct spi_dev_s *cxd56_spibus_initialize(int port);

/****************************************************************************
 * Name: cxd56_spi_dmaconfig
 *
 * Description:
 *   Enable DMA configuration.
 *
 * Input Parameter:
 *   port   - Port number
 *   chtype - Channel type(TX or RX)
 *   handle - DMA channel handle
 *   conf   - DMA configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_DMAC
void cxd56_spi_dmaconfig(int port, int chtype, DMA_HANDLE handle,
                         dma_config_t *conf);
#endif

/****************************************************************************
 * Name:  cxd56_spiXselect, cxd56_spiXstatus, and cxd56_spiXcmddata
 *
 * Description:
 *   These functions must be provided in your board-specific logic.
 *   The cxd56_spi0/1select functions will perform chip selection and the
 *   cxd56_spi0/1status will perform status operations using GPIOs in
 *   the way your board is configured.
 *
 *   If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, then
 *   cxd56_spi0/1cmddata must also be provided.
 *   This functions performs cmd/data selection operations using GPIOs in
 *   the way your board is configured.
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_SPI0
void  cxd56_spi0select(struct spi_dev_s *dev,
                       uint32_t devid,
                       bool selected);
uint8_t cxd56_spi0status(struct spi_dev_s *dev,
                         uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int cxd56_spi0cmddata(struct spi_dev_s *dev,
                      uint32_t devid,
                      bool cmd);
#endif
#endif

#ifdef CONFIG_CXD56_SPI3
void  cxd56_spi3select(struct spi_dev_s *dev,
                       uint32_t devid,
                       bool selected);
uint8_t cxd56_spi3status(struct spi_dev_s *dev,
                         uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int cxd56_spi3cmddata(struct spi_dev_s *dev,
                      uint32_t devid,
                      bool cmd);
#endif
#endif

#ifdef CONFIG_CXD56_SPI4
void  cxd56_spi4select(struct spi_dev_s *dev,
                       uint32_t devid,
                       bool selected);
uint8_t cxd56_spi4status(struct spi_dev_s *dev,
                         uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int cxd56_spi4cmddata(struct spi_dev_s *dev,
                      uint32_t devid,
                      bool cmd);
#endif
#endif

#ifdef CONFIG_CXD56_SPI5
void  cxd56_spi5select(struct spi_dev_s *dev,
                       uint32_t devid,
                       bool selected);
uint8_t cxd56_spi5status(struct spi_dev_s *dev,
                         uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int cxd56_spi5cmddata(struct spi_dev_s *dev,
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
 * Name: cxd56_spiXregister
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
#ifdef CONFIG_CXD56_SPI0
int cxd56_spi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg);
#endif

#ifdef CONFIG_CXD56_SPI3
int cxd56_spi3register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg);
#endif

#ifdef CONFIG_CXD56_SPI4
int cxd56_spi4register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg);
#endif

#ifdef CONFIG_CXD56_SPI5
int cxd56_spi5register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg);
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_CXD56_SPI0/3/4/5 */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_SPI_H */
