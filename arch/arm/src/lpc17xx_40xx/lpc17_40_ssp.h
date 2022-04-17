/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_ssp.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_SSP_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_SSP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/spi.h>

#include "hardware/lpc17_40_ssp.h"

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_sspbus_initialize
 *
 * Description:
 *   Initialize the selected SSP port.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *lpc17_40_sspbus_initialize(int port);

/****************************************************************************
 * Name:
 * lpc17_40_ssp0/ssp1select, lpc17_40_ssp0/ssp1status, and
 * lpc17_40_ssp0/ssp1cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   are implementations of the select, status, and cmddata methods of the
 *   SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods including lpc17_40_sspbus_initialize()) are provided
 *   by common LPC17xx/LPC40xx logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in lpc17_40_boardinitialize() to configure SSP chip
 *      select pins.
 *   2. Provide lpc17_40_ssp0/ssp1select() and lpc17_40_ssp0/ssp1status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      lpc17_40_ssp0/ssp1cmddata() functions in your board-specific logic.
 *      These functions will perform cmd/data selection operations using
 *      GPIOs in the way your board is configured.
 *   3. Add a call to lpc17_40_sspbus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by lpc17_40_sspbus_initialize() may then be used
 *      to bind the SSP driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SSP driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SSP0
void lpc17_40_ssp0select(struct spi_dev_s *dev,
                         uint32_t devid, bool selected);
uint8_t lpc17_40_ssp0status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int lpc17_40_ssp0cmddata(struct spi_dev_s *dev,
                         uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC17_40_SSP1
void lpc17_40_ssp1select(struct spi_dev_s *dev,
                         uint32_t devid, bool selected);
uint8_t lpc17_40_ssp1status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int lpc17_40_ssp1cmddata(struct spi_dev_s *dev,
                         uint32_t devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: ssp_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from ssp0/1select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1)
void ssp_flush(struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: lpc17_40_ssp0/1register
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
#ifdef CONFIG_LPC17_40_SSP0
int lpc17_40_ssp0register(struct spi_dev_s *dev,
                          spi_mediachange_t callback,
                          void *arg);
#endif

#ifdef CONFIG_LPC17_40_SSP1
int lpc17_40_ssp1register(struct spi_dev_s *dev,
                          spi_mediachange_t callback,
                          void *arg);
#endif
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LPC17_40_SSP0 || CONFIG_LPC17_40_SSP1 */
#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_SSP_H */
