/****************************************************************************
 * arch/arm64/src/imx9/imx9_lpspi.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_LPSPI_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_LPSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/spi/spi.h>

#include "chip.h"
#include "hardware/imx9_lpspi.h"

/****************************************************************************
 * Public Functions Prototypes
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

struct spi_dev_s; /* Forward reference */

/****************************************************************************
 * Name: imx9_lpspibus_initialize
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

struct spi_dev_s *imx9_lpspibus_initialize(int bus);

/****************************************************************************
 * Name: imx9_lpspibus_uninitialize
 *
 * Description:
 *   Unitialize the selected SPI bus if refcount is 1
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_lpspi_uninitialize(struct spi_dev_s *dev);

/****************************************************************************
 * Name:  imx9_lpspi1/2/...select and imx9_lpspi1/2/...status
 *
 * Description:
 *   The external functions, imx9_lpspi1/2/...select,
 *   imx9_lpspi1/2/...status, and imx9_lpspi1/2/...cmddata must be
 *   provided by board-specific logic.  These are implementations of the
 *   select, status, and cmddata methods of the SPI interface defined by
 *   struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   (including imx9_lpspibus_initialize()) are provided by common IMX9
 *   logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in imx9_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide imx9_lpspi1/2/...select() and imx9_lpspi1/2/...status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide imx9_lpspi1/2/...cmddata() functions in your
 *      board-specific logic. These functions will perform cmd/data selection
 *      operations using GPIOs in the way your board is configured.
 *   4. Add a calls to imx9_lpspibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by imx9_lpspibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void imx9_lpspi_select(struct spi_dev_s *dev,
                       uint32_t devid, bool selected);
uint8_t imx9_lpspi_status(struct spi_dev_s *dev, uint32_t devid);
int imx9_lpspi_cmddata(struct spi_dev_s *dev,
                       uint32_t devid, bool cmd);

/****************************************************************************
 * Name: imx9_lpspi1/2/...register
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
int imx9_lpspi_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback,
                        void *arg);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_LPSPI_H */
