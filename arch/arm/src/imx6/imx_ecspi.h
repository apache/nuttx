/****************************************************************************
 * arch/arm/src/imx6/imx_ecspi.h
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

#ifndef __ARCH_ARM_IMX6_IMX_ECSPI_H
#define __ARCH_ARM_IMX6_IMX_ECSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/spi/spi.h>

#include "hardware/imx_ecspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif /* __cplusplus */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct spi_dev_s; /* Forward reference */

/****************************************************************************
 * Name: imx_spibus_initialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select. However,
 *   If multiple devices on on the bus, then multiple chip selects will be
 *   required.  Therefore, all GPIO chip management is deferred to board-
 *   specific logic.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *imx_spibus_initialize(int port);

/****************************************************************************
 * The external functions, imx_spiselect, imx_spistatus, and imx_cmddata must
 * be provided by board-specific logic.  These are implementations of the
 * select and status methods of the SPI interface defined by struct spi_ops_s
 * (see include/nuttx/spi/spi.h).  All other methods (including
 * imx_spibus_initialize()) are provided by common logic.
 * To use this common SPI logic on your board:
 *
 *   1. Provide imx_spiselect() and imx_spistatus() functions in your
 *      board-specific logic.  This function will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration, provide
 *      the imx_spicmddata() function in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to imx_spibus_initialize() in your low level
 *      initialization logic
 *   4. The handle returned by imx_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_IMX6_ECSPI1
void imx_spi1select(struct spi_dev_s *dev,
                    uint32_t devid, bool selected);
uint8_t imx_spi1status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_IMX6_ECSPI2
void imx_spi2select(struct spi_dev_s *dev,
                    uint32_t devid, bool selected);
uint8_t imx_spi2status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_IMX6_ECSPI3
void imx_spi3select(struct spi_dev_s *dev,
                    uint32_t devid, bool selected);
uint8_t imx_spi3status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_IMX6_ECSPI4
void imx_spi4select(struct spi_dev_s *dev,
                    uint32_t devid, bool selected);
uint8_t imx_spi4status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_IMX6_ECSPI5
void imx_spi5select(struct spi_dev_s *dev,
                    uint32_t devid, bool selected);
uint8_t imx_spi5status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_IMX6_IMX_ECSPI_H */
