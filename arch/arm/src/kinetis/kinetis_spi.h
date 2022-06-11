/****************************************************************************
 * arch/arm/src/kinetis/kinetis_spi.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_SPI_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/kinetis_dspi.h"

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_spibus_initialize
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

struct spi_dev_s *kinetis_spibus_initialize(int bus);

/****************************************************************************
 * Name:
 *   kinetis_spi[n]select, kinetis_spi[n]status, and kinetis_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s
  *  (see include/nuttx/spi/spi.h). All other methods including
 *   kinetis_spibus_initialize()) are provided by common Kinetis logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in kinetis_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide kinetis_spi[n]select() and kinetis_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      kinetis_spi[n]cmddata() functions in your board-specific logic.
 *      These functions will perform cmd/data selection operations using
 *      GPIOs in the way your board is configured.
 *   3. Add a call to kinetis_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by kinetis_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI0
void  kinetis_spi0select(struct spi_dev_s *dev,
                         uint32_t devid, bool selected);
uint8_t kinetis_spi0status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int kinetis_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif
#ifdef CONFIG_KINETIS_SPI1
void  kinetis_spi1select(struct spi_dev_s *dev,
                         uint32_t devid, bool selected);
uint8_t kinetis_spi1status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int kinetis_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif
#ifdef CONFIG_KINETIS_SPI2
void  kinetis_spi2select(struct spi_dev_s *dev,
                         uint32_t devid, bool selected);
uint8_t kinetis_spi2status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int kinetis_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: ssp_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from spi[n]select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#if defined(CONFIG_KINETIS_SPI0) || defined(CONFIG_KINETIS_SPI1) || defined(CONFIG_KINETIS_SPI2)
struct spi_dev_s;
void spi_flush(struct spi_dev_s *dev);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_SPI_H */
