/****************************************************************************
 * arch/arm/src/max326xx/max326_spim.h
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

#ifndef __ARCH_ARM_STC_MAX326XX_MAX326_SPIM_H
#define __ARCH_ARM_STC_MAX326XX_MAX326_SPIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/spi.h>

#include "chip.h"
#include "hardware/max326_spi.h"

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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: max326_spibus_initialize
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

struct spi_dev_s *max326_spibus_initialize(int bus);

/****************************************************************************
 * Name:  max326_spi[0|1|2]select, max326_spi[0|1|2]status, and
 *        max326_spi[0|1|2]cmddata
 *
 * Description:
 *   The external functions, max326_spi[0|1|2]select,
 *   max326_spi[0|1|2]status, and max326_spi[0|1|2]cmddata must be provided
 *   by board-specific logic.  These are implementations of the select,
 *   status, and cmddata methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).  All other methods
 *   (including max326_spibus_initialize()) are provided by common
 *    MAX326xx logic. To use this common SPI logic on your board:
 *
 *   1. Provide logic in max326_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide max326_spi[0|1|2]select() and max326_spi[0|1|2]status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide max326_spi[0|1|2]cmddata() functions in your
 *      board-specific logic. These functions will perform cmd/data
 *      selection operations using GPIOs in the way your board is configured.
 *   4. Add a calls to max326_spibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by max326_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326XX_SPIM0
void max326_spi0select(struct spi_dev_s *dev,
                       uint32_t devid, bool selected);
uint8_t max326_spi0status(struct spi_dev_s *dev, uint32_t devid);
int max326_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_MAX326XX_SPIM1
void max326_spi1select(struct spi_dev_s *dev,
                       uint32_t devid, bool selected);
uint8_t max326_spi1status(struct spi_dev_s *dev, uint32_t devid);
int max326_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_MAX326XX_SPIMM2
void max326_spi2select(struct spi_dev_s *dev,
                       uint32_t devid, bool selected);
uint8_t max326_spi2status(struct spi_dev_s *dev, uint32_t devid);
int max326_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

/****************************************************************************
 * Name: max326_spi[0|1|2]register
 *
 * Description:
 *   If the board supports a card detect callback to inform the
 *   SPI-based MMC/SD driver when an SD card is inserted or removed,
 *   then CONFIG_SPI_CALLBACK should be defined and the following
 *   function(s) must be implemented.  These functions
 *   implements the registercallback method of the SPI interface
 *   (see include/nuttx/spi/spi.h for details)
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
#ifdef CONFIG_MAX326XX_SPIM0
int max326_spi0register(struct spi_dev_s *dev,
                        spi_mediachange_t callback,
                        void *arg);
#endif

#ifdef CONFIG_MAX326XX_SPIM1
int max326_spi1register(struct spi_dev_s *dev,
                        spi_mediachange_t callback,
                        void *arg);
#endif

#ifdef CONFIG_MAX326XX_SPIM2
int max326_spi2register(struct spi_dev_s *dev,
                        spi_mediachange_t callback,
                        void *arg);
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_STC_MAX326XX_MAX326_SPIM_H */
