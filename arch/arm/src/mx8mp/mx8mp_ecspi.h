/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_ecspi.h
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

#ifndef __ARCH_ARM_SRC_MX8MP_MX8MP_ECSPI_H
#define __ARCH_ARM_SRC_MX8MP_MX8MP_ECSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct spi_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_spibus_initialize
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

struct spi_dev_s *mx8mp_spibus_initialize(int bus);

/****************************************************************************
 * Name:
 *   mx8mp_spi[n]select, mx8mp_spi[n]status, and mx8mp_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s
  *  (see include/nuttx/spi/spi.h). All other methods including
 *   mx8mp_spibus_initialize()) are provided by common Kinetis logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in mx8mp_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide mx8mp_spi[n]select() and mx8mp_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      mx8mp_spi[n]cmddata() functions in your board-specific logic.
 *      These functions will perform cmd/data selection operations using
 *      GPIOs in the way your board is configured.
 *   3. Add a call to mx8mp_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by mx8mp_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_MX8MP_SPI1
void  mx8mp_spi1_select(struct spi_dev_s *dev,
                         uint32_t devid, bool selected);
uint8_t mx8mp_spi1_status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int mx8mp_spi1_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_MX8MP_SPI2
void  mx8mp_spi2_select(struct spi_dev_s *dev,
                         uint32_t devid, bool selected);
uint8_t mx8mp_spi2_status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int mx8mp_spi2_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_MX8MP_SPI3
void  mx8mp_spi3_select(struct spi_dev_s *dev,
                         uint32_t devid, bool selected);
uint8_t mx8mp_spi3_status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int mx8mp_spi3_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#endif /* __ARCH_ARM_SRC_MX8MP_MX8MP_ECSPI_H */
