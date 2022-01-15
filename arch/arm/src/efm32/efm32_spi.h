/****************************************************************************
 * arch/arm/src/efm32/efm32_spi.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_SPI_H
#define __ARCH_ARM_SRC_EFM32_EFM32_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "efm32_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct spi_dev_s;  /* Forward reference */

/****************************************************************************
 * Name: efm32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   port - SPI port number to initialize.  One of {0,1,2}
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *efm32_spibus_initialize(int port);

/****************************************************************************
 * Name:  efm32_spi[n]_select,  efm32_spi[n]_status, and efm32_spi[n]_cmddata
 *
 * Description:
 *   The external functions, efm32_spi[n]_select, efm32_spi[n]_status, and
 *   efm32_spi[n]_cmddata must be provided by board-specific logic.  These
 *   are implementations of the select, status, and cmddata methods of the
 *   SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including efm32_spibus_initialize()) are provided by
 *   common EFM32 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in efm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide efm32_spi[n]_select() and efm32_spi[n]_status() functions in
 *      your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide efm32_spi[n]_cmddata() functions in your board-specific
 *      logic.  These functions will perform cmd/data selection operations
 *      using GPIOs in the way your board is configured.
 *   4. Add a calls to efm32_spibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by efm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_USART0_ISSPI
void efm32_spi0_select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t efm32_spi0_status(struct spi_dev_s *dev, uint32_t devid);
int efm32_spi0_cmddata(struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#endif

#ifdef CONFIG_EFM32_USART1_ISSPI
void efm32_spi1_select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t efm32_spi1_status(struct spi_dev_s *dev, uint32_t devid);
int efm32_spi1_cmddata(struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#endif

#ifdef CONFIG_EFM32_USART2_ISSPI
void efm32_spi2_select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t efm32_spi2_status(struct spi_dev_s *dev, uint32_t devid);
int efm32_spi2_cmddata(struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#endif

#endif /* __ARCH_ARM_SRC_EFM32_EFM32_SPI_H */
