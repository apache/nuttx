/****************************************************************************
 * arm/arm/src/efm32/efm32_spi.h
 *
 *   Copyright (C) 2009-2013 Bouteville Pierre-Noel. All rights reserved.
 *   Author: Bouteville Pierre-Noel <pnb990@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_EFM32_EFM32_SPI_H
#define __ARCH_ARM_EFM32_EFM32_SPI_H

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

/****************************************************************************
 * Name: efm32_spi_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   port - SPI port number to initialize.  One of {0,1,2}
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s;
struct spi_dev_s *efm32_spi_initialize(int port);

/*****************************************************************************
 * Name:  efm32_spi[n]_select,  efm32_spi[n]_status, and efm32_spi[n]_cmddata
 *
 * Description:
 *   The external functions, efm32_spi[n]_select, efm32_spi[n]_status, and
 *   efm32_spi[n]_cmddata must be provided by board-specific logic.  These
 *   are implementations of the select, status, and cmddata methods of the
 *   SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including up_spiinitialize()) are provided by common
 *   EFM32 logic.  To use this common SPI logic on your board:
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
 *   4. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   5. The handle returned by up_spiinitialize() may then be used to bind
 *      the  SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_USART0_ISSPI
void efm32_spi0_select(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool selected);
uint8_t efm32_spi0_status(struct spi_dev_s *dev, enum spi_dev_e devid);
int efm32_spi0_cmddata(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool cmd);
#endif

#ifdef CONFIG_EFM32_USART1_ISSPI
void efm32_spi1_select(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool selected);
uint8_t efm32_spi1_status(struct spi_dev_s *dev, enum spi_dev_e devid);
int efm32_spi1_cmddata(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool cmd);
#endif

#ifdef CONFIG_EFM32_USART2_ISSPI
void efm32_spi2_select(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool selected);
uint8_t efm32_spi2_status(struct spi_dev_s *dev, enum spi_dev_e devid);
int efm32_spi2_cmddata(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool cmd);
#endif

#endif /* __ARCH_ARM_EFM32_EFM32_SPI_H */
