/****************************************************************************
 * arch/arm/src/nrf52/nrf52_spi.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_SPI_H
#define __ARCH_ARM_SRC_NRF52_NRF52_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *nrf52_spibus_initialize(int port);

/****************************************************************************
 * Name:  nrf52_spi0/1/...select and nrf52_spi0/1/...status
 *
 * Description:
 *   The external functions, nrf52_spi0/1/...select, nrf52_spi0/1/...status,
 *   and nrf52_spi0/1/...cmddata must be provided by board-specific logic.
 *   These are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s (include/nuttx/spi/spi.h).
 *   All other methods (including nrf52_spibus_initialize()) are provided by
 *   common NRF52 logic. To use this common SPI logic on your board:
 *
 *   1. Provide logic in nrf52_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide nrf52_spi0/1/...select() and nrf52_spi0/1/...status()
 *      functions in your board-specific logic. These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide nrf52_spi0/1/...cmddata() functions in your
 *      board-specific logic. These functions will perform cmd/data selection
 *      operations using GPIOs in the way your board is configured.
 *   4. Add a calls to nrf52_spibus_initialize() in your low level
 *      application initialization logic.
 *   5. The handle returned by nrf52_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_NRF52_SPI0_MASTER
void nrf52_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t nrf52_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);
int nrf52_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
void nrf52_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t nrf52_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
int nrf52_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_NRF52_SPI2_MASTER
void nrf52_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t nrf52_spi2status(FAR struct spi_dev_s *dev, uint32_t devid);
int nrf52_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
void nrf52_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t nrf52_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
int nrf52_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_SPI_H */
