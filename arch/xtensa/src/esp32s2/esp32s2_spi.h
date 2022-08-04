/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_spi.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_SPI_H
#define __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_ESP32S2_SPI

#include <nuttx/spi/spi.h>

#ifdef CONFIG_ESP32S2_SPI2
#  define ESP32S2_SPI2 2
#endif

#ifdef CONFIG_ESP32S2_SPI3
#  define ESP32S2_SPI3 3
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus.
 *
 * Input Parameters:
 *   port     - Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *esp32s2_spibus_initialize(int port);

/****************************************************************************
 * Name:  esp32s2_spi[2|3]_select and esp32s2_spi[2|3]_status
 *
 * Description:
 *   The external functions, esp32s2_spi[2|3]_select,
 *   esp32s2_spi[2|3]_status, and esp32s2_spi[2|3]_cmddata must be provided
 *   by board-specific logic.
 *   These are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s (include/nuttx/spi/spi.h).
 *   All other methods (including esp32s2_spibus_initialize()) are provided
 *   by common ESP32-S2 logic. To use this common SPI logic on your board:
 *
 *   1. Provide logic in esp32s2_board_initialize() to configure SPI chip
 *      select pins.
 *   2. Provide esp32s2_spi[2|3]_select() and esp32s2_spi[2|3]_status()
 *      functions in your board-specific logic. These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide esp32s2_spi[2|3]_cmddata() functions in your
 *      board-specific logic. These functions will perform cmd/data selection
 *      operations using GPIOs in the way your board is configured.
 *   4. Add a call to esp32s2_spibus_initialize() in your low level
 *      application initialization logic.
 *   5. The handle returned by esp32s2_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_SPI2
void esp32s2_spi2_select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected);
uint8_t esp32s2_spi2_status(struct spi_dev_s *dev, uint32_t devid);
int esp32s2_spi2_cmddata(struct spi_dev_s *dev,
                         uint32_t devid,
                         bool cmd);
#endif

#ifdef CONFIG_ESP32S2_SPI3
void esp32s2_spi3_select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected);
uint8_t esp32s2_spi3_status(struct spi_dev_s *dev, uint32_t devid);
int esp32s2_spi3_cmddata(struct spi_dev_s *dev,
                         uint32_t devid,
                         bool cmd);
#endif

/****************************************************************************
 * Name: esp32s2_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s2_spibus_uninitialize(struct spi_dev_s *dev);

#endif /* CONFIG_ESP32S2_SPI */

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_SPI_H */
