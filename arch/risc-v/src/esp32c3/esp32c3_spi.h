/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_spi.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SPI_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SPI_H

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

#ifdef CONFIG_ESP32C3_SPI

#include <nuttx/spi/spi.h>

#ifdef CONFIG_ESP32C3_SPI2
#  define ESP32C3_SPI2 2
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_spibus_initialize
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

struct spi_dev_s *esp32c3_spibus_initialize(int port);

/****************************************************************************
 * Name:  esp32c3_spi[0|1]_select and esp32c3_spi[0|1]_status
 *
 * Description:
 *   The external functions, esp32c3_spi[0|1]_select,
 *   esp32c3_spi[0|1]_status, and esp32c3_spi[0|1]_cmddata must be provided
 *   by board-specific logic.
 *   These are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s (include/nuttx/spi/spi.h).
 *   All other methods (including esp32c3_spibus_initialize()) are provided
 *   by common ESP32-C3 logic. To use this common SPI logic on your board:
 *
 *   1. Provide logic in esp32c3_board_initialize() to configure SPI chip
 *      select pins.
 *   2. Provide esp32c3_spi[0|1]_select() and esp32c3_spi[0|1]_status()
 *      functions in your board-specific logic. These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide esp32c3_spi[0|1]_cmddata() functions in your
 *      board-specific logic. These functions will perform cmd/data selection
 *      operations using GPIOs in the way your board is configured.
 *   4. Add a call to esp32c3_spibus_initialize() in your low level
 *      application initialization logic.
 *   5. The handle returned by esp32c3_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_SPI2
void esp32c3_spi2_select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected);
uint8_t esp32c3_spi2_status(struct spi_dev_s *dev, uint32_t devid);
int esp32c3_spi2_cmddata(struct spi_dev_s *dev,
                         uint32_t devid,
                         bool cmd);
#endif

/****************************************************************************
 * Name: esp32c3_spibus_uninitialize
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

int esp32c3_spibus_uninitialize(struct spi_dev_s *dev);

/****************************************************************************
 * Name: esp32c3_spislave_ctrlr_initialize
 *
 * Description:
 *   Initialize the selected SPI Slave bus.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple SPI Slave interfaces)
 *
 * Returned Value:
 *   Valid SPI Slave controller structure reference on success;
 *   NULL on failure.
 *
 ****************************************************************************/

struct spi_slave_ctrlr_s *esp32c3_spislave_ctrlr_initialize(int port);

/****************************************************************************
 * Name: esp32c3_spislave_ctrlr_uninitialize
 *
 * Description:
 *   Uninitialize an SPI Slave bus.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32c3_spislave_ctrlr_uninitialize(struct spi_slave_ctrlr_s *ctrlr);

#endif /* CONFIG_ESP32C3_SPI */

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SPI_H */
