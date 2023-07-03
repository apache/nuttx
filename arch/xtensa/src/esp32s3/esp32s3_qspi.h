/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_qspi.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_QSPI_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/qspi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI

#ifdef CONFIG_ESP32S3_SPI2
#  define ESP32S3_SPI2 2
#endif

#ifdef CONFIG_ESP32S3_SPI3
#  define ESP32S3_SPI3 3
#endif

/****************************************************************************
 * Public Function Prototypes
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
 * Name: esp32s3_qspibus_set_attr
 *
 * Description:
 *   Set attribution of QSPI bus transfer.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   dummies    - Number of dummy cycles, this only works in command
 *                transfer, not works in memory transfer
 *   addr_lines - Number of address transmiting I/O pins
 *   data_lines - Number of data transmiting I/O pins
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_qspibus_set_attr(struct qspi_dev_s *dev,
                             uint8_t dummies,
                             uint8_t addr_lines,
                             uint8_t data_lines);

/****************************************************************************
 * Name: esp32s3_qspibus_initialize
 *
 * Description:
 *   Initialize the selected QSPI bus.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple QSPI interfaces)
 *
 * Returned Value:
 *   Valid QSPI device structure reference on success; NULL on failure
 *
 ****************************************************************************/

struct qspi_dev_s *esp32s3_qspibus_initialize(int port);

/****************************************************************************
 * Name: esp32s3_qspibus_uninitialize
 *
 * Description:
 *   Uninitialize an QSPI bus.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_qspibus_uninitialize(struct qspi_dev_s *dev);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ESP32S3_SPI */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_QSPI_H */
