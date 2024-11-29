/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spi_bitbang.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPI_BITBANG_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPI_BITBANG_H

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

#ifdef CONFIG_ESPRESSIF_SPI_BITBANG

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_bitbang.h>
#include "espressif/esp_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros needed to include/nuttx/spi/spi_bitbang.c. */

#define SPI_SETSCK  esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_CLKPIN, true)
#define SPI_CLRSCK  esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_CLKPIN, false)
#define SPI_SETMOSI esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_MOSIPIN, true)
#define SPI_CLRMOSI esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_MOSIPIN, \
                                  false)
#define SPI_GETMISO esp_gpioread(CONFIG_ESPRESSIF_SPI_BITBANG_MISOPIN)
#define SPI_SETCS   esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_CSPIN, true)
#define SPI_CLRCS   esp_gpiowrite(CONFIG_ESPRESSIF_SPI_BITBANG_CSPIN, false)

/* Calibration value for timing loop */

#define SPI_BITBANG_LOOPSPERMSEC CONFIG_BOARD_LOOPSPERMSEC

/* SPI_PERBIT_NSEC is the minimum time to transfer one bit. This determines
 * the maximum frequency and is also used to calculate delays to achieve
 * other SPI frequencies.
 */

#define SPI_PERBIT_NSEC       100

#define ESPRESSIF_SPI_BITBANG 3

/****************************************************************************
 * Name: esp_spi_bitbang_init
 *
 * Description:
 *   Initialize the SPI bit-bang driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-NULL reference to the SPI driver on success
 *
 ****************************************************************************/

struct spi_dev_s *esp_spi_bitbang_init(void);

/****************************************************************************
 * Name:  esp_spi_bitbang_uninitialize
 *
 * Description:
 *   Destroy an instance of the SPI bit-bang driver.
 *
 * Input Parameters:
 *   dev - device instance, target driver to destroy.
 *
 ****************************************************************************/

void esp_spi_bitbang_uninitialize(struct spi_dev_s *dev);

#endif /* CONFIG_ESPRESSIF_SPI_BITBANG */

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPI_BITBANG_H */
