/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_spi.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SPI_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s2_soc.h"
#include "soc/spi_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI_LCD_SRGB_MODE_EN : R/W; bitpos: [31]; default: 0;
 * 1: Enable LCD mode output vsync, hsync, de. 0: Disable.
 */

#define SPI_LCD_SRGB_MODE_EN    (BIT(31))
#define SPI_LCD_SRGB_MODE_EN_M  (SPI_LCD_SRGB_MODE_EN_V << SPI_LCD_SRGB_MODE_EN_S)
#define SPI_LCD_SRGB_MODE_EN_V  0x00000001
#define SPI_LCD_SRGB_MODE_EN_S  31

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SPI_H */
