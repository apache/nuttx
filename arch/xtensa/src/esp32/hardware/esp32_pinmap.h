/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_pinmap.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PINMAP_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PINMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * Peripheral' fixed mapped pins by IOMUX, these GPIO pins can have better
 * speed performance.
 */

/* SPI2 */

#define SPI2_IOMUX_MISOPIN      (12)
#define SPI2_IOMUX_MOSIPIN      (13)
#define SPI2_IOMUX_CLKPIN       (14)
#define SPI2_IOMUX_CSPIN        (15)
#define SPI2_IOMUX_WPPIN        (2)
#define SPI2_IOMUX_HDPIN        (4)

/* SPI3 */

#define SPI3_IOMUX_MISOPIN      (19)
#define SPI3_IOMUX_MOSIPIN      (23)
#define SPI3_IOMUX_CLKPIN       (18)
#define SPI3_IOMUX_CSPIN        (5)
#define SPI3_IOMUX_WPPIN        (22)
#define SPI3_IOMUX_HDPIN        (21)

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_PINMAP_H */
