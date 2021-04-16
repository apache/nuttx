/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/esp32c3_pinmap.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_PINMAP_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_PINMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral's fixed mapped pins by IOMUX, these GPIO pins can have better
 * speed performance.
 */

/* SPI2 */

#define SPI2_IOMUX_MISOPIN      (2)
#define SPI2_IOMUX_MOSIPIN      (7)
#define SPI2_IOMUX_CLKPIN       (6)
#define SPI2_IOMUX_CSPIN        (10)
#define SPI2_IOMUX_WPPIN        (5)
#define SPI2_IOMUX_HDPIN        (4)

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_ESP32C3_PINMAP_H */
