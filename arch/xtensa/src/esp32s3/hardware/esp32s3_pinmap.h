/****************************************************************************
 * arch/xtensa/src/esp32s3/hardware/esp32s3_pinmap.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_PINMAP_H
#define __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_PINMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * Peripheral' fixed mapped pins by IOMUX, these GPIO pins can have better
 * speed performance.
 */

/* USB Serial JTAG */

#define JTAG_IOMUX_USB_DM       (19)
#define JTAG_IOMUX_USB_DP       (20)

/* UART0 */

#define UART0_IOMUX_TXPIN       (43)
#define UART0_IOMUX_RXPIN       (44)
#define UART0_IOMUX_RTSPIN      (15)
#define UART0_IOMUX_CTSPIN      (16)

/* UART1 */

#define UART1_IOMUX_TXPIN       (17)
#define UART1_IOMUX_RXPIN       (18)
#define UART1_IOMUX_RTSPIN      (19)
#define UART1_IOMUX_CTSPIN      (20)

/* SPI2 */

#define SPI2_IOMUX_MISOPIN      (13)
#define SPI2_IOMUX_MOSIPIN      (11)
#define SPI2_IOMUX_CLKPIN       (12)
#define SPI2_IOMUX_CSPIN        (10)
#define SPI2_IOMUX_WPPIN        (14)
#define SPI2_IOMUX_HDPIN        (9)

/* SPI3 */

/* SPI3 have no iomux pins */

#endif /* __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_PINMAP_H */
