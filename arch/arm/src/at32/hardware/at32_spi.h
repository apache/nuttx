/****************************************************************************
 * arch/arm/src/at32/hardware/at32_spi.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32_SPI_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI version **************************************************************/

#define HAVE_SPI_I2S                /* Some SPI peripherals have I2S mode */
#define HAVE_SPI_I2S_ASTRT          /* Supports I2S asynchronous start capability */
#define HAVE_SPI_TI_MODE            /* Have Motorola and TI frame modes */
#define HAVE_SPI_ARB_DATA_SIZE      /* Supports arbitrary data size from 4-16 bits */
#define HAVE_SPI_FIFOS              /* Have Tx/Rx FIFOs */
#define HAVE_SPI_NSSP               /* Have NSS Pulse Management in master mode */

/* Maximum allowed speed as per specifications for all SPIs */

#define AT32_SPI_CLK_MAX            14400000UL

/* Register Offsets *********************************************************/

#define AT32_SPI_CTRL1_OFFSET       0x00  /* SPI Control Register 1 (16-bit) */
#define AT32_SPI_CTRL2_OFFSET       0x04  /* SPI control register 2 (16-bit) */
#define AT32_SPI_STS_OFFSET         0x08  /* SPI status register (16-bit) */
#define AT32_SPI_DT_OFFSET          0x0c  /* SPI data register (16-bit) */
#define AT32_SPI_CPOLY_OFFSET       0x10  /* SPI CRC polynomial register (16-bit) */
#define AT32_SPI_RCRC_OFFSET        0x14  /* SPI Rx CRC register (16-bit) */
#define AT32_SPI_TCRC_OFFSET        0x18  /* SPI Tx CRC register (16-bit) */

#if defined(HAVE_SPI_I2S)
#  define AT32_SPI_I2SCTRL_OFFSET   0x1c  /* I2S configuration register */
#  define AT32_SPI_I2SCLKP_OFFSET   0x20  /* I2S prescaler register */
#endif

/* Register Addresses *******************************************************/

#if AT32_NSPI > 0
#  define AT32_SPI1_CTRL1          (AT32_SPI1_BASE + AT32_SPI_CTRL1_OFFSET)
#  define AT32_SPI1_CTRL2          (AT32_SPI1_BASE + AT32_SPI_CTRL2_OFFSET)
#  define AT32_SPI1_STS            (AT32_SPI1_BASE + AT32_SPI_STS_OFFSET)
#  define AT32_SPI1_DT             (AT32_SPI1_BASE + AT32_SPI_DT_OFFSET)
#  define AT32_SPI1_CPOLY          (AT32_SPI1_BASE + AT32_SPI_CPOLY_OFFSET)
#  define AT32_SPI1_RCRC           (AT32_SPI1_BASE + AT32_SPI_RCRC_OFFSET)
#  define AT32_SPI1_TCRC           (AT32_SPI1_BASE + AT32_SPI_TCRC_OFFSET)
#  if defined(HAVE_SPI_I2S)
#    define AT32_SPI1_I2SCTRL      (AT32_SPI1_BASE + AT32_SPI_I2SCTRL_OFFSET)
#    define AT32_SPI1_I2SCLKP      (AT32_SPI1_BASE + AT32_SPI_I2SCLKP_OFFSET)
#  endif
#endif

#if AT32_NSPI > 1
#  define AT32_SPI2_CTRL1          (AT32_SPI2_BASE + AT32_SPI_CTRL1_OFFSET)
#  define AT32_SPI2_CTRL2          (AT32_SPI2_BASE + AT32_SPI_CTRL2_OFFSET)
#  define AT32_SPI2_STS            (AT32_SPI2_BASE + AT32_SPI_STS_OFFSET)
#  define AT32_SPI2_DT             (AT32_SPI2_BASE + AT32_SPI_DT_OFFSET)
#  define AT32_SPI2_CPOLY          (AT32_SPI2_BASE + AT32_SPI_CPOLY_OFFSET)
#  define AT32_SPI2_RCRC           (AT32_SPI2_BASE + AT32_SPI_RCRC_OFFSET)
#  define AT32_SPI2_TCRC           (AT32_SPI2_BASE + AT32_SPI_TCRC_OFFSET)
#  if defined(HAVE_SPI_I2S)
#    define AT32_SPI2_I2SCTRL      (AT32_SPI2_BASE + AT32_SPI_I2SCTRL_OFFSET)
#    define AT32_SPI2_I2SCLKP      (AT32_SPI2_BASE + AT32_SPI_I2SCLKP_OFFSET)
#  endif
#endif

#if AT32_NSPI > 2
#  define AT32_SPI3_CTRL1          (AT32_SPI3_BASE + AT32_SPI_CTRL1_OFFSET)
#  define AT32_SPI3_CTRL2          (AT32_SPI3_BASE + AT32_SPI_CTRL2_OFFSET)
#  define AT32_SPI3_STS            (AT32_SPI3_BASE + AT32_SPI_STS_OFFSET)
#  define AT32_SPI3_DT             (AT32_SPI3_BASE + AT32_SPI_DT_OFFSET)
#  define AT32_SPI3_CPOLY          (AT32_SPI3_BASE + AT32_SPI_CPOLY_OFFSET)
#  define AT32_SPI3_RCRC           (AT32_SPI3_BASE + AT32_SPI_RCRC_OFFSET)
#  define AT32_SPI3_TCRC           (AT32_SPI3_BASE + AT32_SPI_TCRC_OFFSET)
#  if defined(HAVE_SPI_I2S)
#    define AT32_SPI3_I2SCTRL      (AT32_SPI3_BASE + AT32_SPI_I2SCTRL_OFFSET)
#    define AT32_SPI3_I2SCLKP      (AT32_SPI3_BASE + AT32_SPI_I2SCLKP_OFFSET)
#  endif
#endif

#if AT32_NSPI > 3
#  define AT32_SPI4_CTRL1          (AT32_SPI4_BASE + AT32_SPI_CTRL1_OFFSET)
#  define AT32_SPI4_CTRL2          (AT32_SPI4_BASE + AT32_SPI_CTRL2_OFFSET)
#  define AT32_SPI4_STS            (AT32_SPI4_BASE + AT32_SPI_STS_OFFSET)
#  define AT32_SPI4_DT             (AT32_SPI4_BASE + AT32_SPI_DT_OFFSET)
#  define AT32_SPI4_CPOLY          (AT32_SPI4_BASE + AT32_SPI_CPOLY_OFFSET)
#  define AT32_SPI4_RCRC           (AT32_SPI4_BASE + AT32_SPI_RCRC_OFFSET)
#  define AT32_SPI4_TCRC           (AT32_SPI4_BASE + AT32_SPI_TCRC_OFFSET)
#  if defined(HAVE_SPI_I2S)
#    define AT32_SPI4_I2SCTRL      (AT32_SPI4_BASE + AT32_SPI_I2SCTRL_OFFSET)
#    define AT32_SPI4_I2SCLKP      (AT32_SPI4_BASE + AT32_SPI_I2SCLKP_OFFSET)
#  endif
#endif

/* Register Bitfield Definitions ********************************************/

/* SPI Control Register 1 */

#define SPI_CTRL1_CLKPHA            (1 << 0) /* Clock phase */
#define SPI_CTRL1_CLKPOL            (1 << 1) /* Clock polarity */
#define SPI_CTRL1_MSTEN             (1 << 2) /* Master enable */

#define SPI_CTRL1_MDIV_SHIFT        (3) /* Master clock frequency division */
#define SPI_CTRL1_MDIV_MASK         (7 << SPI_CTRL1_MDIV_SHIFT)
#  define SPI_CTRL1_MDIV_2          (0 << SPI_CTRL1_MDIV_SHIFT) /* Div 2 */
#  define SPI_CTRL1_MDIV_4          (1 << SPI_CTRL1_MDIV_SHIFT) /* Div 4 */
#  define SPI_CTRL1_MDIV_8          (2 << SPI_CTRL1_MDIV_SHIFT) /* Div 8 */
#  define SPI_CTRL1_MDIV_16         (3 << SPI_CTRL1_MDIV_SHIFT) /* Div 16 */
#  define SPI_CTRL1_MDIV_32         (4 << SPI_CTRL1_MDIV_SHIFT) /* Div 32 */
#  define SPI_CTRL1_MDIV_64         (5 << SPI_CTRL1_MDIV_SHIFT) /* Div 64 */
#  define SPI_CTRL1_MDIV_128        (6 << SPI_CTRL1_MDIV_SHIFT) /* Div 128 */
#  define SPI_CTRL1_MDIV_256        (7 << SPI_CTRL1_MDIV_SHIFT) /* Div 256 */
#  define SPI_CTRL1_MDIV_512        (8 << SPI_CTRL1_MDIV_SHIFT) /* Div 512 */
#  define SPI_CTRL1_MDIV_1024       (9 << SPI_CTRL1_MDIV_SHIFT) /* Div 1024 */

#define SPI_CTRL1_SPIEN             (1 << 6)  /* SPI enable */
#define SPI_CTRL1_LTF               (1 << 7)  /* LSB transmit first */
#define SPI_CTRL1_SWCSIL            (1 << 8)  /* Software CS internal level */
#define SPI_CTRL1_SWCSEN            (1 << 9)  /* Software CS enable */
#define SPI_CTRL1_ORA               (1 << 10) /* Only receive active */
#define SPI_CTRL1_FBN               (1 << 11) /* frame bit num */
#define SPI_CTRL1_NTC               (1 << 12) /* Next transmission CRC */
#define SPI_CTRL1_CCEN              (1 << 13) /* CRC calculation enable */
#define SPI_CTRL1_SLBTD             (1 << 14) /* Single line bidirectional half-duplex transmission direction */
#define SPI_CTRL1_SLBEN             (1 << 15) /* Single line bidirectional halfduplex enable */

/* SPI Control Register 2 */

#define SPI_CTRL2_DMAREN            (1 << 0) /* DMA receive enable */
#define SPI_CTRL2_DMATEN            (1 << 1) /* DMA transmit enable */
#define SPI_CTRL2_HWCSOE            (1 << 2) /* Hardware CS output enable */
#define SPI_CTRL2_TIEN              (1 << 4) /* TI mode enable */
#define SPI_CTRL2_ERRIE             (1 << 5) /* Error interrupt enable */
#define SPI_CTRL2_RDBFIE            (1 << 6) /* Receive data buffer full interrupt enable */
#define SPI_CTRL2_TDBEIE            (1 << 7) /* Transmit data buffer empty interrupt enable */
#define SPI_CTRL2_MDIV              (1 << 8) /* Master clock frequency division */
#define SPI_CTRL2_MDIV3EN           (1 << 9) /* Master clock frequency3 division enable */

/* SPI status register */

#define SPI_STS_RDBF                (1 << 0) /* Receive data buffer full */
#define SPI_STS_TDBE                (1 << 1) /* Transmit data buffer empty */
#define SPI_STS_ACS                 (1 << 2) /* Audio channel state */
#define SPI_STS_TUERR               (1 << 3) /* Transmitter underload error */
#define SPI_STS_CCERR               (1 << 4) /* CRC calculation error */
#define SPI_STS_MMERR               (1 << 5) /* Master mode error */
#define SPI_STS_ROERR               (1 << 6) /* Receiver overflow error */
#define SPI_STS_BF                  (1 << 7) /* Busy flag */
#define SPI_STS_CSPAS               (1 << 8) /* CS pulse abnormal setting fiag */

/* I2S configuration register */

/* I2S prescaler register */

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32_SPI_H */
