/****************************************************************************
 * arch/arm/src/stm32u5/hardware/stm32u585xx_spi.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32U585XX_SPI_H
#define __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32U585XX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32U5_STM32U585XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum allowed speed as per data sheet for all SPIs */

#  define STM32_SPI_CLK_MAX      160000000UL

/* Register Offsets *********************************************************/

#define STM32_SPI_CR1_OFFSET      0x0000  /* SPI/I2S Control Register 1 */
#define STM32_SPI_CR2_OFFSET      0x0004  /* SPI control register 2 */
#define STM32_SPI_CFG1_OFFSET     0x0008  /* SPI configuration register 1 */
#define STM32_SPI_CFG2_OFFSET     0x000C  /* SPI configuration register 2 */
#define STM32_SPI_IER_OFFSET      0x0010  /* SPI/I2S interrupt enable register */
#define STM32_SPI_SR_OFFSET       0x0014  /* SPI/I2S status register */
#define STM32_SPI_IFCR_OFFSET     0x0018  /* SPI/I2S interrupt/status flags clear register */
#define STM32_SPI_TXDR_OFFSET     0x0020  /* SPI/I2S transmit data register */
#define STM32_SPI_RXDR_OFFSET     0x0030  /* SPI/I2S receive data register */
#define STM32_SPI_CRCPOLY_OFFSET  0x0040  /* SPI/I2S SPI polynominal register */
#define STM32_SPI_TXCRC_OFFSET    0x0044  /* SPI/I2S SPI transmitter CRC register */
#define STM32_SPI_RXCRC_OFFSET    0x0048  /* SPI/I2S SPI receiver CRC register */
#define STM32_SPI_UDRDR_OFFSET    0x004C  /* SPI/I2S SPI underrun data register */

/* Register Addresses *******************************************************/

#if STM32_NSPI > 0
#  define STM32_SPI1_CR1          (STM32_SPI1_BASE+STM32_SPI_CR1_OFFSET)
#  define STM32_SPI1_CR2          (STM32_SPI1_BASE+STM32_SPI_CR2_OFFSET)
#  define STM32_SPI1_CFG1         (STM32_SPI1_BASE+STM32_SPI_CFG1_OFFSET)
#  define STM32_SPI1_CFG2         (STM32_SPI1_BASE+STM32_SPI_CFG2_OFFSET)
#  define STM32_SPI1_IER          (STM32_SPI1_BASE+STM32_SPI_IER_OFFSET)
#  define STM32_SPI1_SR           (STM32_SPI1_BASE+STM32_SPI_SR_OFFSET)
#  define STM32_SPI1_IFCR         (STM32_SPI1_BASE+STM32_SPI_IFCR_OFFSET)
#  define STM32_SPI1_TXDR         (STM32_SPI1_BASE+STM32_SPI_TXDR_OFFSET)
#  define STM32_SPI1_RXDR         (STM32_SPI1_BASE+STM32_SPI_RXDR_OFFSET)
#  define STM32_SPI1_CRCPOLY      (STM32_SPI1_BASE+STM32_SPI_CRCPOLY_OFFSET)
#  define STM32_SPI1_TXCRC        (STM32_SPI1_BASE+STM32_SPI_TXCRC_OFFSET)
#  define STM32_SPI1_RXCRC        (STM32_SPI1_BASE+STM32_SPI_RXCRC_OFFSET)
#  define STM32_SPI1_UDRDR        (STM32_SPI1_BASE+STM32_SPI_UDRDR_OFFSET)
#endif

#if STM32_NSPI > 1
#  define STM32_SPI2_CR1          (STM32_SPI2_BASE+STM32_SPI_CR1_OFFSET)
#  define STM32_SPI2_CR2          (STM32_SPI2_BASE+STM32_SPI_CR2_OFFSET)
#  define STM32_SPI2_CFG1         (STM32_SPI2_BASE+STM32_SPI_CFG1_OFFSET)
#  define STM32_SPI2_CFG2         (STM32_SPI2_BASE+STM32_SPI_CFG2_OFFSET)
#  define STM32_SPI2_IER          (STM32_SPI2_BASE+STM32_SPI_IER_OFFSET)
#  define STM32_SPI2_SR           (STM32_SPI2_BASE+STM32_SPI_SR_OFFSET)
#  define STM32_SPI2_IFCR         (STM32_SPI2_BASE+STM32_SPI_IFCR_OFFSET)
#  define STM32_SPI2_TXDR         (STM32_SPI2_BASE+STM32_SPI_TXDR_OFFSET)
#  define STM32_SPI2_RXDR         (STM32_SPI2_BASE+STM32_SPI_RXDR_OFFSET)
#  define STM32_SPI2_CRCPOLY      (STM32_SPI2_BASE+STM32_SPI_CRCPOLY_OFFSET)
#  define STM32_SPI2_TXCRC        (STM32_SPI2_BASE+STM32_SPI_TXCRC_OFFSET)
#  define STM32_SPI2_RXCRC        (STM32_SPI2_BASE+STM32_SPI_RXCRC_OFFSET)
#  define STM32_SPI2_UDRDR        (STM32_SPI2_BASE+STM32_SPI_UDRDR_OFFSET)
#endif

#if STM32_NSPI > 2
#  define STM32_SPI3_CR1          (STM32_SPI3_BASE+STM32_SPI_CR1_OFFSET)
#  define STM32_SPI3_CR2          (STM32_SPI3_BASE+STM32_SPI_CR2_OFFSET)
#  define STM32_SPI3_CFG1         (STM32_SPI3_BASE+STM32_SPI_CFG1_OFFSET)
#  define STM32_SPI3_CFG2         (STM32_SPI3_BASE+STM32_SPI_CFG2_OFFSET)
#  define STM32_SPI3_IER          (STM32_SPI3_BASE+STM32_SPI_IER_OFFSET)
#  define STM32_SPI3_SR           (STM32_SPI3_BASE+STM32_SPI_SR_OFFSET)
#  define STM32_SPI3_IFCR         (STM32_SPI3_BASE+STM32_SPI_IFCR_OFFSET)
#  define STM32_SPI3_TXDR         (STM32_SPI3_BASE+STM32_SPI_TXDR_OFFSET)
#  define STM32_SPI3_RXDR         (STM32_SPI3_BASE+STM32_SPI_RXDR_OFFSET)
#  define STM32_SPI3_CRCPOLY      (STM32_SPI3_BASE+STM32_SPI_CRCPOLY_OFFSET)
#  define STM32_SPI3_TXCRC        (STM32_SPI3_BASE+STM32_SPI_TXCRC_OFFSET)
#  define STM32_SPI3_RXCRC        (STM32_SPI3_BASE+STM32_SPI_RXCRC_OFFSET)
#  define STM32_SPI3_UDRDR        (STM32_SPI3_BASE+STM32_SPI_UDRDR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* SPI Control Register 1 */

#define SPI_CR1_SPE               (1 << 0)   /* Bit 0: SPI Enable */
                                             /* Bits 1-7: Reserved */
#define SPI_CR1_MASRX             (1 << 8)   /* Bit 8: */
#define SPI_CR1_CSTART            (1 << 9)   /* Bit 9: master transfer start */
#define SPI_CR1_CSUSP             (1 << 10)  /* Bit 10: master suspend request */
#define SPI_CR1_HDDIR             (1 << 11)  /* Bit 11: RX/TX direction at Half-duplex mode */
#define SPI_CR1_SSI               (1 << 12)  /* Bit 12: Internal slave select */
#define SPI_CR1_CRC33_17          (1 << 13)  /* Bit 13: 32-bit CRC polynominal configuration */
#define SPI_CR1_RCRCINI           (1 << 14)  /* Bit 14: CRC calculation initialization pattern control for receiver */
#define SPI_CR1_TCRCINI           (1 << 15)  /* Bit 15: CRC calculation initialization pattern control for transmitter */
#define SPI_CR1_IOLOCK            (1 << 16)  /* Bit 16: locking the AF configuration of associated IOs */
                                             /* Bits 17-31: Reserved */

/* SPI Control Register 2 */

#define SPI_CR2_TSIZE_SHIFT       (0)         /* Bits 0-15 */
#define SPI_CR2_TSIZE_MASK        (0xff << SPI_CR2_TSIZE_SHIFT)
#define SPI_CR2_TSER_SHIFT        (16)        /* Bits 16-31 */
#define SPI_CR2_TSER_MASK         (0xff << SPI_CR2_TSER_SHIFT)

/* SPI configuration register 1 */

#define SPI_CFG1_DSIZE_SHIFT      (0) /* Bits 0-4: number of bits in at single SPI data frame */
#define SPI_CFG1_DSIZE_VAL(n)     ((n-1) << SPI_CFG1_DSIZE_SHIFT)
#define SPI_CFG1_DSIZE_MASK       (0x1f << SPI_CFG1_DSIZE_SHIFT)

                                      /* 00000 - 00010 - not used */
#  define SPI_CFG1_DSIZE_4BIT     (3 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_5BIT     (4 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_6BIT     (5 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_7BIT     (6 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_8BIT     (7 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_9BIT     (8 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_10BIT    (9 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_11BIT    (10 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_12BIT    (11 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_13BIT    (12 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_14BIT    (13 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_15BIT    (14 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_16BIT    (15 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_17BIT    (16 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_18BIT    (17 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_19BIT    (18 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_20BIT    (19 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_21BIT    (20 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_22BIT    (21 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_23BIT    (22 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_24BIT    (23 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_25BIT    (24 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_26BIT    (25 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_27BIT    (26 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_28BIT    (27 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_29BIT    (28 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_30BIT    (29 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_31BIT    (30 << SPI_CFG1_DSIZE_SHIFT)
#  define SPI_CFG1_DSIZE_32BIT    (31 << SPI_CFG1_DSIZE_SHIFT)
#define SPI_CFG1_FTHLV_SHIFT      (5) /* Bits 5-8: FIFO threshold level */
#define SPI_CFG1_FTHLV_MASK       (0xf << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_1DATA    (0 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_2DATA    (1 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_3DATA    (2 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_4DATA    (3 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_5DATA    (4 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_6DATA    (5 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_7DATA    (6 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_8DATA    (7 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_9DATA    (8 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_10DATA   (9 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_11DATA   (10 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_12DATA   (11 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_13DATA   (12 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_14DATA   (13 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_15DATA   (14 << SPI_CFG1_FTHLV_SHIFT)
#  define SPI_CFG1_FTHLV_16DATA   (15 << SPI_CFG1_FTHLV_SHIFT)
#define SPI_CFG1_UDRCFG_SHIFT     (9) /* Bits 9-10: behavior of slave transmitter at underrun condition */
#define SPI_CFG1_UDRCFG_MASK      (0x3 << SPI_CFG1_UDRCFG_SHIFT)
#  define SPI_CFG1_UDRCFG_CONST   (0 << SPI_CFG1_UDRCFG_SHIFT)
#  define SPI_CFG1_UDRCFG_LASTRX  (1 << SPI_CFG1_UDRCFG_SHIFT)
#  define SPI_CFG1_UDRCFG_LASTTX  (2 << SPI_CFG1_UDRCFG_SHIFT)

                                       /* 11: Reserved */
#define SPI_CFG1_UDRDET_SHIFT     (11) /* Bits 11-12: detection of underrun condition at slave transmitter */
#define SPI_CFG1_UDRDET_MASK      (0x3 << SPI_CFG1_UDRDET_SHIFT)
#  define SPI_CFG1_UDRDET_BEG     (0 << SPI_CFG1_UDRDET_SHIFT)
#  define SPI_CFG1_UDRDET_END     (1 << SPI_CFG1_UDRDET_SHIFT)
#  define SPI_CFG1_UDRDET_SS      (2 << SPI_CFG1_UDRDET_SHIFT)

                                            /* 11: Reserved */

                                            /* Bit 13: Reserved */
#define SPI_CFG1_RXDMAEN          (1 << 14) /* Bit 14: RX-DMA stream enable */
#define SPI_CFG1_TXDMAEN          (1 << 15) /* Bit 15: TX-DMA stream enable */
#define SPI_CFG1_CRCSIZE_SHIFT    (16)      /* Bits 16-20: length of CRC frame to be transacted and compared */
#define SPI_CFG1_CRCSIZE_VAL(n)   ((n-1) << SPI_CFG1_CRCSIZE_SHIFT)
#define SPI_CFG1_CRCSIZE_MASK     (0x1f << SPI_CFG1_CRCSIZE_SHIFT)

                                       /* 00000-00010: Reserved */
#  define SPI_CFG1_CRCSIZE_4BIT   (3 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_5BIT   (4 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_6BIT   (5 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_7BIT   (6 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_8BIT   (7 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_9BIT   (8 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_10BIT  (9 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_11BIT  (10 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_12BIT  (11 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_13BIT  (12 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_14BIT  (13 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_15BIT  (14 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_16BIT  (15 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_17BIT  (16 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_18BIT  (17 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_19BIT  (18 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_20BIT  (19 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_21BIT  (20 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_22BIT  (21 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_23BIT  (22 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_24BIT  (23 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_25BIT  (24 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_26BIT  (25 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_27BIT  (26 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_28BIT  (27 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_29BIT  (28 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_30BIT  (29 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_31BIT  (30 << SPI_CFG1_CRCSIZE_SHIFT)
#  define SPI_CFG1_CRCSIZE_32BIT  (31 << SPI_CFG1_CRCSIZE_SHIFT)

                                            /* Bit 21: Reserved */
#define SPI_CFG1_CRCEN            (1 << 22) /* Bit 22: hardware CRC computation enable */

                                            /* Bits 23-27: Reserved */
#define SPI_CFG1_MBR_SHIFT        (28)      /* Bits 28-30: master baud rate */
#define SPI_CFG1_MBR_MASK         (0x7 << SPI_CFG1_MBR_SHIFT)
#  define SPI_CFG1_MBR_FPCLKd2    (0 << SPI_CFG1_MBR_SHIFT)
#  define SPI_CFG1_MBR_FPCLKd4    (1 << SPI_CFG1_MBR_SHIFT)
#  define SPI_CFG1_MBR_FPCLKd8    (2 << SPI_CFG1_MBR_SHIFT)
#  define SPI_CFG1_MBR_FPCLKd16   (3 << SPI_CFG1_MBR_SHIFT)
#  define SPI_CFG1_MBR_FPCLKd32   (4 << SPI_CFG1_MBR_SHIFT)
#  define SPI_CFG1_MBR_FPCLKd64   (5 << SPI_CFG1_MBR_SHIFT)
#  define SPI_CFG1_MBR_FPCLKd128  (6 << SPI_CFG1_MBR_SHIFT)
#  define SPI_CFG1_MBR_FPCLKd256  (7 << SPI_CFG1_MBR_SHIFT)

                                       /* Bit 31: Reserved */

/* SPI configuration register 2 */

#define SPI_CFG2_MSSI_SHIFT       (0)  /* Bits 0-3:master SS idleness */
#define SPI_CFG2_MSSI_MASK        (0xf << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_0CLK      (0 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_1CLK      (1 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_2CLK      (2 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_3CLK      (3 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_4CLK      (4 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_5CLK      (5 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_6CLK      (6 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_7CLK      (7 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_8CLK      (8 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_9CLK      (9 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_10CLK     (10 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_11CLK     (11 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_12CLK     (12 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_13CLK     (13 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_14CLK     (14 << SPI_CFG2_MSSI_SHIFT)
#  define SPI_CFG2_MSSI_15CLK     (15 << SPI_CFG2_MSSI_SHIFT)
#define SPI_CFG2_MIDI_SHIFT       (4)  /* Bits 4-7: master Inter-Data idleness */
#define SPI_CFG2_MIDI_MASK        (0xf << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_0CLK      (0 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_1CLK      (1 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_2CLK      (2 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_3CLK      (3 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_4CLK      (4 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_5CLK      (5 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_6CLK      (6 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_7CLK      (7 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_8CLK      (8 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_9CLK      (9 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_10CLK     (10 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_11CLK     (11 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_12CLK     (12 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_13CLK     (13 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_14CLK     (14 << SPI_CFG2_MIDI_SHIFT)
#  define SPI_CFG2_MIDI_15CLK     (15 << SPI_CFG2_MIDI_SHIFT)

                                            /* Bits 8-14: Reserved */
#define SPI_CFG2_IOSWP            (1 << 15) /* Bit 15: swap functionality of MISO and MOSI pins */

                                            /* Bit 16: Reserved */
#define SPI_CFG2_COMM_SHIFT       (17)      /* Bits 17-18: SPI communication mode */
#define SPI_CFG2_COMM_MASK        (0x3 << SPI_CFG2_COMM_SHIFT)
#  define SPI_CFG2_COMM_FULL      (0 << SPI_CFG2_COMM_SHIFT)
#  define SPI_CFG2_COMM_STX       (1 << SPI_CFG2_COMM_SHIFT)
#  define SPI_CFG2_COMM_SRX       (2 << SPI_CFG2_COMM_SHIFT)
#  define SPI_CFG2_COMM_HALF      (3 << SPI_CFG2_COMM_SHIFT)
#define SPI_CFG2_SP_SHIFT         (19) /* Bits 19-21: serial protocol */
#define SPI_CFG2_SP_MASK          (0x7 << SPI_CFG2_SP_SHIFT)
#  define SPI_CFG2_SP_MOTOROLA    (0 << SPI_CFG2_SP_SHIFT)
#  define SPI_CFG2_SP_TI          (1 << SPI_CFG2_SP_SHIFT)

                                            /* 010-111: Reserved */
#define SPI_CFG2_MASTER           (1 << 22) /* Bit 22: SPI master */
#define SPI_CFG2_LSBFRST          (1 << 23) /* Bit 23: data frame format */
#define SPI_CFG2_CPHA             (1 << 24) /* Bit 24: clock phase */
#define SPI_CFG2_CPOL             (1 << 25) /* Bit 25: clock polarity */
#define SPI_CFG2_SSM              (1 << 26) /* Bit 26: software management of SS signal input */
                                            /* Bit 27: Reserved */
#define SPI_CFG2_SSIOP            (1 << 28) /* Bit 28: SS input/output polarity */
#define SPI_CFG2_SSOE             (1 << 29) /* Bit 29: SS output enable */
#define SPI_CFG2_SSOM             (1 << 30) /* Bit 30: SS output management in master mode */
#define SPI_CFG2_AFCNTR           (1 << 31) /* Bit 31: alternate function GPIOs control */

/* SPI/I2S status register */

#define SPI_SR_RXP               (1 << 0)  /* Bit 0: Rx-packet available */
#define SPI_SR_TXP               (1 << 1)  /* Bit 1: Tx-packet space available */
#define SPI_SR_DXP               (1 << 2)  /* Bit 2: duplex packet */
#define SPI_SR_EOT               (1 << 3)  /* Bit 3: end of transfer */
#define SPI_SR_TXTF              (1 << 4)  /* Bit 4: transmission transfer filled */
#define SPI_SR_UDR               (1 << 5)  /* Bit 5: underrun at slave transmission mode */
#define SPI_SR_OVR               (1 << 6)  /* Bit 6: overrun */
#define SPI_SR_CRCE              (1 << 7)  /* Bit 7: CRC error */
#define SPI_SR_TIFRE             (1 << 8)  /* Bit 8: TI frame format error */
#define SPI_SR_MODF              (1 << 9)  /* Bit 9: mode fault */
#define SPI_SR_TSERF             (1 << 10) /* Bit 10: additional number of SPI data to be transacted was reload */
#define SPI_SR_SUSP              (1 << 11) /* Bit 11: suspend */
#define SPI_SR_TXC               (1 << 12) /* Bit 12: TxFIFO transmission complete */
#define SPI_SR_RXPLVL_SHIFT      (13)      /* Bits 13-14: RxFIFO packing level */
#define SPI_SR_RXPLVL_MASK       (1 << SPI_SR_RXPLVL_SHIFT)
#define SPI_SR_RXWNE             (1 << 15) /* Bit 15: RxFIFO word not empty */
#define SPI_SR_CTSIZE_SHIFT      (16)      /* Bits 16-31: number of data frames remaining in current TSIZE session */
#define SPI_SR_CTSIZE_MASK       (1 << SPI_SR_CTSIZE_SHIFT)

/* SPI/I2S interrupt/status flags interrupt enable register */

#define SPI_IER_RXPIE            (1 << 0)  /* Bit 0: RXP Interrupt enable */
#define SPI_IER_TXPIE            (1 << 1)  /* Bit 1: TXP interrupt enable */
#define SPI_IER_DXPIE            (1 << 2)  /* Bit 2: DXP interrupt enable */
#define SPI_IER_EOTIE            (1 << 3)  /* Bit 3: EOT, SUSP and TXC interrupt enable */
#define SPI_IER_TXTFIE           (1 << 4)  /* Bit 4: TXTFIE interrupt enable */
#define SPI_IER_UDRIE            (1 << 5)  /* Bit 5: UDR interrupt enable */
#define SPI_IER_OVRIE            (1 << 6)  /* Bit 6: OVR interrupt enable */
#define SPI_IER_CRCEIE           (1 << 7)  /* Bit 7: CRC error interrupt enable */
#define SPI_IER_TIFREIE          (1 << 8)  /* Bit 8: TIFRE interrupt enable */
#define SPI_IER_MODFIE           (1 << 9)  /* Bit 9: mode fault interrupt enable */
#define SPI_IER_TSERFIE          (1 << 10) /* Bit 10: additional number of transactions
                                            * reload interrupt enable */

/* SPI/I2S interrupt/status flags clear register */

                                           /* Bits 0-2: Reserved */
#define SPI_IFCR_EOTC            (1 << 3)  /* Bit 3: end of transfer flag clear */
#define SPI_IFCR_TXTFC           (1 << 4)  /* Bit 4: transmission Transfer Flilled flag clear */
#define SPI_IFCR_UDRC            (1 << 5)  /* Bit 5: underrun flag clear */
#define SPI_IFCR_OVRC            (1 << 6)  /* Bit 6: overrun flag clear */
#define SPI_IFCR_CRCEC           (1 << 7)  /* Bit 7: CRC error flag clear */
#define SPI_IFCR_TIFREC          (1 << 8)  /* Bit 8: TI frame format error flag clear */
#define SPI_IFCR_MODFC           (1 << 9)  /* Bit 9: mode fault flag clear */
#define SPI_IFCR_TSERFC          (1 << 10) /* Bit 10: TSERF flag clear*/
#define SPI_IFCR_SUSPC           (1 << 11) /* Bit 11: suspend flag clear */
                                           /* Bits 12-31: Reserved */

/* SPI/I2S transmit data register */

#define SPI_TXDR_TXDR_SHIFT      (0) /* Bits 0-15: transmit data register */
#define SPI_TXDR_TXDR_MASK       (0xffff << SPI_TXDR_TXDR_SHIFT)
                                     /* Bits 16-31: write ignored */

/* SPI/I2S receive data register */

#define SPI_RXDR_RXDR_SHIFT      (0) /* Bits 0-15: receive data register */
#define SPI_RXDR_RXDR_MASK       (0xffff << SPI_RXDR_RXDR_SHIFT)
                                     /* Bits 16-31: read zero */

/* SPI/I2S SPI polynominal register */

#define SPI_CRCPOLY_CRCPOLY_SHIFT      (0) /* Bits 0-15: CRC polynominal register */
#define SPI_CRCPOLY_CRCPOLY_MASK       (0xffff << SPI_CRCPOLY_CRCPOLY_SHIFT)
                                           /* Bits 16-31: write ignored */

/* SPI/I2S SPI transmitter CRC register */

#define SPI_TXCRC_TXCRC_SHIFT      (0) /* Bits 0-15: CRC register for transmitter */
#define SPI_TXCRC_TXCRC_MASK       (0xffff << SPI_TXCRC_TXCRC_SHIFT)
                                       /* Bits 16-31: write ignored */

/* SPI/I2S SPI receiver CRC register */

#define SPI_RXCRC_RXCRC_SHIFT      (0) /* Bits 0-15: CRC register for receiver */
#define SPI_RXCRC_RXCRC_MASK       (0xffff << SPI_RXCRC_RXCRC_SHIFT)
                                       /* Bits 16-31: read zero */

/* SPI/I2S SPI underrun data register */

#define SPI_UDRDR_UDRDR_SHIFT      (0) /* Bits 0-15: data at slave underrun condition*/
#define SPI_UDRDR_UDRDR_MASK       (0xffff << SPI_UDRDR_UDRDR_SHIFT)
                                       /* Bits 16-31: read zero */

#endif /* CONFIG_STM32U5_STM32U585XX */
#endif /* __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32U585XX_SPI_H */
