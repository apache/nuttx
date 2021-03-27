/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_spi.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SPI_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC43_SPI_CR_OFFSET  0x0000  /* Control Register */
#define LPC43_SPI_SR_OFFSET  0x0004  /* SPI Status Register */
#define LPC43_SPI_DR_OFFSET  0x0008  /* SPI Data Register */
#define LPC43_SPI_CCR_OFFSET 0x000c  /* SPI Clock Counter Register */
#define LPC43_SPI_TCR_OFFSET 0x0010  /* SPI Test Control Register */
#define LPC43_SPI_TSR_OFFSET 0x0014  /* SPI Test Status Register */
#define LPC43_SPI_INT_OFFSET 0x001c  /* SPI Interrupt Register */

/* Register addresses *******************************************************/

#define LPC43_SPI_CR         (LPC43_SPI_BASE+LPC43_SPI_CR_OFFSET)
#define LPC43_SPI_SR         (LPC43_SPI_BASE+LPC43_SPI_SR_OFFSET)
#define LPC43_SPI_DR         (LPC43_SPI_BASE+LPC43_SPI_DR_OFFSET)
#define LPC43_SPI_CCR        (LPC43_SPI_BASE+LPC43_SPI_CCR_OFFSET)
#define LPC43_TCR_CCR        (LPC43_SPI_BASE+LPC43_SPI_TCR_OFFSET)
#define LPC43_TSR_CCR        (LPC43_SPI_BASE+LPC43_SPI_TSR_OFFSET)
#define LPC43_SPI_INT        (LPC43_SPI_BASE+LPC43_SPI_INT_OFFSET)

/* Register bit definitions *************************************************/

/* Control Register */

                                       /* Bits 0-1: Reserved */
#define SPI_CR_BITENABLE     (1 << 2)  /* Bit 2:  Enable word size selected by BITS */
#define SPI_CR_CPHA          (1 << 3)  /* Bit 3:  Clock phase control */
#define SPI_CR_CPOL          (1 << 4)  /* Bit 4:  Clock polarity control */
#define SPI_CR_MSTR          (1 << 5)  /* Bit 5:  Master mode select */
#define SPI_CR_LSBF          (1 << 6)  /* Bit 6:  SPI data is transferred LSB first */
#define SPI_CR_SPIE          (1 << 7)  /* Bit 7:  Serial peripheral interrupt enable */
#define SPI_CR_BITS_SHIFT    (8)       /* Bits 8-11: Number of bits per word (BITENABLE==1) */
#define SPI_CR_BITS_MASK     (15 << SPI_CR_BITS_SHIFT)
#  define SPI_CR_BITS_8BITS  (8 <<  SPI_CR_BITS_SHIFT) /* 8 bits per transfer */
#  define SPI_CR_BITS_9BITS  (9 <<  SPI_CR_BITS_SHIFT) /* 9 bits per transfer */
#  define SPI_CR_BITS_10BITS (10 << SPI_CR_BITS_SHIFT) /* 10 bits per transfer */
#  define SPI_CR_BITS_11BITS (11 << SPI_CR_BITS_SHIFT) /* 11 bits per transfer */
#  define SPI_CR_BITS_12BITS (12 << SPI_CR_BITS_SHIFT) /* 12 bits per transfer */
#  define SPI_CR_BITS_13BITS (13 << SPI_CR_BITS_SHIFT) /* 13 bits per transfer */
#  define SPI_CR_BITS_14BITS (14 << SPI_CR_BITS_SHIFT) /* 14 bits per transfer */
#  define SPI_CR_BITS_15BITS (15 << SPI_CR_BITS_SHIFT) /* 15 bits per transfer */
#  define SPI_CR_BITS_16BITS (0 <<  SPI_CR_BITS_SHIFT) /* 16 bits per transfer */

                                       /* Bits 12-31: Reserved */

/* SPI Status Register */

                                       /* Bits 0-2: Reserved */
#define SPI_SR_ABRT          (1 << 3)  /* Bit 3:  Slave abort */
#define SPI_SR_MODF          (1 << 4)  /* Bit 4:  Mode fault */
#define SPI_SR_ROVR          (1 << 5)  /* Bit 5:  Read overrun */
#define SPI_SR_WCOL          (1 << 6)  /* Bit 6:  Write collision */
#define SPI_SR_SPIF          (1 << 7)  /* Bit 7:  SPI transfer complete */
                                       /* Bits 8-31: Reserved */

/* SPI Data Register */

#define SPI_DR_MASK          (0xff)    /* Bits 0-15: SPI Bi-directional data port */
#define SPI_DR_MASKWIDE      (0xffff)  /* Bits 0-15: If SPI_CR_BITENABLE != 0 */
                                       /* Bits 8-31: Reserved */

/* SPI Clock Counter Register */

#define SPI_CCR_MASK         (0xff)    /* Bits 0-7: SPI Clock counter setting */
                                       /* Bits 8-31: Reserved */

/* SPI Test Control Register */

                                       /* Bit 0: Reserved */
#define SPI_TCR_TEST_SHIFT   (1)       /* Bits 1-7: SPI test mode */
#define SPI_TCR_TEST_MASK    (0x7f << SPI_TCR_TEST_SHIFT)
                                       /* Bits 8-31: Reserved */

/* SPI Test Status Register */

                                       /* Bits 0-2: Reserved */
#define SPI_TSR_ABRT         (1 << 3)  /* Bit 3:  Slave abort */
#define SPI_TSR_MODF         (1 << 4)  /* Bit 4:  Mode fault */
#define SPI_TSR_ROVR         (1 << 5)  /* Bit 5:  Read overrun */
#define SPI_TSR_WCOL         (1 << 6)  /* Bit 6:  Write collision */
#define SPI_TSR_SPIF         (1 << 7)  /* Bit 7:  SPI transfer complete */
                                       /* Bits 8-31: Reserved */

/* SPI Interrupt Register */

#define SPI_INT_SPIF         (1 << 0)  /* SPI interrupt */
                                       /* Bits 1-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SPI_H */
