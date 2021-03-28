/****************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_spi.h
 *
 *   Copyright (C) 2013 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *
 * Derived arch/arm/src/lpc17xx_40xx/lpc17_40_spi.h
 *
 *   Copyright (C) 2010, 2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC2378_LPC23XX_SPI_H
#define __ARCH_ARM_SRC_LPC2378_LPC23XX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI Pin Configuration ****************************************************/

#define SPI_BASE             SPI0_BASE_ADDR

#define SPI_PCLKSEL_MASK     (0x03 << 16)
#define SPI_PCLKSEL          (0x01 << 16)

#define SPI_PINSEL0_MASK     (0x03 << 30)   /* P0.15 SCK    PINSEL0 */
#define SPI_PINSEL1_MASK     (0x0f << 2)    /* P0.17 P0.18  PINSEL1 */
#define SPI_PINSEL0          (0x03 << 30)
#define SPI_PINSEL1          (0x0f << 2)

/* SPI PRegister offsets ****************************************************/

#define SPI_CR_OFFSET        0x0000  /* Control Register */
#define SPI_SR_OFFSET        0x0004  /* SPI Status Register */
#define SPI_DR_OFFSET        0x0008  /* SPI Data Register */
#define SPI_CCR_OFFSET       0x000c  /* SPI Clock Counter Register */
#define SPI_TCR_OFFSET       0x0010  /* SPI Test Control Register */
#define SPI_TSR_OFFSET       0x0014  /* SPI Test Status Register */
#define SPI_INT_OFFSET       0x001c  /* SPI Interrupt Register */

/* SPI Register addresses ***************************************************/

#define SPI_CR               (SPI_BASE+SPI_CR_OFFSET)
#define SPI_SR               (SPI_BASE+SPI_SR_OFFSET)
#define SPI_DR               (SPI_BASE+SPI_DR_OFFSET)
#define SPI_CCR              (SPI_BASE+SPI_CCR_OFFSET)
#define TCR_CCR              (SPI_BASE+SPI_TCR_OFFSET)
#define TSR_CCR              (SPI_BASE+SPI_TSR_OFFSET)
#define SPI_INT              (SPI_BASE+SPI_INT_OFFSET)

/* SPI Register bit definitions *********************************************/

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

struct spi_dev_s;  /* Forward reference */

/****************************************************************************
 * Name: lpc23_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *lpc23_spibus_initialize(int port);

/****************************************************************************
 * These functions must be provided by the board specific logic that has
 * knowledge the chip select and card detect GPIO configuration of the board.
 *
 ****************************************************************************/

void  lpc23xx_spiselect(FAR struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc23xx_spistatus(FAR struct spi_dev_s *dev, uint32_t devid);

#endif /* __ARCH_ARM_SRC_LPC2378_LPC23XX_SPI_H */
