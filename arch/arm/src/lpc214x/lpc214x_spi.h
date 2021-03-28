/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_spi.h
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

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_SPI_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register address definitions *********************************************/

/* SPI absolute register addresses */

#define LPC214X_SPI0_CR   (LPC214X_SPI0_BASE+LPC214X_SPI0_CR_OFFSET)   /* 16-bits wide */
#define LPC214X_SPI0_SR   (LPC214X_SPI0_BASE+LPC214X_SPI0_SR_OFFSET)   /*  8-bits wide */
#define LPC214X_SPI0_DR   (LPC214X_SPI0_BASE+LPC214X_SPI0_DR_OFFSET)   /* 16-bits wide */
#define LPC214X_SPI0_CCR  (LPC214X_SPI0_BASE+LPC214X_SPI0_CCR_OFFSET)  /*  8-bits wide */
#define LPC214X_SPI0_INT  (LPC214X_SPI0_BASE+LPC214X_SPI0_INT_OFFSET)  /*  8-bits wide */

#define LPC214X_SPI1_CR0  (LPC214X_SPI1_BASE+LPC214X_SPI1_CR0_OFFSET)  /* 16-bits wide */
#define LPC214X_SPI1_CR1  (LPC214X_SPI1_BASE+LPC214X_SPI1_CR1_OFFSET)  /*  8-bits wide */
#define LPC214X_SPI1_DR   (LPC214X_SPI1_BASE+LPC214X_SPI1_DR_OFFSET)   /* 16-bits wide */
#define LPC214X_SPI1_SR   (LPC214X_SPI1_BASE+LPC214X_SPI1_SR_OFFSET)   /*  8-bits wide */
#define LPC214X_SPI1_CPSR (LPC214X_SPI1_BASE+LPC214X_SPI1_IMSC_OFFSET) /*  8-bits wide */
#define LPC214X_SPI1_IMSC (LPC214X_SPI1_BASE+LPC214X_SPI1_IMSC_OFFSET) /*  8-bits wide */
#define LPC214X_SPI1_RIS  (LPC214X_SPI1_BASE+LPC214X_SPI1_RIS_OFFSET)  /*  8-bits wide */
#define LPC214X_SPI1_MIS  (LPC214X_SPI1_BASE+LPC214X_SPI1_ICR_OFFSET)  /*  8-bits wide */
#define LPC214X_SPI1_ICR  (LPC214X_SPI1_BASE+LPC214X_SPI1_ICR_OFFSET)  /*  8-bits wide */

/* SPI0 register bit definitions ********************************************/

/* Control Register (CR) for SPI0 */

#define LPC214X_SPI0CR0_BITSENB  (0x0004) /* Bit 2=0: 8-bits, else see bits 8-11 */
#define LPC214X_SPI0CR0_CPHA     (0x0008) /* Bit 3: Clock phase control */
#define LPC214X_SPI0CR0_CPOL     (0x0010) /* Bit 4: Clock polarity control */
#define LPC214X_SPI0CR0_MSTR     (0x0020) /* Bit 5=1: Master 0: Slave */
#define LPC214X_SPI0CR0_LSBF     (0x0040) /* Bit 6=1: Shift LSB first */
#define LPC214X_SPI0CR0_SPIE     (0x0080) /* Bit 7=1: SPI interrupt enable */
#define LPC214X_SPI0CR0_BITSMASK (0x0f00) /* Bits 8-11: Number of bits per transfer */
#define LPC214X_SPI0CR0_BITS8    (0x0800) /*   8-bits per transfer */
#define LPC214X_SPI0CR0_BITS9    (0x0900) /*   9-bits per transfer */
#define LPC214X_SPI0CR0_BITS10   (0x0a00) /*  10-bits per transfer */
#define LPC214X_SPI0CR0_BITS11   (0x0b00) /*  11-bits per transfer */
#define LPC214X_SPI0CR0_BITS12   (0x0c00) /*  12-bits per transfer */
#define LPC214X_SPI0CR0_BITS13   (0x0d00) /*  13-bits per transfer */
#define LPC214X_SPI0CR0_BITS14   (0x0e00) /*  14-bits per transfer */
#define LPC214X_SPI0CR0_BITS15   (0x0f00) /*  15-bits per transfer */
#define LPC214X_SPI0CR0_BITS16   (0x0000) /*  16-bits per transfer */

/* Status Register (SR) for SPI0 */

#define LPC214X_SPI0SR_ABRT      (0x08)   /* Bit 3=1: Slave abort */
#define LPC214X_SPI0SR_MODF      (0x10)   /* Bit 4=1: Mode fault */
#define LPC214X_SPI0SR_ROVR      (0x20)   /* Bit 5=1: Read overrun */
#define LPC214X_SPI0SR_WCOL      (0x40)   /* Bit 6=1: Write collision */
#define LPC214X_SPI0SR_SPIF      (0x80)   /* Bit 7=1: SPI transfer complete */

/* Interrupt Register for SPI0 */

#define LPC214X_SPO0INT_SPI      (0x01)   /* Bit 0=1: SPI interrupt */

/* SPI1 register bit definitions ********************************************/

/* Control Register 0 (CR0) for SPI1 */

#define LPC214X_SPI1CR0_DSSMASK  (0x000f) /* Bits 0-3: Data size select mask */
#define LPC214X_SPI1CR0_DSS4BIT  (0x0003) /*  4-bit transfer */
#define LPC214X_SPI1CR0_DSS5BIT  (0x0004) /*  5-bit transfer */
#define LPC214X_SPI1CR0_DSS6BIT  (0x0005) /*  6-bit transfer */
#define LPC214X_SPI1CR0_DSS7BIT  (0x0006) /*  7-bit transfer */
#define LPC214X_SPI1CR0_DSS8BIT  (0x0007) /*  8-bit transfer */
#define LPC214X_SPI1CR0_DSS9BIT  (0x0008) /*  9-bit transfer */
#define LPC214X_SPI1CR0_DSS10BIT (0x0009) /*  10-bit transfer */
#define LPC214X_SPI1CR0_DSS11BIT (0x000a) /*  11-bit transfer */
#define LPC214X_SPI1CR0_DSS12BIT (0x000b) /*  12-bit transfer */
#define LPC214X_SPI1CR0_DSS13BIT (0x000c) /*  13-bit transfer */
#define LPC214X_SPI1CR0_DSS14BIT (0x000d) /*  14-bit transfer */
#define LPC214X_SPI1CR0_DSS15BIT (0x000e) /*  15-bit transfer */
#define LPC214X_SPI1CR0_DSS16BIT (0x000f) /*  16-bit transfer */
#define LPC214X_SPI1CR0_FRFMASK  (0x0030) /* Bits 4-5: Frame format mask */
#define LPC214X_SPI1CR0_FRFSPI   (0x0000) /*   SPI */
#define LPC214X_SPI1CR0_FRFSSI   (0x0010) /*   SSI */
#define LPC214X_SPI1CR0_FRFMW    (0x0020) /*   Microwire */
#define LPC214X_SPI1CR0_CPOL     (0x0040) /* Bit 6: Clock polarity control */
#define LPC214X_SPI1CR0_CPHA     (0x0080) /* Bit 7: Clock phase control */
#define LPC214X_SPI1CR0_SCR      (0xff00) /* Bits 8-15: Serial clock reate */

/* Control Register 1 (CR1) */

#define LPC214X_SPI1CR1_LBM      (0x01) /* Bit 0: 1=Loopback mode */
#define LPC214X_SPI1CR1_SSE      (0x02) /* Bit 1: 1=SSP enable */
#define LPC214X_SPI1CR1_MS       (0x04) /* Bit 2: 1=Controller is slave */
#define LPC214X_SPI1CR1_SOD      (0x08) /* Bit 3: 1=Slave output disable */

/* SSP Status Register (SR) */

#define LPC214X_SPI1SR_TFE       (0x01) /* Bit 0: 1=Transmit FIFO Empty */
#define LPC214X_SPI1SR_TNF       (0x02) /* Bit 1: 1=Transmit FIFO not full */
#define LPC214X_SPI1SR_RNE       (0x04) /* Bit 2: 1=Receive FIFO not empty */
#define LPC214X_SPI1SR_RFF       (0x08) /* Bit 3: 1=Receive FIFO full */
#define LPC214X_SPI1SR_BSY       (0x10) /* Bit 4: 1=Busy */

/* Interrupt set/clear/status/mask registers (can't clear RXIM or TXIM) */

#define LPC214X_SP1INT_ROR       (0x01) /* Bit 0: 1=Receive Overrun */
#define LPC214X_SP1INT_RTIM      (0x02) /* Bit 1: 1=Receive Timeout */
#define LPC214X_SP1INT_RXIM      (0x04) /* Bit 2: 1=RX FIFO at least half full */
#define LPC214X_SP1INT_TXIM      (0x08) /* Bit 3: 1=TX FIFO at least half empty */

/* SPI1 supports an 8-frame FIFO */

#define LPC214X_SPI1_FIFOSZ      (8)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct spi_dev_s; /* Forward reference */

/****************************************************************************
 * Name: lpc214x_spibus_initialize
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

FAR struct spi_dev_s *lpc214x_spibus_initialize(int port);

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_SPI_H */
