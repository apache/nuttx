/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_spi.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_SPI_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_SPI_SSPCR0_OFFSET        0x000000  /* Control register 0 */
#define RP23XX_SPI_SSPCR1_OFFSET        0x000004  /* Control register 1 */
#define RP23XX_SPI_SSPDR_OFFSET         0x000008  /* Data register */
#define RP23XX_SPI_SSPSR_OFFSET         0x00000c  /* Status register */
#define RP23XX_SPI_SSPCPSR_OFFSET       0x000010  /* Clock prescale register */
#define RP23XX_SPI_SSPIMSC_OFFSET       0x000014  /* Interrupt mask set or clear register */
#define RP23XX_SPI_SSPRIS_OFFSET        0x000018  /* Raw interrupt status register */
#define RP23XX_SPI_SSPMIS_OFFSET        0x00001c  /* Masked interrupt status register */
#define RP23XX_SPI_SSPICR_OFFSET        0x000020  /* Interrupt clear register */
#define RP23XX_SPI_SSPDMACR_OFFSET      0x000024  /* DMA control register */
#define RP23XX_SPI_SSPPERIPHID0_OFFSET  0x000fe0  /* Peripheral identification registers */
#define RP23XX_SPI_SSPPERIPHID1_OFFSET  0x000fe4  /* Peripheral identification registers */
#define RP23XX_SPI_SSPPERIPHID2_OFFSET  0x000fe8  /* Peripheral identification registers */
#define RP23XX_SPI_SSPPERIPHID3_OFFSET  0x000fec  /* Peripheral identification registers */
#define RP23XX_SPI_SSPPCELLID0_OFFSET   0x000ff0  /* PrimeCell identification registers */
#define RP23XX_SPI_SSPPCELLID1_OFFSET   0x000ff4  /* PrimeCell identification registers */
#define RP23XX_SPI_SSPPCELLID2_OFFSET   0x000ff8  /* PrimeCell identification registers */
#define RP23XX_SPI_SSPPCELLID3_OFFSET   0x000ffc  /* PrimeCell identification registers */

/* Register definitions *****************************************************/

#define RP23XX_SPI_SSPCR0(n)        (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPCR0_OFFSET)
#define RP23XX_SPI_SSPCR1(n)        (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPCR1_OFFSET)
#define RP23XX_SPI_SSPDR(n)         (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPDR_OFFSET)
#define RP23XX_SPI_SSPSR(n)         (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPSR_OFFSET)
#define RP23XX_SPI_SSPCPSR(n)       (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPCPSR_OFFSET)
#define RP23XX_SPI_SSPIMSC(n)       (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPIMSC_OFFSET)
#define RP23XX_SPI_SSPRIS(n)        (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPRIS_OFFSET)
#define RP23XX_SPI_SSPMIS(n)        (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPMIS_OFFSET)
#define RP23XX_SPI_SSPICR(n)        (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPICR_OFFSET)
#define RP23XX_SPI_SSPDMACR(n)      (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPDMACR_OFFSET)
#define RP23XX_SPI_SSPPERIPHID0(n)  (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPPERIPHID0_OFFSET)
#define RP23XX_SPI_SSPPERIPHID1(n)  (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPPERIPHID1_OFFSET)
#define RP23XX_SPI_SSPPERIPHID2(n)  (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPPERIPHID2_OFFSET)
#define RP23XX_SPI_SSPPERIPHID3(n)  (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPPERIPHID3_OFFSET)
#define RP23XX_SPI_SSPPCELLID0(n)   (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPPCELLID0_OFFSET)
#define RP23XX_SPI_SSPPCELLID1(n)   (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPPCELLID1_OFFSET)
#define RP23XX_SPI_SSPPCELLID2(n)   (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPPCELLID2_OFFSET)
#define RP23XX_SPI_SSPPCELLID3(n)   (RP23XX_SPI_BASE(n) + RP23XX_SPI_SSPPCELLID3_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_SPI_SSPCR0_SCR_SHIFT                 (8)       /* Serial clock rate */
#define RP23XX_SPI_SSPCR0_SCR_MASK                  (0xff << RP23XX_SPI_SSPCR0_SCR_SHIFT)
#define RP23XX_SPI_SSPCR0_SPH                       (1 << 7)  /* SSPCLKOUT phase */
#define RP23XX_SPI_SSPCR0_SPO                       (1 << 6)  /* SSPCLKOUT polarity */
#define RP23XX_SPI_SSPCR0_FRF_SHIFT                 (4)       /* Frame format */
#define RP23XX_SPI_SSPCR0_FRF_MASK                  (0x03 << RP23XX_SPI_SSPCR0_FRF_SHIFT)
#define RP23XX_SPI_SSPCR0_DSS_MASK                  (0x0f)    /* Data Size Select */
#define RP23XX_SPI_SSPCR0_DSS_SHIFT                 (0)

#define RP23XX_SPI_SSPCR1_SOD                       (1 << 3)  /* Slave-mode output disable */
#define RP23XX_SPI_SSPCR1_MS                        (1 << 2)  /* Master or slave mode select */
#define RP23XX_SPI_SSPCR1_SSE                       (1 << 1)  /* Synchronous serial port enable: 0 SSP operation disabled. 1 SSP operation enabled. */
#define RP23XX_SPI_SSPCR1_LBM                       (1 << 0)  /* Loop back mode */

#define RP23XX_SPI_SSPDR_DATA_MASK                  (0xffff)  /* Transmit/Receive FIFO */

#define RP23XX_SPI_SSPSR_BSY                        (1 << 4)  /* PrimeCell SSP busy flag */
#define RP23XX_SPI_SSPSR_RFF                        (1 << 3)  /* Receive FIFO full */
#define RP23XX_SPI_SSPSR_RNE                        (1 << 2)  /* Receive FIFO not empty */
#define RP23XX_SPI_SSPSR_TNF                        (1 << 1)  /* Transmit FIFO not full */
#define RP23XX_SPI_SSPSR_TFE                        (1 << 0)  /* Transmit FIFO empty */

#define RP23XX_SPI_SSPCPSR_CPSDVSR_MASK             (0xff)    /* Clock prescale divisor. Must be an even number from 2-254 */

#define RP23XX_SPI_SSPIMSC_TXIM                     (1 << 3)  /* Transmit FIFO interrupt mask */
#define RP23XX_SPI_SSPIMSC_RXIM                     (1 << 2)  /* Receive FIFO interrupt mask */
#define RP23XX_SPI_SSPIMSC_RTIM                     (1 << 1)  /* Receive timeout interrupt mask */
#define RP23XX_SPI_SSPIMSC_RORIM                    (1 << 0)  /* Receive overrun interrupt mask */

#define RP23XX_SPI_SSPRIS_TXRIS                     (1 << 3)  /* Gives the raw interrupt state, prior to masking, of the SSPTXINTR interrupt */
#define RP23XX_SPI_SSPRIS_RXRIS                     (1 << 2)  /* Gives the raw interrupt state, prior to masking, of the SSPRXINTR interrupt */
#define RP23XX_SPI_SSPRIS_RTRIS                     (1 << 1)  /* Gives the raw interrupt state, prior to masking, of the SSPRTINTR interrupt */
#define RP23XX_SPI_SSPRIS_RORRIS                    (1 << 0)  /* Gives the raw interrupt state, prior to masking, of the SSPRORINTR interrupt */

#define RP23XX_SPI_SSPMIS_TXMIS                     (1 << 3)  /* Gives the transmit FIFO masked interrupt state, after masking, of the SSPTXINTR interrupt */
#define RP23XX_SPI_SSPMIS_RXMIS                     (1 << 2)  /* Gives the receive FIFO masked interrupt state, after masking, of the SSPRXINTR interrupt */
#define RP23XX_SPI_SSPMIS_RTMIS                     (1 << 1)  /* Gives the receive timeout masked interrupt state, after masking, of the SSPRTINTR interrupt */
#define RP23XX_SPI_SSPMIS_RORMIS                    (1 << 0)  /* Gives the receive over run masked interrupt status, after masking, of the SSPRORINTR interrupt */

#define RP23XX_SPI_SSPICR_RTIC                      (1 << 1)  /* Clears the SSPRTINTR interrupt */
#define RP23XX_SPI_SSPICR_RORIC                     (1 << 0)  /* Clears the SSPRORINTR interrupt */

#define RP23XX_SPI_SSPDMACR_TXDMAE                  (1 << 1)  /* Transmit DMA Enable. If this bit is set to 1, DMA for the transmit FIFO is enabled. */
#define RP23XX_SPI_SSPDMACR_RXDMAE                  (1 << 0)  /* Receive DMA Enable. If this bit is set to 1, DMA for the receive FIFO is enabled. */

#define RP23XX_SPI_SSPPERIPHID0_PARTNUMBER0_MASK    (0xff)  /* These bits read back as 0x22 */

#define RP23XX_SPI_SSPPERIPHID1_DESIGNER0_SHIFT     (4)  /* These bits read back as 0x1 */
#define RP23XX_SPI_SSPPERIPHID1_DESIGNER0_MASK      (0x0f << RP23XX_SPI_SSPPERIPHID1_DESIGNER0_SHIFT)
#define RP23XX_SPI_SSPPERIPHID1_PARTNUMBER1_MASK    (0x0f)  /* These bits read back as 0x0 */

#define RP23XX_SPI_SSPPERIPHID2_REVISION_SHIFT      (4)  /* These bits return the peripheral revision */
#define RP23XX_SPI_SSPPERIPHID2_REVISION_MASK       (0x0f << RP23XX_SPI_SSPPERIPHID2_REVISION_SHIFT)
#define RP23XX_SPI_SSPPERIPHID2_DESIGNER1_MASK      (0x0f)  /* These bits read back as 0x4 */

#define RP23XX_SPI_SSPPERIPHID3_CONFIGURATION_MASK  (0xff)  /* These bits read back as 0x00 */

#define RP23XX_SPI_SSPPCELLID0_MASK                 (0xff)  /* These bits read back as 0x0D */

#define RP23XX_SPI_SSPPCELLID1_MASK                 (0xff)  /* These bits read back as 0xF0 */

#define RP23XX_SPI_SSPPCELLID2_MASK                 (0xff)  /* These bits read back as 0x05 */

#define RP23XX_SPI_SSPPCELLID3_MASK                 (0xff)  /* These bits read back as 0xB1 */

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_SPI_H */
