/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_spi.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SPI_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_SPI_SSPCR0_OFFSET        0x000000  /* Control register 0 */
#define RP2040_SPI_SSPCR1_OFFSET        0x000004  /* Control register 1 */
#define RP2040_SPI_SSPDR_OFFSET         0x000008  /* Data register */
#define RP2040_SPI_SSPSR_OFFSET         0x00000c  /* Status register */
#define RP2040_SPI_SSPCPSR_OFFSET       0x000010  /* Clock prescale register */
#define RP2040_SPI_SSPIMSC_OFFSET       0x000014  /* Interrupt mask set or clear register */
#define RP2040_SPI_SSPRIS_OFFSET        0x000018  /* Raw interrupt status register */
#define RP2040_SPI_SSPMIS_OFFSET        0x00001c  /* Masked interrupt status register */
#define RP2040_SPI_SSPICR_OFFSET        0x000020  /* Interrupt clear register */
#define RP2040_SPI_SSPDMACR_OFFSET      0x000024  /* DMA control register */
#define RP2040_SPI_SSPPERIPHID0_OFFSET  0x000fe0  /* Peripheral identification registers */
#define RP2040_SPI_SSPPERIPHID1_OFFSET  0x000fe4  /* Peripheral identification registers */
#define RP2040_SPI_SSPPERIPHID2_OFFSET  0x000fe8  /* Peripheral identification registers */
#define RP2040_SPI_SSPPERIPHID3_OFFSET  0x000fec  /* Peripheral identification registers */
#define RP2040_SPI_SSPPCELLID0_OFFSET   0x000ff0  /* PrimeCell identification registers */
#define RP2040_SPI_SSPPCELLID1_OFFSET   0x000ff4  /* PrimeCell identification registers */
#define RP2040_SPI_SSPPCELLID2_OFFSET   0x000ff8  /* PrimeCell identification registers */
#define RP2040_SPI_SSPPCELLID3_OFFSET   0x000ffc  /* PrimeCell identification registers */

/* Register definitions *****************************************************/

#define RP2040_SPI_SSPCR0(n)        (RP2040_SPI_BASE(n) + RP2040_SPI_SSPCR0_OFFSET)
#define RP2040_SPI_SSPCR1(n)        (RP2040_SPI_BASE(n) + RP2040_SPI_SSPCR1_OFFSET)
#define RP2040_SPI_SSPDR(n)         (RP2040_SPI_BASE(n) + RP2040_SPI_SSPDR_OFFSET)
#define RP2040_SPI_SSPSR(n)         (RP2040_SPI_BASE(n) + RP2040_SPI_SSPSR_OFFSET)
#define RP2040_SPI_SSPCPSR(n)       (RP2040_SPI_BASE(n) + RP2040_SPI_SSPCPSR_OFFSET)
#define RP2040_SPI_SSPIMSC(n)       (RP2040_SPI_BASE(n) + RP2040_SPI_SSPIMSC_OFFSET)
#define RP2040_SPI_SSPRIS(n)        (RP2040_SPI_BASE(n) + RP2040_SPI_SSPRIS_OFFSET)
#define RP2040_SPI_SSPMIS(n)        (RP2040_SPI_BASE(n) + RP2040_SPI_SSPMIS_OFFSET)
#define RP2040_SPI_SSPICR(n)        (RP2040_SPI_BASE(n) + RP2040_SPI_SSPICR_OFFSET)
#define RP2040_SPI_SSPDMACR(n)      (RP2040_SPI_BASE(n) + RP2040_SPI_SSPDMACR_OFFSET)
#define RP2040_SPI_SSPPERIPHID0(n)  (RP2040_SPI_BASE(n) + RP2040_SPI_SSPPERIPHID0_OFFSET)
#define RP2040_SPI_SSPPERIPHID1(n)  (RP2040_SPI_BASE(n) + RP2040_SPI_SSPPERIPHID1_OFFSET)
#define RP2040_SPI_SSPPERIPHID2(n)  (RP2040_SPI_BASE(n) + RP2040_SPI_SSPPERIPHID2_OFFSET)
#define RP2040_SPI_SSPPERIPHID3(n)  (RP2040_SPI_BASE(n) + RP2040_SPI_SSPPERIPHID3_OFFSET)
#define RP2040_SPI_SSPPCELLID0(n)   (RP2040_SPI_BASE(n) + RP2040_SPI_SSPPCELLID0_OFFSET)
#define RP2040_SPI_SSPPCELLID1(n)   (RP2040_SPI_BASE(n) + RP2040_SPI_SSPPCELLID1_OFFSET)
#define RP2040_SPI_SSPPCELLID2(n)   (RP2040_SPI_BASE(n) + RP2040_SPI_SSPPCELLID2_OFFSET)
#define RP2040_SPI_SSPPCELLID3(n)   (RP2040_SPI_BASE(n) + RP2040_SPI_SSPPCELLID3_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_SPI_SSPCR0_SCR_SHIFT                 (8)       /* Serial clock rate */
#define RP2040_SPI_SSPCR0_SCR_MASK                  (0xff << RP2040_SPI_SSPCR0_SCR_SHIFT)
#define RP2040_SPI_SSPCR0_SPH                       (1 << 7)  /* SSPCLKOUT phase */
#define RP2040_SPI_SSPCR0_SPO                       (1 << 6)  /* SSPCLKOUT polarity */
#define RP2040_SPI_SSPCR0_FRF_SHIFT                 (4)       /* Frame format */
#define RP2040_SPI_SSPCR0_FRF_MASK                  (0x03 << RP2040_SPI_SSPCR0_FRF_SHIFT)
#define RP2040_SPI_SSPCR0_DSS_MASK                  (0x0f)    /* Data Size Select */
#define RP2040_SPI_SSPCR0_DSS_SHIFT                 (0)

#define RP2040_SPI_SSPCR1_SOD                       (1 << 3)  /* Slave-mode output disable */
#define RP2040_SPI_SSPCR1_MS                        (1 << 2)  /* Master or slave mode select */
#define RP2040_SPI_SSPCR1_SSE                       (1 << 1)  /* Synchronous serial port enable: 0 SSP operation disabled. 1 SSP operation enabled. */
#define RP2040_SPI_SSPCR1_LBM                       (1 << 0)  /* Loop back mode */

#define RP2040_SPI_SSPDR_DATA_MASK                  (0xffff)  /* Transmit/Receive FIFO */

#define RP2040_SPI_SSPSR_BSY                        (1 << 4)  /* PrimeCell SSP busy flag */
#define RP2040_SPI_SSPSR_RFF                        (1 << 3)  /* Receive FIFO full */
#define RP2040_SPI_SSPSR_RNE                        (1 << 2)  /* Receive FIFO not empty */
#define RP2040_SPI_SSPSR_TNF                        (1 << 1)  /* Transmit FIFO not full */
#define RP2040_SPI_SSPSR_TFE                        (1 << 0)  /* Transmit FIFO empty */

#define RP2040_SPI_SSPCPSR_CPSDVSR_MASK             (0xff)    /* Clock prescale divisor. Must be an even number from 2-254 */

#define RP2040_SPI_SSPIMSC_TXIM                     (1 << 3)  /* Transmit FIFO interrupt mask */
#define RP2040_SPI_SSPIMSC_RXIM                     (1 << 2)  /* Receive FIFO interrupt mask */
#define RP2040_SPI_SSPIMSC_RTIM                     (1 << 1)  /* Receive timeout interrupt mask */
#define RP2040_SPI_SSPIMSC_RORIM                    (1 << 0)  /* Receive overrun interrupt mask */

#define RP2040_SPI_SSPRIS_TXRIS                     (1 << 3)  /* Gives the raw interrupt state, prior to masking, of the SSPTXINTR interrupt */
#define RP2040_SPI_SSPRIS_RXRIS                     (1 << 2)  /* Gives the raw interrupt state, prior to masking, of the SSPRXINTR interrupt */
#define RP2040_SPI_SSPRIS_RTRIS                     (1 << 1)  /* Gives the raw interrupt state, prior to masking, of the SSPRTINTR interrupt */
#define RP2040_SPI_SSPRIS_RORRIS                    (1 << 0)  /* Gives the raw interrupt state, prior to masking, of the SSPRORINTR interrupt */

#define RP2040_SPI_SSPMIS_TXMIS                     (1 << 3)  /* Gives the transmit FIFO masked interrupt state, after masking, of the SSPTXINTR interrupt */
#define RP2040_SPI_SSPMIS_RXMIS                     (1 << 2)  /* Gives the receive FIFO masked interrupt state, after masking, of the SSPRXINTR interrupt */
#define RP2040_SPI_SSPMIS_RTMIS                     (1 << 1)  /* Gives the receive timeout masked interrupt state, after masking, of the SSPRTINTR interrupt */
#define RP2040_SPI_SSPMIS_RORMIS                    (1 << 0)  /* Gives the receive over run masked interrupt status, after masking, of the SSPRORINTR interrupt */

#define RP2040_SPI_SSPICR_RTIC                      (1 << 1)  /* Clears the SSPRTINTR interrupt */
#define RP2040_SPI_SSPICR_RORIC                     (1 << 0)  /* Clears the SSPRORINTR interrupt */

#define RP2040_SPI_SSPDMACR_TXDMAE                  (1 << 1)  /* Transmit DMA Enable. If this bit is set to 1, DMA for the transmit FIFO is enabled. */
#define RP2040_SPI_SSPDMACR_RXDMAE                  (1 << 0)  /* Receive DMA Enable. If this bit is set to 1, DMA for the receive FIFO is enabled. */

#define RP2040_SPI_SSPPERIPHID0_PARTNUMBER0_MASK    (0xff)  /* These bits read back as 0x22 */

#define RP2040_SPI_SSPPERIPHID1_DESIGNER0_SHIFT     (4)  /* These bits read back as 0x1 */
#define RP2040_SPI_SSPPERIPHID1_DESIGNER0_MASK      (0x0f << RP2040_SPI_SSPPERIPHID1_DESIGNER0_SHIFT)
#define RP2040_SPI_SSPPERIPHID1_PARTNUMBER1_MASK    (0x0f)  /* These bits read back as 0x0 */

#define RP2040_SPI_SSPPERIPHID2_REVISION_SHIFT      (4)  /* These bits return the peripheral revision */
#define RP2040_SPI_SSPPERIPHID2_REVISION_MASK       (0x0f << RP2040_SPI_SSPPERIPHID2_REVISION_SHIFT)
#define RP2040_SPI_SSPPERIPHID2_DESIGNER1_MASK      (0x0f)  /* These bits read back as 0x4 */

#define RP2040_SPI_SSPPERIPHID3_CONFIGURATION_MASK  (0xff)  /* These bits read back as 0x00 */

#define RP2040_SPI_SSPPCELLID0_MASK                 (0xff)  /* These bits read back as 0x0D */

#define RP2040_SPI_SSPPCELLID1_MASK                 (0xff)  /* These bits read back as 0xF0 */

#define RP2040_SPI_SSPPCELLID2_MASK                 (0xff)  /* These bits read back as 0x05 */

#define RP2040_SPI_SSPPCELLID3_MASK                 (0xff)  /* These bits read back as 0xB1 */

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SPI_H */
