/************************************************************************************
 * arch/arm/src/stm32h7/chip/stm32h7x3xx_spi.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_SPI_H
#define __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_SPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32H7_STM32H7X3XX)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Maximum allowed speed as per data sheet for all SPIs */

#  define STM32_SPI_CLK_MAX      150000000UL

/* Register Offsets *****************************************************************/

#define STM32_SPI_CR1_OFFSET      0x0000  /* SPI/I2S Control Register 1 */
#define STM32_SPI_CR2_OFFSET      0x0004  /* SPI control register 2 */
#define STM32_SPI_CFG1_OFFSET     0x0008  /* SPI configuration register 1 */
#define STM32_SPI_CFG2_OFFSET     0x000C  /* SPI configuration register 2 */
#define STM32_SPI_IER_OFFSET      0x0010  /* SPI/I2S interupt enable register */
#define STM32_SPI_SR_OFFSET       0x0014  /* SPI/I2S status register */
#define STM32_SPI_IFCR_OFFSET     0x0018  /* SPI/I2S interrupt/status flags clear register */
#define STM32_SPI_TXDR_OFFSET     0x0020  /* SPI/I2S transmit data register */
#define STM32_SPI_RXDR_OFFSET     0x0030  /* SPI/I2S receive data register */
#define STM32_SPI_CRCPOLY_OFFSET  0x0040  /* SPI/I2S SPI polynominal register */
#define STM32_SPI_TXCRC_OFFSET    0x0044  /* SPI/I2S SPI transmitter CRC register */
#define STM32_SPI_RXCRC_OFFSET    0x0048  /* SPI/I2S SPI receiver CRC register */
#define STM32_SPI_UDRDR_OFFSET    0x004C  /* SPI/I2S SPI underrun data register */
#define STM32_SPI_I2SCFGR_OFFSET  0x0050  /* SPI/I2S configuration register*/

/* Register Addresses ***************************************************************/

#if STM32H7_NSPI > 0
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
#  define STM32_SPI1_I2SCFGR      (STM32_SPI1_BASE+STM32_SPI_I2SCFGR_OFFSET)
#endif

#if STM32H7_NSPI > 1
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
#  define STM32_SPI2_I2SCFGR      (STM32_SPI2_BASE+STM32_SPI_I2SCFGR_OFFSET)
#endif

#if STM32H7_NSPI > 2
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
#  define STM32_SPI3_I2SCFGR      (STM32_SPI3_BASE+STM32_SPI_I2SCFGR_OFFSET)
#endif

#if STM32H7_NSPI > 3
#  define STM32_SPI4_CR1          (STM32_SPI4_BASE+STM32_SPI_CR1_OFFSET)
#  define STM32_SPI4_CR2          (STM32_SPI4_BASE+STM32_SPI_CR2_OFFSET)
#  define STM32_SPI4_CFG1         (STM32_SPI4_BASE+STM32_SPI_CFG1_OFFSET)
#  define STM32_SPI4_CFG2         (STM32_SPI4_BASE+STM32_SPI_CFG2_OFFSET)
#  define STM32_SPI4_IER          (STM32_SPI4_BASE+STM32_SPI_IER_OFFSET)
#  define STM32_SPI4_SR           (STM32_SPI4_BASE+STM32_SPI_SR_OFFSET)
#  define STM32_SPI4_IFCR         (STM32_SPI4_BASE+STM32_SPI_IFCR_OFFSET)
#  define STM32_SPI4_TXDR         (STM32_SPI4_BASE+STM32_SPI_TXDR_OFFSET)
#  define STM32_SPI4_RXDR         (STM32_SPI4_BASE+STM32_SPI_RXDR_OFFSET)
#  define STM32_SPI4_CRCPOLY      (STM32_SPI4_BASE+STM32_SPI_CRCPOLY_OFFSET)
#  define STM32_SPI4_TXCRC        (STM32_SPI4_BASE+STM32_SPI_TXCRC_OFFSET)
#  define STM32_SPI4_RXCRC        (STM32_SPI4_BASE+STM32_SPI_RXCRC_OFFSET)
#  define STM32_SPI4_UDRDR        (STM32_SPI4_BASE+STM32_SPI_UDRDR_OFFSET)
#  define STM32_SPI4_I2SCFGR      (STM32_SPI4_BASE+STM32_SPI_I2SCFGR_OFFSET)
#endif

#if STM32H7_NSPI > 4
#  define STM32_SPI5_CR1          (STM32_SPI5_BASE+STM32_SPI_CR1_OFFSET)
#  define STM32_SPI5_CR2          (STM32_SPI5_BASE+STM32_SPI_CR2_OFFSET)
#  define STM32_SPI5_CFG1         (STM32_SPI5_BASE+STM32_SPI_CFG1_OFFSET)
#  define STM32_SPI5_CFG2         (STM32_SPI5_BASE+STM32_SPI_CFG2_OFFSET)
#  define STM32_SPI5_IER          (STM32_SPI5_BASE+STM32_SPI_IER_OFFSET)
#  define STM32_SPI5_SR           (STM32_SPI5_BASE+STM32_SPI_SR_OFFSET)
#  define STM32_SPI5_IFCR         (STM32_SPI5_BASE+STM32_SPI_IFCR_OFFSET)
#  define STM32_SPI5_TXDR         (STM32_SPI5_BASE+STM32_SPI_TXDR_OFFSET)
#  define STM32_SPI5_RXDR         (STM32_SPI5_BASE+STM32_SPI_RXDR_OFFSET)
#  define STM32_SPI5_CRCPOLY      (STM32_SPI5_BASE+STM32_SPI_CRCPOLY_OFFSET)
#  define STM32_SPI5_TXCRC        (STM32_SPI5_BASE+STM32_SPI_TXCRC_OFFSET)
#  define STM32_SPI5_RXCRC        (STM32_SPI5_BASE+STM32_SPI_RXCRC_OFFSET)
#  define STM32_SPI5_UDRDR        (STM32_SPI5_BASE+STM32_SPI_UDRDR_OFFSET)
#  define STM32_SPI5_I2SCFGR      (STM32_SPI5_BASE+STM32_SPI_I2SCFGR_OFFSET)
#endif

#if STM32H7_NSPI > 5
#  define STM32_SPI6_CR1          (STM32_SPI6_BASE+STM32_SPI_CR1_OFFSET)
#  define STM32_SPI6_CR2          (STM32_SPI6_BASE+STM32_SPI_CR2_OFFSET)
#  define STM32_SPI6_CFG1         (STM32_SPI6_BASE+STM32_SPI_CFG1_OFFSET)
#  define STM32_SPI6_CFG2         (STM32_SPI6_BASE+STM32_SPI_CFG2_OFFSET)
#  define STM32_SPI6_IER          (STM32_SPI6_BASE+STM32_SPI_IER_OFFSET)
#  define STM32_SPI6_SR           (STM32_SPI6_BASE+STM32_SPI_SR_OFFSET)
#  define STM32_SPI6_IFCR         (STM32_SPI6_BASE+STM32_SPI_IFCR_OFFSET)
#  define STM32_SPI6_TXDR         (STM32_SPI6_BASE+STM32_SPI_TXDR_OFFSET)
#  define STM32_SPI6_RXDR         (STM32_SPI6_BASE+STM32_SPI_RXDR_OFFSET)
#  define STM32_SPI6_CRCPOLY      (STM32_SPI6_BASE+STM32_SPI_CRCPOLY_OFFSET)
#  define STM32_SPI6_TXCRC        (STM32_SPI6_BASE+STM32_SPI_TXCRC_OFFSET)
#  define STM32_SPI6_RXCRC        (STM32_SPI6_BASE+STM32_SPI_RXCRC_OFFSET)
#  define STM32_SPI6_UDRDR        (STM32_SPI6_BASE+STM32_SPI_UDRDR_OFFSET)
#  define STM32_SPI6_I2SCFGR      (STM32_SPI6_BASE+STM32_SPI_I2SCFGR_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

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

/* TODO: SPI configuration register 1 */

/* TODO: SPI configuration register 2 */

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

/* TODO: SPI/I2S configuration register*/


#endif /* CONFIG_STM32H7_STM32H7X3XX */
#endif /* __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_SPI_H */
