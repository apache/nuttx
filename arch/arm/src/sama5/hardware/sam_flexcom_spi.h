/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_flexcom_spi.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_SPI_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAM_FLEXCOM_SPI_NCS               2      /* Two chip selects */

/* Flexcom SPI Register Offsets *********************************************/

/* SPI register offsets *****************************************************/

#define SAM_FLEXCOM_SPI_CR_OFFSET         0x0400 /* Control Register */
#define SAM_FLEXCOM_SPI_MR_OFFSET         0x0404 /* Mode Register */
#define SAM_FLEXCOM_SPI_RDR_OFFSET        0x0408 /* Receive Data Register */
#define SAM_FLEXCOM_SPI_TDR_OFFSET        0x040c /* Transmit Data Register */
#define SAM_FLEXCOM_SPI_SR_OFFSET         0x0410 /* Status Register */
#define SAM_FLEXCOM_SPI_IER_OFFSET        0x0414 /* Interrupt Enable Register */
#define SAM_FLEXCOM_SPI_IDR_OFFSET        0x0418 /* Interrupt Disable Register */
#define SAM_FLEXCOM_SPI_IMR_OFFSET        0x041c /* Interrupt Mask Register */
                                                 /* 0x420-0x42c: Reserved */
#define SAM_FLEXCOM_SPI_CSR0_OFFSET       0x0430 /* Chip Select Register 0 */
#define SAM_FLEXCOM_SPI_CSR1_OFFSET       0x0434 /* Chip Select Register 1 */
                                                 /* 0x438-0x43C: Reserved */
#define SAM_FLEXCOM_SPI_FMR_OFFSET        0x0440 /* FIFO Mode register */
#define SAM_FLEXCOM_SPI_FLR_OFFSET        0x0444 /* FIFO Level Register */
#define SAM_FLEXCOM_SPI_CMPR_OFFSET       0x0448 /* Comparison register */
                                                 /* 0x044C-0x04E0: Reserved */
#define SAM_FLEXCOM_SPI_WPMR_OFFSET       0x04e4 /* Write Protection Mode Register */
#define SAM_FLEXCOM_SPI_WPSR_OFFSET       0x04e8 /* Write Protection Status Register */
                                                 /* 0x04ec-0x5FC: Reserved */

/* Flexcom SPI Register Addresses *******************************************/
#ifdef CONFIG_SAMA5_FLEXCOM0_SPI
#define SAM_FLEXCOM_SPI0_CR               (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_CR_OFFSET)
#define SAM_FLEXCOM_SPI0_MR               (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_MR_OFFSET)
#define SAM_FLEXCOM_SPI0_RDR              (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_RDR_OFFSET)
#define SAM_FLEXCOM_SPI0_TDR              (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_TDR_OFFSET)
#define SAM_FLEXCOM_SPI0_SR               (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_SR_OFFSET)
#define SAM_FLEXCOM_SPI0_IER              (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_IER_OFFSET)
#define SAM_FLEXCOM_SPI0_IDR              (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_IDR_OFFSET)
#define SAM_FLEXCOM_SPI0_IMR              (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_IMR_OFFSET)
#define SAM_FLEXCOM_SPI0_CSR0             (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_CSR0_OFFSET)
#define SAM_FLEXCOM_SPI0_CSR1             (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_CSR1_OFFSET)
#define SAM_FLEXCOM_SPI0_FMR              (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_FMR_OFFSET)
#define SAM_FLEXCOM_SPI0_FLR              (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_FLR_OFFSET)
#define SAM_FLEXCOM_SPI0_CMPR             (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_CMPR_OFFSET)
#define SAM_FLEXCOM_SPI0_WPMR             (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_WPMR_OFFSET)
#define SAM_FLEXCOM_SPI0_WPSR             (SAM_FLEXCOM0_VBASE+SAM_FLEXCOM_SPI_WPSR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_FLEXCOM1_SPI
#define SAM_FLEXCOM_SPI1_CR               (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_CR_OFFSET)
#define SAM_FLEXCOM_SPI1_MR               (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_MR_OFFSET)
#define SAM_FLEXCOM_SPI1_RDR              (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_RDR_OFFSET)
#define SAM_FLEXCOM_SPI1_TDR              (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_TDR_OFFSET)
#define SAM_FLEXCOM_SPI1_SR               (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_SR_OFFSET)
#define SAM_FLEXCOM_SPI1_IER              (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_IER_OFFSET)
#define SAM_FLEXCOM_SPI1_IDR              (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_IDR_OFFSET)
#define SAM_FLEXCOM_SPI1_IMR              (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_IMR_OFFSET)
#define SAM_FLEXCOM_SPI1_CSR0             (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_CSR0_OFFSET)
#define SAM_FLEXCOM_SPI1_CSR1             (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_CSR1_OFFSET)
#define SAM_FLEXCOM_SPI1_FMR              (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_FMR_OFFSET)
#define SAM_FLEXCOM_SPI1_FLR              (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_FLR_OFFSET)
#define SAM_FLEXCOM_SPI1_CMPR             (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_CMPR_OFFSET)
#define SAM_FLEXCOM_SPI1_WPMR             (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_WPMR_OFFSET)
#define SAM_FLEXCOM_SPI1_WPSR             (SAM_FLEXCOM1_VBASE+SAM_FLEXCOM_SPI_WPSR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_FLEXCOM2_SPI
#define SAM_FLEXCOM_SPI2_CR               (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_CR_OFFSET)
#define SAM_FLEXCOM_SPI2_MR               (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_MR_OFFSET)
#define SAM_FLEXCOM_SPI2_RDR              (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_RDR_OFFSET)
#define SAM_FLEXCOM_SPI2_TDR              (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_TDR_OFFSET)
#define SAM_FLEXCOM_SPI2_SR               (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_SR_OFFSET)
#define SAM_FLEXCOM_SPI2_IER              (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_IER_OFFSET)
#define SAM_FLEXCOM_SPI2_IDR              (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_IDR_OFFSET)
#define SAM_FLEXCOM_SPI2_IMR              (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_IMR_OFFSET)
#define SAM_FLEXCOM_SPI2_CSR0             (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_CSR0_OFFSET)
#define SAM_FLEXCOM_SPI2_CSR1             (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_CSR1_OFFSET)
#define SAM_FLEXCOM_SPI2_FMR              (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_FMR_OFFSET)
#define SAM_FLEXCOM_SPI2_FLR              (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_FLR_OFFSET)
#define SAM_FLEXCOM_SPI2_CMPR             (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_CMPR_OFFSET)
#define SAM_FLEXCOM_SPI2_WPMR             (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_WPMR_OFFSET)
#define SAM_FLEXCOM_SPI2_WPSR             (SAM_FLEXCOM2_VBASE+SAM_FLEXCOM_SPI_WPSR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_FLEXCOM3_SPI
#define SAM_FLEXCOM_SPI3_CR               (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_CR_OFFSET)
#define SAM_FLEXCOM_SPI3_MR               (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_MR_OFFSET)
#define SAM_FLEXCOM_SPI3_RDR              (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_RDR_OFFSET)
#define SAM_FLEXCOM_SPI3_TDR              (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_TDR_OFFSET)
#define SAM_FLEXCOM_SPI3_SR               (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_SR_OFFSET)
#define SAM_FLEXCOM_SPI3_IER              (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_IER_OFFSET)
#define SAM_FLEXCOM_SPI3_IDR              (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_IDR_OFFSET)
#define SAM_FLEXCOM_SPI3_IMR              (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_IMR_OFFSET)
#define SAM_FLEXCOM_SPI3_CSR0             (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_CSR0_OFFSET)
#define SAM_FLEXCOM_SPI3_CSR1             (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_CSR1_OFFSET)
#define SAM_FLEXCOM_SPI3_FMR              (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_FMR_OFFSET)
#define SAM_FLEXCOM_SPI3_FLR              (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_FLR_OFFSET)
#define SAM_FLEXCOM_SPI3_CMPR             (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_CMPR_OFFSET)
#define SAM_FLEXCOM_SPI3_WPMR             (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_WPMR_OFFSET)
#define SAM_FLEXCOM_SPI3_WPSR             (SAM_FLEXCOM3_VBASE+SAM_FLEXCOM_SPI_WPSR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_FLEXCOM4_SPI
#define SAM_FLEXCOM_SPI4_CR               (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_CR_OFFSET)
#define SAM_FLEXCOM_SPI4_MR               (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_MR_OFFSET)
#define SAM_FLEXCOM_SPI4_RDR              (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_RDR_OFFSET)
#define SAM_FLEXCOM_SPI4_TDR              (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_TDR_OFFSET)
#define SAM_FLEXCOM_SPI4_SR               (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_SR_OFFSET)
#define SAM_FLEXCOM_SPI4_IER              (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_IER_OFFSET)
#define SAM_FLEXCOM_SPI4_IDR              (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_IDR_OFFSET)
#define SAM_FLEXCOM_SPI4_IMR              (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_IMR_OFFSET)
#define SAM_FLEXCOM_SPI4_CSR0             (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_CSR0_OFFSET)
#define SAM_FLEXCOM_SPI4_CSR1             (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_CSR1_OFFSET)
#define SAM_FLEXCOM_SPI4_FMR              (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_FMR_OFFSET)
#define SAM_FLEXCOM_SPI4_FLR              (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_FLR_OFFSET)
#define SAM_FLEXCOM_SPI4_CMPR             (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_CMPR_OFFSET)
#define SAM_FLEXCOM_SPI4_WPMR             (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_WPMR_OFFSET)
#define SAM_FLEXCOM_SPI4_WPSR             (SAM_FLEXCOM4_VBASE+SAM_FLEXCOM_SPI_WPSR_OFFSET)
#endif

/* Flexcom SPI Register Bit Field Definitions *******************************/

/* SPI Control Register */

#define FLEXCOM_SPI_CR_SPIEN              (1 << 0)  /* Bit 0:  SPI Enable */
#define FLEXCOM_SPI_CR_SPIDIS             (1 << 1)  /* Bit 1:  SPI Disable */
#define FLEXCOM_SPI_CR_SWRST              (1 << 7)  /* Bit 7:  SPI Software Reset */
#define FLEXCOM_SPI_CR_REQCLR             (1 << 12) /* Bit 12: SPI Request to Clear Comparison Trigger */
#define FLEXCOM_SPI_CR_TXFCLR             (1 << 16) /* Bit 16: SPI Transmit FIFO clear */
#define FLEXCOM_SPI_CR_RXFCLR             (1 << 17) /* Bit 17: SPI Receive FIFO clear */
#define FLEXCOM_SPI_CR_LASTXFER           (1 << 24) /* Bit 24: Last Transfer */
#define FLEXCOM_SPI_CR_FIFOEN             (1 << 30) /* Bit 30: Enable TX and RX FIFOs */
#define FLEXCOM_SPI_CR_FIFODIS            (1 << 31) /* Bit 31: Disable TX and RX FIFOs */

/* SPI Mode Register */

#define FLEXCOM_SPI_MR_MSTR               (1 << 0)  /* Bit 0:  Master/Slave Mode */
#define FLEXCOM_SPI_MR_PS                 (1 << 1)  /* Bit 1:  Peripheral Select */
#define FLEXCOM_SPI_MR_PCSDEC             (1 << 2)  /* Bit 2:  Chip Select Decode */
#define FLEXCOM_SPI_MR_BSCRCCLK           (1 << 3)  /* Bit 3:  Bit Rate Source Clock */
#define FLEXCOM_SPI_MR_MODFDIS            (1 << 4)  /* Bit 4:  Mode Fault Detection */
#define FLEXCOM_SPI_MR_WDRBT              (1 << 5)  /* Bit 5:  Wait Data Read Before Transfer */
#define FLEXCOM_SPI_MR_LLB                (1 << 7)  /* Bit 7:  Local Loopback Enable */
#define FLEXCOM_SPI_MR_CMPMODE            (1 << 12) /* Bit 12: Comparison Mode */
#define FLEXCOM_SPI_MR_PCS_SHIFT          (16)      /* Bits 16-17: Peripheral Chip Select */
#define FLEXCOM_SPI_MR_PCS_MASK           (3 << FLEXCOM_SPI_MR_PCS_SHIFT)
                                                    /* Only 2 chip selects */
#define FLEXCOM_SPI_MR_PCS0               (0 << FLEXCOM_SPI_MR_PCS_SHIFT)
                                                    /* NPCS[1:0] = 10 */
#define FLEXCOM_SPI_MR_PCS1               (1 << FLEXCOM_SPI_MR_PCS_SHIFT)
                                                    /* NPCS[1:0] = 01 */

#define FLEXCOM_SPI_MR_DLYBCS_SHIFT       (24)      /* Bits 24-31: Delay Between Chip Selects */
#define FLEXCOM_SPI_MR_DLYBCS_MASK        (0xff << FLEXCOM_SPI_MR_DLYBCS_SHIFT)

/* SPI Receive Data Register */

#define FLEXCOM_SPI_RDR_RD_SHIFT          (0)       /* Bits 0-15: Receive Data */
#define FLEXCOM_SPI_RDR_RD_MASK           (0xffff << FLEXCOM_SPI_RDR_RD_SHIFT)
#define FLEXCOM_SPI_RDR_PCS_SHIFT         (16)      /* Bits 16-19: Peripheral Chip Select */
#define FLEXCOM_SPI_RDR_PCS_MASK          (3 << FLEXCOM_SPI_RDR_PCS_SHIFT)
#  define FLEXCOM_SPI_RDR_PCS0            (0 << FLEXCOM_SPI_RDR_PCS_SHIFT) /* NPCS[1:0] = 10 */
#  define FLEXCOM_SPI_RDR_PCS1            (1 << FLEXCOM_SPI_RDR_PCS_SHIFT) /* NPCS[1:0] = 01 */

/* SPI Transmit Data Register */

#define FLEXCOM_SPI_TDR_TD_SHIFT          (0)       /* Bits 0-15:  Transmit Data */
#define FLEXCOM_SPI_TDR_TD_MASK           (0xffff << FLEXCOM_SPI_TDR_TD_SHIFT)
#define FLEXCOM_SPI_TDR_PCS_SHIFT         (16)      /* Bits 16-19: Peripheral Chip Select */
#define FLEXCOM_SPI_TDR_PCS_MASK          (3 << FLEXCOM_SPI_TDR_PCS_SHIFT)
#define FLEXCOM_SPI_TDR_PCS0              (0 << FLEXCOM_SPI_TDR_PCS_SHIFT) /* NPCS[1:0] = 1110 (w/PCSDEC=1) */
#define FLEXCOM_SPI_TDR_PCS1              (1 << FLEXCOM_SPI_TDR_PCS_SHIFT) /* NPCS[1:0] = 1101 (w/PCSDEC=1) */

#define FLEXCOM_SPI_TDR_LASTXFER          (1 << 24) /* Bit 24: Last Transfer */

/* SPI Status Register, SPI Interrupt Enable Register, SPI Interrupt
 * Disable Register, and SPI Interrupt Mask Register (common bit fields)
 */

#define FLEXCOM_SPI_INT_RDRF              (1 << 0)  /* Bit 0:  Receive Data Register Full Interrupt */
#define FLEXCOM_SPI_INT_TDRE              (1 << 1)  /* Bit 1:  Transmit Data Register Empty Interrupt */
#define FLEXCOM_SPI_INT_MODF              (1 << 2)  /* Bit 2:  Mode Fault Error Interrupt */
#define FLEXCOM_SPI_INT_OVRES             (1 << 3)  /* Bit 3:  Overrun Error Interrupt */
#define FLEXCOM_SPI_INT_NSSR              (1 << 8)  /* Bit 8:  NSS Rising Interrupt */
#define FLEXCOM_SPI_INT_TXEMPTY           (1 << 9)  /* Bit 9:  Transmission Registers Empty Interrupt */
#define FLEXCOM_SPI_INT_UNDES             (1 << 10) /* Bit 10: Underrun Error Status Interrupt (slave) */
#define FLEXCOM_SPI_INT_CMP               (1 << 11) /* Bit 11: Comparison Status */
#define FLEXCOM_SPI_SR_SPIENS             (1 << 16) /* Bit 16: SPI Enable Status (SR only) */
#define FLEXCOM_SPI_INT_TXFEF             (1 << 24) /* Bit 24: TX FIFO empty flag  */
#define FLEXCOM_SPI_INT_TXFFF             (1 << 25) /* Bit 25: TX FIFO full flag  */
#define FLEXCOM_SPI_INT_TXFTHF            (1 << 26) /* Bit 25: TX FIFO FIFO Threshold flag  */
#define FLEXCOM_SPI_INT_RXFEF             (1 << 27) /* Bit 27: RX FIFO empty flag   */
#define FLEXCOM_SPI_INT_RXFFF             (1 << 28) /* Bit 28: RX FIFO full flag   */
#define FLEXCOM_SPI_INT_RXFTHF            (1 << 29) /* Bit 29: RX FIFO FIFO Threshold flag  */
#define FLEXCOM_SPI_INT_TXFPTEF           (1 << 30) /* Bit 30: TX FIFO pointer error flag  */
#define FLEXCOM_SPI_INT_RXFPTEF           (1 << 31) /* Bit 31: RX FIFO pointer error flag  */

/* SPI Chip Select Registers 0-3 */

#define FLEXCOM_SPI_CSR_CPOL              (1 << 0)  /* Bit 0:  Clock Polarity */
#define FLEXCOM_SPI_CSR_NCPHA             (1 << 1)  /* Bit 1:  Clock Phase */
#define FLEXCOM_SPI_CSR_CSNAAT            (1 << 2)  /* Bit 2:  Chip Select Not Active After Transfer */
#define FLEXCOM_SPI_CSR_CSAAT             (1 << 3)  /* Bit 3:  Chip Select Active After Transfer */
#define FLEXCOM_SPI_CSR_BITS_SHIFT        (4)       /* Bits 4-7: Bits Per Transfer */
#define FLEXCOM_SPI_CSR_BITS_MASK         (15 << FLEXCOM_SPI_CSR_BITS_SHIFT)
#  define FLEXCOM_SPI_CSR_BITS(n)         (((n)-8) << FLEXCOM_SPI_CSR_BITS_SHIFT) /* n, n=8-16 */

#  define FLEXCOM_SPI_CSR_BITS8           (0 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 8 */
#  define FLEXCOM_SPI_CSR_BITS9           (1 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 9 */
#  define FLEXCOM_SPI_CSR_BITS10          (2 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 10 */
#  define FLEXCOM_SPI_CSR_BITS11          (3 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 11 */
#  define FLEXCOM_SPI_CSR_BITS12          (4 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 12 */
#  define FLEXCOM_SPI_CSR_BITS13          (5 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 13 */
#  define FLEXCOM_SPI_CSR_BITS14          (6 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 14 */
#  define FLEXCOM_SPI_CSR_BITS15          (7 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 15 */
#  define FLEXCOM_SPI_CSR_BITS16          (8 << FLEXCOM_SPI_CSR_BITS_SHIFT) /* 16 */

#define FLEXCOM_SPI_CSR_SCBR_SHIFT        (8)       /* Bits 8-15: Serial Clock Baud Rate */
#define FLEXCOM_SPI_CSR_SCBR_MASK         (0xff << FLEXCOM_SPI_CSR_SCBR_SHIFT)
#  define FLEXCOM_SPI_CSR_SCBR(n)         ((uint32_t)(n) << FLEXCOM_SPI_CSR_SCBR_SHIFT)
#define FLEXCOM_SPI_CSR_DLYBS_SHIFT       (16)      /* Bits 16-23: Delay Before SPCK */
#define FLEXCOM_SPI_CSR_DLYBS_MASK        (0xff << FLEXCOM_SPI_CSR_DLYBS_SHIFT)
#  define FLEXCOM_SPI_CSR_DLYBS(n)        ((uint32_t)(n) << FLEXCOM_SPI_CSR_DLYBS_SHIFT)
#define FLEXCOM_SPI_CSR_DLYBCT_SHIFT      (24)      /* Bits 24-31: Delay Between Consecutive Transfers */
#define FLEXCOM_SPI_CSR_DLYBCT_MASK       (0xff << FLEXCOM_SPI_CSR_DLYBCT_SHIFT)
#  define FLEXCOM_SPI_CSR_DLYBCT(n)       ((uint32_t)(n) << FLEXCOM_SPI_CSR_DLYBCT_SHIFT)

/* SPI FIFO Mode Register */
#define FLEXCOM_SPI_FMR_TXRDYM_SHIFT      (1 << 0)  /* Bits 0-1: TX Data Register empty mode */
#define FLEXCOM_SPI_FMR_TXRDYM_MASK       (3 << FLEXCOM_SPI_FMR_TXRDYM_SHIFT)
#define FLEXCOM_SPI_FMR_RXRDYM_SHIFT      (1 << 4)  /* Bits 0-1: RX Data Register full mode */
#define FLEXCOM_SPI_FMR_RXRDYM_MASK       (3 << FLEXCOM_SPI_FMR_RXRDYM_SHIFT)
#define FLEXCOM_SPI_FMR_TXFTHRES_SHIFT    (1 << 16) /* Bits 16-21: Transmit FIFO threshold */
#define FLEXCOM_SPI_FMR_TXFTHRES_MASK     (0x3f << FLEXCOM_SPI_FMR_TXFTHRES_SHIFT)
#define FLEXCOM_SPI_FMR_RXFTHRES_SHIFT    (1 << 24) /* Bits 24-29: Transmit FIFO threshold */
#define FLEXCOM_SPI_FMR_RXFTHRES_MASK     (0x3f << FLEXCOM_SPI_FMR_RXFTHRES_SHIFT)

/* SPI FIFO Level Register */
#define FLEXCOM_SPI_FLR_TXFL_SHIFT        (1 << 0)  /* Bits 0-5: Transmit FIFO Level */
#define FLEXCOM_SPI_FLR_TXFL_MASK         (0x3f << FLEXCOM_SPI_FLR_TXFL_SHIFT)
#define FLEXCOM_SPI_FLR_RXFL_SHIFT        (16 << 0)  /* Bits 16-21: Transmit FIFO Level */
#define FLEXCOM_SPI_FLR_RXFL_MASK         (0x3f << FLEXCOM_SPI_FLR_RXFL_SHIFT)

/* SPI Comparison Register */
#define FLEXCOM_SPI_CMPR_VAL1_SHIFT       (1 << 0)  /* Bits 0-15: First Comparison Value for Received Character */
#define FLEXCOM_SPI_CMPR_VAL1_MASK        (0xffff << FLEXCOM_SPI_CMPR_VAL1_SHIFT)
#define FLEXCOM_SPI_CMPR_VAL2_SHIFT       (1 << 0)  /* Bits 16-32: Second Comparison Value for Received Character */
#define FLEXCOM_SPI_CMPR_VAL2_MASK        (0xffff << FLEXCOM_SPI_CMPR_VAL2_SHIFT)

/* SPI Write Protection Control Register */

#define FLEXCOM_SPI_WPCR_WPEN             (1 << 0)  /* Bit 0:  SPI Write Protection Enable */
#define FLEXCOM_SPI_WPCR_WPKEY_SHIFT      (8)       /* Bits 8-31: SPI Write Protection Key Password */
#define FLEXCOM_SPI_WPCR_WPKEY_MASK       (0x00ffffff << FLEXCOM_SPI_WPCR_WPKEY_SHIFT)
#  define FLEXCOM_SPI_WPCR_WPKEY          (0x00535049 << FLEXCOM_SPI_WPCR_WPKEY_SHIFT)

/* SPI Write Protection Status Register */

#define FLEXCOM_SPI_WPSR_WPVS             (1 << 0)  /* Bit 0: SPI Write Protection Violation Status */
#define FLEXCOM_SPI_WPSR_WPVSRC_SHIFT     (8)       /* Bits 8-15: SPI Write Protection Violation Source */
#define FLEXCOM_SPI_WPSR_WPVSRC_MASK      (0xff << FLEXCOM_SPI_WPSR_WPVSRC_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_SPI_H */
