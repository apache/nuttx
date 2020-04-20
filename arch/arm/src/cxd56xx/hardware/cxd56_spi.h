/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_spi.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPI_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 8 frame FIFOs for both transmit and receive */

#define CXD56_SPI_FIFOSZ        8

/* Register offsets *********************************************************/

#define CXD56_SPI_CR0_OFFSET       0x0000 /* Control Register 0 */
#define CXD56_SPI_CR1_OFFSET       0x0004 /* Control Register 1 */
#define CXD56_SPI_DR_OFFSET        0x0008 /* Data Register */
#define CXD56_SPI_SR_OFFSET        0x000c /* Status Register */
#define CXD56_SPI_CPSR_OFFSET      0x0010 /* Clock Prescale Register */
#define CXD56_SPI_IMSC_OFFSET      0x0014 /* Interrupt Mask Set and Clear Reg */
#define CXD56_SPI_RIS_OFFSET       0x0018 /* Raw Interrupt Status Register */
#define CXD56_SPI_MIS_OFFSET       0x001c /* Masked Interrupt Status Register */
#define CXD56_SPI_ICR_OFFSET       0x0020 /* Interrupt Clear Register */
#define CXD56_SPI_DMACR_OFFSET     0x0024 /* DMA Control Register */
#define CXD56_SPI_CSMODE_OFFSET    0x0090 /* CS control mode */
#define CXD56_SPI_CS_OFFSET        0x0094 /* CS output */
#define CXD56_SPI_SLAVETYPE_OFFSET 0x0098 /* Slave type */

/* Register addresses *******************************************************/

#define CXD56_SPI4_CR0          (CXD56_IMG_SPI_BASE+CXD56_SPI_CR0_OFFSET)
#define CXD56_SPI4_CR1          (CXD56_IMG_SPI_BASE+CXD56_SPI_CR1_OFFSET)
#define CXD56_SPI4_DR           (CXD56_IMG_SPI_BASE+CXD56_SPI_DR_OFFSET)
#define CXD56_SPI4_SR           (CXD56_IMG_SPI_BASE+CXD56_SPI_SR_OFFSET)
#define CXD56_SPI4_CPSR         (CXD56_IMG_SPI_BASE+CXD56_SPI_CPSR_OFFSET)
#define CXD56_SPI4_IMSC         (CXD56_IMG_SPI_BASE+CXD56_SPI_IMSC_OFFSET)
#define CXD56_SPI4_RIS          (CXD56_IMG_SPI_BASE+CXD56_SPI_RIS_OFFSET)
#define CXD56_SPI4_MIS          (CXD56_IMG_SPI_BASE+CXD56_SPI_MIS_OFFSET)
#define CXD56_SPI4_ICR          (CXD56_IMG_SPI_BASE+CXD56_SPI_ICR_OFFSET)
#define CXD56_SPI4_DMACR        (CXD56_IMG_SPI_BASE+CXD56_SPI_DMACR_OFFSET)

#define CXD56_SPI5_CR0          (CXD56_IMG_WSPI_BASE+CXD56_SPI_CR0_OFFSET)
#define CXD56_SPI5_CR1          (CXD56_IMG_WSPI_BASE+CXD56_SPI_CR1_OFFSET)
#define CXD56_SPI5_DR           (CXD56_IMG_WSPI_BASE+CXD56_SPI_DR_OFFSET)
#define CXD56_SPI5_SR           (CXD56_IMG_WSPI_BASE+CXD56_SPI_SR_OFFSET)
#define CXD56_SPI5_CPSR         (CXD56_IMG_WSPI_BASE+CXD56_SPI_CPSR_OFFSET)
#define CXD56_SPI5_IMSC         (CXD56_IMG_WSPI_BASE+CXD56_SPI_IMSC_OFFSET)
#define CXD56_SPI5_RIS          (CXD56_IMG_WSPI_BASE+CXD56_SPI_RIS_OFFSET)
#define CXD56_SPI5_MIS          (CXD56_IMG_WSPI_BASE+CXD56_SPI_MIS_OFFSET)
#define CXD56_SPI5_ICR          (CXD56_IMG_WSPI_BASE+CXD56_SPI_ICR_OFFSET)
#define CXD56_SPI5_DMACR        (CXD56_IMG_WSPI_BASE+CXD56_SPI_DMACR_OFFSET)

#define CXD56_SPI0_CR0          (CXD56_SPIM_BASE+CXD56_SPI_CR0_OFFSET)
#define CXD56_SPI0_CR1          (CXD56_SPIM_BASE+CXD56_SPI_CR1_OFFSET)
#define CXD56_SPI0_DR           (CXD56_SPIM_BASE+CXD56_SPI_DR_OFFSET)
#define CXD56_SPI0_SR           (CXD56_SPIM_BASE+CXD56_SPI_SR_OFFSET)
#define CXD56_SPI0_CPSR         (CXD56_SPIM_BASE+CXD56_SPI_CPSR_OFFSET)
#define CXD56_SPI0_IMSC         (CXD56_SPIM_BASE+CXD56_SPI_IMSC_OFFSET)
#define CXD56_SPI0_RIS          (CXD56_SPIM_BASE+CXD56_SPI_RIS_OFFSET)
#define CXD56_SPI0_MIS          (CXD56_SPIM_BASE+CXD56_SPI_MIS_OFFSET)
#define CXD56_SPI0_ICR          (CXD56_SPIM_BASE+CXD56_SPI_ICR_OFFSET)
#define CXD56_SPI0_DMACR        (CXD56_SPIM_BASE+CXD56_SPI_DMACR_OFFSET)
#define CXD56_SPI0_CSMODE       (CXD56_SPIM_BASE+CXD56_SPI_CSMODE_OFFSET)
#define CXD56_SPI0_CS           (CXD56_SPIM_BASE+CXD56_SPI_CS_OFFSET)
#define CXD56_SPI0_SLAVETYPE    (CXD56_SPIM_BASE+CXD56_SPI_SLAVETYPE_OFFSET)

#define CXD56_SPI3_CR0          (CXD56_SCU_SPI_BASE+CXD56_SPI_CR0_OFFSET)
#define CXD56_SPI3_CR1          (CXD56_SCU_SPI_BASE+CXD56_SPI_CR1_OFFSET)
#define CXD56_SPI3_DR           (CXD56_SCU_SPI_BASE+CXD56_SPI_DR_OFFSET)
#define CXD56_SPI3_SR           (CXD56_SCU_SPI_BASE+CXD56_SPI_SR_OFFSET)
#define CXD56_SPI3_CPSR         (CXD56_SCU_SPI_BASE+CXD56_SPI_CPSR_OFFSET)
#define CXD56_SPI3_IMSC         (CXD56_SCU_SPI_BASE+CXD56_SPI_IMSC_OFFSET)
#define CXD56_SPI3_RIS          (CXD56_SCU_SPI_BASE+CXD56_SPI_RIS_OFFSET)
#define CXD56_SPI3_MIS          (CXD56_SCU_SPI_BASE+CXD56_SPI_MIS_OFFSET)
#define CXD56_SPI3_ICR          (CXD56_SCU_SPI_BASE+CXD56_SPI_ICR_OFFSET)
#define CXD56_SPI3_DMACR        (CXD56_SCU_SPI_BASE+CXD56_SPI_DMACR_OFFSET)
#define CXD56_SPI3_CSMODE       (CXD56_SCU_SPI_BASE+CXD56_SPI_CSMODE_OFFSET)
#define CXD56_SPI3_CS           (CXD56_SCU_SPI_BASE+CXD56_SPI_CS_OFFSET)
#define CXD56_SPI3_SLAVETYPE    (CXD56_SCU_SPI_BASE+CXD56_SPI_SLAVETYPE_OFFSET)

/* Register bit definitions *************************************************/

/* Control Register 0 */

#define SPI_CR0_DSS_SHIFT       (0)       /* Bits 0-3: DSS Data Size Select */
#define SPI_CR0_DSS_MASK        (15 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_4BIT        (3 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_5BIT        (4 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_6BIT        (5 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_7BIT        (6 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_8BIT        (7 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_9BIT        (8 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_10BIT       (9 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_11BIT       (10 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_12BIT       (11 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_13BIT       (12 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_14BIT       (13 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_15BIT       (14 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_DSS_16BIT       (15 << SPI_CR0_DSS_SHIFT)
#define SPI_CR0_FRF_SHIFT       (4)       /* Bits 4-5: FRF Frame Format */
#define SPI_CR0_FRF_MASK        (3 << SPI_CR0_FRF_SHIFT)
#define SPI_CR0_FRF_SPI         (0 << SPI_CR0_FRF_SHIFT)
#define SPI_CR0_FRF_TI          (1 << SPI_CR0_FRF_SHIFT)
#define SPI_CR0_FRF_UWIRE       (2 << SPI_CR0_FRF_SHIFT)
#define SPI_CR0_CPOL            (1 << 6)  /* Bit 6:  Clock Out Polarity */
#define SPI_CR0_CPHA            (1 << 7)  /* Bit 7:  Clock Out Phase */
#define SPI_CR0_SCR_SHIFT       (8)       /* Bits 8-15: Serial Clock Rate */
#define SPI_CR0_SCR_MASK        (0xff << SPI_CR0_SCR_SHIFT)
                                          /* Bits 8-31: Reserved */

/* Control Register 1 */

#define SPI_CR1_LBM             (1 << 0)  /* Bit 0:  Loop Back Mode */
#define SPI_CR1_SSE             (1 << 1)  /* Bit 1:  SPI Enable */
#define SPI_CR1_MS              (1 << 2)  /* Bit 2:  Master/Slave Mode */
#define SPI_CR1_SOD             (1 << 3)  /* Bit 3:  Slave Output Disable */
                                          /* Bits 4-31: Reserved */

/* Data Register */

#define SPI_DR_MASK             (0xffff)  /* Bits 0-15: Data */
                                          /* Bits 16-31: Reserved */

/* Status Register */

#define SPI_SR_TFE              (1 << 0)  /* Bit 0:  Transmit FIFO Empty */
#define SPI_SR_TNF              (1 << 1)  /* Bit 1:  Transmit FIFO Not Full */
#define SPI_SR_RNE              (1 << 2)  /* Bit 2:  Receive FIFO Not Empty */
#define SPI_SR_RFF              (1 << 3)  /* Bit 3:  Receive FIFO Full */
#define SPI_SR_BSY              (1 << 4)  /* Bit 4:  Busy */
                                          /* Bits 5-31: Reserved */

/* Clock Prescale Register */

#define SPI_CPSR_DVSR_MASK      (0xff)    /* Bits 0-7: clock = SPI_PCLK/DVSR */
                                          /* Bits 8-31: Reserved */

/* Common format for interrupt control registers:
 *
 *   Interrupt Mask Set and Clear Register (IMSC)
 *   Raw Interrupt Status Register (RIS)
 *   Masked Interrupt Status Register (MIS)
 *   Interrupt Clear Register (ICR)
 */

#define SPI_INT_ROR             (1 << 0)  /* Bit 0: RX FIFO overrun */
#define SPI_INT_RT              (1 << 1)  /* Bit 1: RX FIFO timeout */
#define SPI_INT_RX              (1 << 2)  /* Bit 2: RX FIFO at least half full */
#define SPI_INT_TX              (1 << 3 ) /* Bit 3: TX FIFO at least half empty */
                                          /* Bits 4-31: Reserved */

/* DMA Control Register */

#define SPI_DMACR_RXDMAE        (1 << 0)  /* Bit 0:  Receive DMA Enable */
#define SPI_DMACR_TXDMAE        (1 << 1)  /* Bit 1:  Transmit DMA Enable */
                                          /* Bits 2-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPI_H */
