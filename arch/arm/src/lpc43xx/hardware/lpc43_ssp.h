/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_ssp.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SSP_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SSP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 8 frame FIFOs for both transmit and receive */

#define LPC43_SSP_FIFOSZ        8

/* Register offsets *********************************************************/

#define LPC43_SSP_CR0_OFFSET    0x0000 /* Control Register 0 */
#define LPC43_SSP_CR1_OFFSET    0x0004 /* Control Register 1 */
#define LPC43_SSP_DR_OFFSET     0x0008 /* Data Register */
#define LPC43_SSP_SR_OFFSET     0x000c /* Status Register */
#define LPC43_SSP_CPSR_OFFSET   0x0010 /* Clock Prescale Register */
#define LPC43_SSP_IMSC_OFFSET   0x0014 /* Interrupt Mask Set and Clear Register */
#define LPC43_SSP_RIS_OFFSET    0x0018 /* Raw Interrupt Status Register */
#define LPC43_SSP_MIS_OFFSET    0x001c /* Masked Interrupt Status Register */
#define LPC43_SSP_ICR_OFFSET    0x0020 /* Interrupt Clear Register */
#define LPC43_SSP_DMACR_OFFSET  0x0024 /* DMA Control Register */

/* Register addresses *******************************************************/

#define LPC43_SSP0_CR0          (LPC43_SSP0_BASE+LPC43_SSP_CR0_OFFSET)
#define LPC43_SSP0_CR1          (LPC43_SSP0_BASE+LPC43_SSP_CR1_OFFSET)
#define LPC43_SSP0_DR           (LPC43_SSP0_BASE+LPC43_SSP_DR_OFFSET)
#define LPC43_SSP0_SR           (LPC43_SSP0_BASE+LPC43_SSP_SR_OFFSET)
#define LPC43_SSP0_CPSR         (LPC43_SSP0_BASE+LPC43_SSP_CPSR_OFFSET)
#define LPC43_SSP0_IMSC         (LPC43_SSP0_BASE+LPC43_SSP_IMSC_OFFSET)
#define LPC43_SSP0_RIS          (LPC43_SSP0_BASE+LPC43_SSP_RIS_OFFSET)
#define LPC43_SSP0_MIS          (LPC43_SSP0_BASE+LPC43_SSP_MIS_OFFSET)
#define LPC43_SSP0_ICR          (LPC43_SSP0_BASE+LPC43_SSP_ICR_OFFSET)
#define LPC43_SSP0_DMACR        (LPC43_SSP0_BASE+LPC43_SSP_DMACR_OFFSET)

#define LPC43_SSP1_CR0          (LPC43_SSP1_BASE+LPC43_SSP_CR0_OFFSET)
#define LPC43_SSP1_CR1          (LPC43_SSP1_BASE+LPC43_SSP_CR1_OFFSET)
#define LPC43_SSP1_DR           (LPC43_SSP1_BASE+LPC43_SSP_DR_OFFSET)
#define LPC43_SSP1_SR           (LPC43_SSP1_BASE+LPC43_SSP_SR_OFFSET)
#define LPC43_SSP1_CPSR         (LPC43_SSP1_BASE+LPC43_SSP_CPSR_OFFSET)
#define LPC43_SSP1_IMSC         (LPC43_SSP1_BASE+LPC43_SSP_IMSC_OFFSET)
#define LPC43_SSP1_RIS          (LPC43_SSP1_BASE+LPC43_SSP_RIS_OFFSET)
#define LPC43_SSP1_MIS          (LPC43_SSP1_BASE+LPC43_SSP_MIS_OFFSET)
#define LPC43_SSP1_ICR          (LPC43_SSP1_BASE+LPC43_SSP_ICR_OFFSET)
#define LPC43_SSP1_DMACR        (LPC43_SSP1_BASE+LPC43_SSP_DMACR_OFFSET)

/* Register bit definitions *************************************************/

/* Control Register 0 */

#define SSP_CR0_DSS_SHIFT       (0)       /* Bits 0-3: DSS Data Size Select */
#define SSP_CR0_DSS_MASK        (15 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_4BIT      (3 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_5BIT      (4 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_6BIT      (5 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_7BIT      (6 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_8BIT      (7 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_9BIT      (8 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_10BIT     (9 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_11BIT     (10 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_12BIT     (11 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_13BIT     (12 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_14BIT     (13 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_15BIT     (14 << SSP_CR0_DSS_SHIFT)
#  define SSP_CR0_DSS_16BIT     (15 << SSP_CR0_DSS_SHIFT)
#define SSP_CR0_FRF_SHIFT       (4)       /* Bits 4-5: FRF Frame Format */
#define SSP_CR0_FRF_MASK        (3 << SSP_CR0_FRF_SHIFT)
#  define SSP_CR0_FRF_SPI       (0 << SSP_CR0_FRF_SHIFT)
#  define SSP_CR0_FRF_TI        (1 << SSP_CR0_FRF_SHIFT)
#  define SSP_CR0_FRF_UWIRE     (2 << SSP_CR0_FRF_SHIFT)
#define SSP_CR0_CPOL            (1 << 6)  /* Bit 6:  Clock Out Polarity */
#define SSP_CR0_CPHA            (1 << 7)  /* Bit 7:  Clock Out Phase */
#define SSP_CR0_SCR_SHIFT       (8)       /* Bits 8-15: Serial Clock Rate */
#define SSP_CR0_SCR_MASK        (0xff << SSP_CR0_SCR_SHIFT)
                                          /* Bits 8-31: Reserved */

/* Control Register 1 */

#define SSP_CR1_LBM             (1 << 0)  /* Bit 0:  Loop Back Mode */
#define SSP_CR1_SSE             (1 << 1)  /* Bit 1:  SSP Enable */
#define SSP_CR1_MS              (1 << 2)  /* Bit 2:  Master/Slave Mode */
#define SSP_CR1_SOD             (1 << 3)  /* Bit 3:  Slave Output Disable */
                                          /* Bits 4-31: Reserved */

/* Data Register */

#define SSP_DR_MASK             (0xffff)  /* Bits 0-15: Data */
                                          /* Bits 16-31: Reserved */

/* Status Register */

#define SSP_SR_TFE              (1 << 0)  /* Bit 0:  Transmit FIFO Empty */
#define SSP_SR_TNF              (1 << 1)  /* Bit 1:  Transmit FIFO Not Full */
#define SSP_SR_RNE              (1 << 2)  /* Bit 2:  Receive FIFO Not Empty */
#define SSP_SR_RFF              (1 << 3)  /* Bit 3:  Receive FIFO Full */
#define SSP_SR_BSY              (1 << 4)  /* Bit 4:  Busy */
                                          /* Bits 5-31: Reserved */

/* Clock Prescale Register */

#define SSP_CPSR_DVSR_MASK      (0xff)    /* Bits 0-7: clock = SSP_PCLK/DVSR */
                                          /* Bits 8-31: Reserved */

/* Common format for interrupt control registers:
 *
 *   Interrupt Mask Set and Clear Register (IMSC)
 *   Raw Interrupt Status Register (RIS)
 *   Masked Interrupt Status Register (MIS)
 *   Interrupt Clear Register (ICR)
 */

#define SSP_INT_ROR             (1 << 0)  /* Bit 0: RX FIFO overrun */
#define SSP_INT_RT              (1 << 1)  /* Bit 1: RX FIFO timeout */
#define SSP_INT_RX              (1 << 2)  /* Bit 2: RX FIFO at least half full (not ICR) */
#define SSP_INT_TX              (1 << 3 ) /* Bit 3: TX FIFO at least half empty (not ICR) */
                                          /* Bits 4-31: Reserved */

/* DMA Control Register */

#define SSP_DMACR_RXDMAE        (1 << 0)  /* Bit 0:  Receive DMA Enable */
#define SSP_DMACR_TXDMAE        (1 << 1)  /* Bit 1:  Transmit DMA Enable */
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

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_SSP_H */
