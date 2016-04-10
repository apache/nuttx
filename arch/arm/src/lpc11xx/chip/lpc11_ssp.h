/************************************************************************************
 * arch/arm/src/lpc11xx/chip/lpc11_ssp.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_SSP_H
#define __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_SSP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc11_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC11_SSP_CR0_OFFSET   0x0000  /* Control Register 0 */
#define LPC11_SSP_CR1_OFFSET   0x0004  /* Control Register 1 */
#define LPC11_SSP_DR_OFFSET    0x0008  /* Data Register */
#define LPC11_SSP_SR_OFFSET    0x000c  /* Status Register */
#define LPC11_SSP_CPSR_OFFSET  0x0010  /* Clock Prescale Register */
#define LPC11_SSP_IMSC_OFFSET  0x0014  /* Interrupt Mask Set/Clear Register */
#define LPC11_SSP_RIS_OFFSET   0x0018  /* Raw Interrupt Status Register */
#define LPC11_SSP_MIS_OFFSET   0x001c  /* Masked Interrupt Status Register */
#define LPC11_SSP_ICR_OFFSET   0x0020  /* Interrupt Clear Register */

/* Register addresses ***************************************************************/
/* SPI 0 */
#define LPC11_SSP0_CR0         (LPC11_SPI0_BASE+LPC11_SSP_CR0_OFFSET)
#define LPC11_SSP0_CR1         (LPC11_SPI0_BASE+LPC11_SSP_CR1_OFFSET)
#define LPC11_SSP0_DR          (LPC11_SPI0_BASE+LPC11_SSP_DR_OFFSET)
#define LPC11_SSP0_SR          (LPC11_SPI0_BASE+LPC11_SSP_SR_OFFSET)
#define LPC11_SSP0_CPSR        (LPC11_SPI0_BASE+LPC11_SSP_CPSR_OFFSET)
#define LPC11_SSP0_IMSC        (LPC11_SPI0_BASE+LPC11_SSP_IMSC_OFFSET)
#define LPC11_SSP0_RIS         (LPC11_SPI0_BASE+LPC11_SSP_RIS_OFFSET)
#define LPC11_SSP0_MIS         (LPC11_SPI0_BASE+LPC11_SSP_MIS_OFFSET)
#define LPC11_SSP0_ICR         (LPC11_SPI0_BASE+LPC11_SSP_ICR_OFFSET)

/* SPI 1 */
#define LPC11_SSP1_CR0         (LPC11_SPI1_BASE+LPC11_SSP_CR0_OFFSET)
#define LPC11_SSP1_CR1         (LPC11_SPI1_BASE+LPC11_SSP_CR1_OFFSET)
#define LPC11_SSP1_DR          (LPC11_SPI1_BASE+LPC11_SSP_DR_OFFSET)
#define LPC11_SSP1_SR          (LPC11_SPI1_BASE+LPC11_SSP_SR_OFFSET)
#define LPC11_SSP1_CPSR        (LPC11_SPI1_BASE+LPC11_SSP_CPSR_OFFSET)
#define LPC11_SSP1_IMSC        (LPC11_SPI1_BASE+LPC11_SSP_IMSC_OFFSET)
#define LPC11_SSP1_RIS         (LPC11_SPI1_BASE+LPC11_SSP_RIS_OFFSET)
#define LPC11_SSP1_MIS         (LPC11_SPI1_BASE+LPC11_SSP_MIS_OFFSET)
#define LPC11_SSP1_ICR         (LPC11_SPI1_BASE+LPC11_SSP_ICR_OFFSET)

/* Register bit definitions *********************************************************/

/* SPI/SSP Control Register 0 */

#define SSP_CR0_DSS_SHIFT      (0)  /* Data Size Select */
#define SSP_CR0_DSS_MASK       (15 << SSP_CR0_SHIFT)
#  define SSP_CR0_DSS_4BITS    (3 << SSP_CR0_DSS_SHIFT)  /* 4 bits per transfer */
#  define SSP_CR0_DSS_5BITS    (4 << SSP_CR0_DSS_SHIFT)  /* 5 bits per transfer */
#  define SSP_CR0_DSS_6BITS    (5 << SSP_CR0_DSS_SHIFT)  /* 6 bits per transfer */
#  define SSP_CR0_DSS_7BITS    (6 << SSP_CR0_DSS_SHIFT)  /* 7 bits per transfer */
#  define SSP_CR0_DSS_8BITS    (7 << SSP_CR0_DSS_SHIFT)  /* 8 bits per transfer */
#  define SSP_CR0_DSS_9BITS    (8 << SSP_CR0_DSS_SHIFT)  /* 9 bits per transfer */
#  define SSP_CR0_DSS_10BITS   (9 << SSP_CR0_DSS_SHIFT)  /* 10 bits per transfer */
#  define SSP_CR0_DSS_11BITS   (10 << SSP_CR0_DSS_SHIFT)  /* 11 bits per transfer */
#  define SSP_CR0_DSS_12BITS   (11 << SSP_CR0_DSS_SHIFT)  /* 12 bits per transfer */
#  define SSP_CR0_DSS_13BITS   (12 << SSP_CR0_DSS_SHIFT)  /* 13 bits per transfer */
#  define SSP_CR0_DSS_14BITS   (13 << SSP_CR0_DSS_SHIFT)  /* 14 bits per transfer */
#  define SSP_CR0_DSS_15BITS   (14 << SSP_CR0_DSS_SHIFT)  /* 15 bits per transfer */
#  define SSP_CR0_DSS_16BITS   (15 << SSP_CR0_DSS_SHIFT)  /* 16 bits per transfer */
#define SSP_CR0_FRF_SHIFT      (4)  /* Frame Format */
#define SSP_CR0_FRF_MASK       (3 << SSP_CR0_FRF_SHIFT)
#  define SSP_CR0_FRF_SPI      (0 << SSP_CR0_FRF_SHIFT)  /* SPI Frame Format */
#  define SSP_CR0_FRF_TI       (1 << SSP_CR0_FRF_SHIFT)  /* TI Frame Format */
#  define SSP_CR0_FRF_MWIRE    (2 << SSP_CR0_FRF_SHIFT)  /* Microwire Frame Format */
                               /* (3 << SSP_CR0_FRF_SHIFT) format is not supported */
#define SSP_CR0_CPOL           (1 << 6)  /* Bit 6:  Clock polarity control */
#define SSP_CR0_CPHA           (1 << 7)  /* Bit 7:  Clock phase control */
#define SSP_CR0_SCR_SHIFT      (8)  /* Bit 8-15: Serial Clock Rate. PCLK/(CPSDVSR x [SCR + 1] */
#define SSP_CR0_SCR_MASK       (255 << SSP_CR0_SCR_SHIFT)

/* SPI/SSP Control Register 1 */

#define SSP_CR1_LBM            (1 << 0)  /* Bit 0: Loop Back Mode */
#define SSP_CR1_SSE            (1 << 1)  /* Bit 1: SPI Enable */
#define SSP_CR1_MS             (1 << 2)  /* Bit 2: Master/Slave Mode */
#define SSP_CR1_SOD            (1 << 3)  /* Bit 3: Slave Output Disable */
                                         /* Bits 4-31: Reserved */

/* SPI/SSP Data Register */

#define SSP_DR_MASK             (0xffff)  /* Bits 0-15: Data */
                                          /* Bits 16-31: Reserved */
/* SPI/SSP Status Register */

#define SSP_SR_TFE              (1 << 0)  /* Bit 0:  Transmit FIFO Empty */
#define SSP_SR_TNF              (1 << 1)  /* Bit 1:  Transmit FIFO Not Full */
#define SSP_SR_RNE              (1 << 2)  /* Bit 2:  Receive FIFO Not Empty */
#define SSP_SR_RFF              (1 << 3)  /* Bit 3:  Receive FIFO Full */
#define SSP_SR_BSY              (1 << 4)  /* Bit 4:  Busy */
                                          /* Bits 5-31: Reserved */
/* SPI/SSP Clock Prescale Register */

#define SSP_CPSR_DVSR_MASK      (0xff)   /* Even values between 2 and 254 */

/* SPI/SSP Interrupt Mask Set/Clear Register */

#define SSP_IMSC_RORIM          (1 << 0) /* Bit 0: Enable Receive Overrun Interrupt */
#define SSP_IMSC_RTIM           (1 << 1) /* Bit 1: Enable Receive Timeout Interrupt */
#define SSP_IMSC_RXIM           (1 << 2) /* Bit 2: Enable Rx FIFO half full Interrupt */
#define SSP_IMSC_TXIM           (1 << 3) /* Bit 3: Enable Tx FIFO halt empty */
                                         /* Bits 4-31: Reserved */

/* SPI/SSP Raw Interrupt Status */

#define SSP_RIS_RORIS           (1 << 0) /* Bit 0: An Overrun event occurred */
#define SSP_RIS_RTRIS           (1 << 1) /* Bit 1: Rx FIFO has data and MCU didn't read it */
#define SSP_RIS_RXRIS           (1 << 2) /* Bit 2: The Rx FIFO is at least half full */
#define SSP_RIS_TXRIS           (1 << 3) /* Bit 3: Tx FIFO is at least halt empty */
                                         /* Bits 4-31: Reserved */

/* SPI/SSP Masked Interrupt Status Register */

#define SSP_MIS_RORMIS          (1 << 0) /* Bit 0: An Overrun occurred and this interrupt is enabled */
#define SSP_MIS_RTMIS           (1 << 1) /* Bit 1: An Rx FIFO timeout happened and this int is enabled */
#define SSP_MIS_RXMIS           (1 << 2) /* Bit 2: Rx FIFO is at least half empty and this int is enabled */
#define SSP_MIS_TXMIS           (1 << 3) /* Bit 3: Tx FIFO is at least halt full and this int is enabled */
                                         /* Bits 4-31: Reserved */
/* SPI/SSP Interrupt Clear Register */

#define SSP_ICR_RORIC           (1 << 0)  /* Bit 0: Clear Rx FIFO Overrun Interrupt */
#define SSP_ICR_RTIC            (1 << 1)  /* Bit 1: Clear Rx FIFO read timeout Interrupt */
                                          /* Bits 2-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_SPI_H */
