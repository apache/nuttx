/************************************************************************************
 * arch/arm/src/sama5/hardware/sam_ssc.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SSC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SSC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define SAM_SSC_MAXPERCLK         66000000 /* Maximum peripheral clock frequency */

/* SSC Register Offsets *************************************************************/

#define SAM_SSC_CR_OFFSET         0x0000 /* Control Register */
#define SAM_SSC_CMR_OFFSET        0x0004 /* Clock Mode Register */
                                         /* 0x0008-0x000c Reserved */
#define SAM_SSC_RCMR_OFFSET       0x0010 /* Receive Clock Mode Register */
#define SAM_SSC_RFMR_OFFSET       0x0014 /* Receive Frame Mode Register */
#define SAM_SSC_TCMR_OFFSET       0x0018 /* Transmit Clock Mode Register */
#define SAM_SSC_TFMR_OFFSET       0x001c /* Transmit Frame Mode Register */
#define SAM_SSC_RHR_OFFSET        0x0020 /* Receive Holding Register */
#define SAM_SSC_THR_OFFSET        0x0024 /* Transmit Holding Register */
                                         /* 0x0028-0x002c Reserved */
#define SAM_SSC_RSHR_OFFSET       0x0030 /* Receive Sync. Holding Register */
#define SAM_SSC_TSHR_OFFSET       0x0034 /* Transmit Sync. Holding Register */
#define SAM_SSC_RC0R_OFFSET       0x0038 /* Receive Compare 0 Register */
#define SAM_SSC_RC1R_OFFSET       0x003c /* Receive Compare 1 Register */
#define SAM_SSC_SR_OFFSET         0x0040 /* Status Register */
#define SAM_SSC_IER_OFFSET        0x0044 /* Interrupt Enable Register */
#define SAM_SSC_IDR_OFFSET        0x0048 /* Interrupt Disable Register */
#define SAM_SSC_IMR_OFFSET        0x004c /* Interrupt Mask Register */
#define SAM_SSC_WPMR_OFFSET       0x00e4 /* Write Protect Mode Register */
#define SAM_SSC_WPSR_OFFSET       0x00e8 /* Write Protect Status Register */
                                         /* 0x50-0x124 Reserved */

/* SSC Register Addresses ***********************************************************/

#define SAM_SSC0_CR               (SAM_SSC0_VBASE+SAM_SSC_CR_OFFSET)
#define SAM_SSC0_CMR              (SAM_SSC0_VBASE+SAM_SSC_CMR_OFFSET)
#define SAM_SSC0_RCMR             (SAM_SSC0_VBASE+SAM_SSC_RCMR_OFFSET)
#define SAM_SSC0_RFMR             (SAM_SSC0_VBASE+SAM_SSC_RFMR_OFFSET)
#define SAM_SSC0_TCMR             (SAM_SSC0_VBASE+SAM_SSC_TCMR_OFFSET)
#define SAM_SSC0_TFMR             (SAM_SSC0_VBASE+SAM_SSC_TFMR_OFFSET)
#define SAM_SSC0_RHR              (SAM_SSC0_VBASE+SAM_SSC_RHR_OFFSET)
#define SAM_SSC0_THR              (SAM_SSC0_VBASE+SAM_SSC_THR_OFFSET)
#define SAM_SSC0_RSHR             (SAM_SSC0_VBASE+SAM_SSC_RSHR_OFFSET)
#define SAM_SSC0_TSHR             (SAM_SSC0_VBASE+SAM_SSC_TSHR_OFFSET)
#define SAM_SSC0_RC0R             (SAM_SSC0_VBASE+SAM_SSC_RC0R_OFFSET)
#define SAM_SSC0_RC1R             (SAM_SSC0_VBASE+SAM_SSC_RC1R_OFFSET)
#define SAM_SSC0_SR               (SAM_SSC0_VBASE+SAM_SSC_SR_OFFSET)
#define SAM_SSC0_IER              (SAM_SSC0_VBASE+SAM_SSC_IER_OFFSET)
#define SAM_SSC0_IDR              (SAM_SSC0_VBASE+SAM_SSC_IDR_OFFSET)
#define SAM_SSC0_IMR              (SAM_SSC0_VBASE+SAM_SSC_IMR_OFFSET)
#define SAM_SSC0_WPMR             (SAM_SSC0_VBASE+SAM_SSC_WPMR_OFFSET)
#define SAM_SSC0_WPSR             (SAM_SSC0_VBASE+SAM_SSC_WPSR_OFFSET)

#define SAM_SSC1_CR               (SAM_SSC1_VBASE+SAM_SSC_CR_OFFSET)
#define SAM_SSC1_CMR              (SAM_SSC1_VBASE+SAM_SSC_CMR_OFFSET)
#define SAM_SSC1_RCMR             (SAM_SSC1_VBASE+SAM_SSC_RCMR_OFFSET)
#define SAM_SSC1_RFMR             (SAM_SSC1_VBASE+SAM_SSC_RFMR_OFFSET)
#define SAM_SSC1_TCMR             (SAM_SSC1_VBASE+SAM_SSC_TCMR_OFFSET)
#define SAM_SSC1_TFMR             (SAM_SSC1_VBASE+SAM_SSC_TFMR_OFFSET)
#define SAM_SSC1_RHR              (SAM_SSC1_VBASE+SAM_SSC_RHR_OFFSET)
#define SAM_SSC1_THR              (SAM_SSC1_VBASE+SAM_SSC_THR_OFFSET)
#define SAM_SSC1_RSHR             (SAM_SSC1_VBASE+SAM_SSC_RSHR_OFFSET)
#define SAM_SSC1_TSHR             (SAM_SSC1_VBASE+SAM_SSC_TSHR_OFFSET)
#define SAM_SSC1_RC0R             (SAM_SSC1_VBASE+SAM_SSC_RC0R_OFFSET)
#define SAM_SSC1_RC1R             (SAM_SSC1_VBASE+SAM_SSC_RC1R_OFFSET)
#define SAM_SSC1_SR               (SAM_SSC1_VBASE+SAM_SSC_SR_OFFSET)
#define SAM_SSC1_IER              (SAM_SSC1_VBASE+SAM_SSC_IER_OFFSET)
#define SAM_SSC1_IDR              (SAM_SSC1_VBASE+SAM_SSC_IDR_OFFSET)
#define SAM_SSC1_IMR              (SAM_SSC1_VBASE+SAM_SSC_IMR_OFFSET)
#define SAM_SSC1_WPMR             (SAM_SSC1_VBASE+SAM_SSC_WPMR_OFFSET)
#define SAM_SSC1_WPSR             (SAM_SSC1_VBASE+SAM_SSC_WPSR_OFFSET)

/* SSC Register Bit Definitions *****************************************************/

/* Control Register */

#define SSC_CR_RXEN               (1 << 0)  /* Bit 0:  Receive Enable */
#define SSC_CR_RXDIS              (1 << 1)  /* Bit 1:  Receive Disable */
#define SSC_CR_TXEN               (1 << 8)  /* Bit 8:  Transmit Enable */
#define SSC_CR_TXDIS              (1 << 9)  /* Bit 9:  Transmit Disable */
#define SSC_CR_SWRST              (1 << 15) /* Bit 15: Software Reset */

/* Clock Mode Register */

#define SSC_CMR_DIV_MASK          (0x00000fff) /* Bits 0-11: DIV: Clock Divider */

/* Receive Clock Mode Register */

#define SSC_RCMR_CKS_SHIFT        (0)       /* Bits 0-1: Receive Clock Selection */
#define SSC_RCMR_CKS_MASK         (3 << SSC_RCMR_CKS_SHIFT)
#  define SSC_RCMR_CKS_MCK        (0 << SSC_RCMR_CKS_SHIFT) /* Divided Clock */
#  define SSC_RCMR_CKS_TK         (1 << SSC_RCMR_CKS_SHIFT) /* TK Clock signal */
#  define SSC_RCMR_CKS_RK         (2 << SSC_RCMR_CKS_SHIFT) /* RK pin */
#define SSC_RCMR_CKO_SHIFT        (2)       /* Bits 2-4: Receive Clock Output Mode Selection */
#define SSC_RCMR_CKO_MASK         (7 << SSC_RCMR_CKO_SHIFT)
#  define SSC_RCMR_CKO_NONE       (0 << SSC_RCMR_CKO_SHIFT) /* None, RK pin is an input */
#  define SSC_RCMR_CKO_CONT       (1 << SSC_RCMR_CKO_SHIFT) /* Continuous Receive Clock, RK pin is an output */
#  define SSC_RCMR_CKO_TRANSFER   (2 << SSC_RCMR_CKO_SHIFT) /* Receive Clock during transfers, RK pin is an output */
#define SSC_RCMR_CKI              (1 << 5)  /* Bit 5:  Receive Clock Inversion */
#define SSC_RCMR_CKG_SHIFT        (6)       /* Bits 6-7: Receive Clock Gating Selection */
#define SSC_RCMR_CKG_MASK         (3 << SSC_RCMR_CKG_SHIFT)
#  define SSC_RCMR_CKG_CONT       (0 << SSC_RCMR_CKG_SHIFT) /* None */
#  define SSC_RCMR_CKG_ENRFLOW    (2 << SSC_RCMR_CKG_SHIFT) /* Receive Clock enabled only if RF Pin is Low */
#  define SSC_RCMR_CKG_ENRFHIGH   (3 << SSC_RCMR_CKG_SHIFT) /* Receive Clock enabled only if RF Pin is High */
#define SSC_RCMR_START_SHIFT      (8)       /* Bits 8-11: Receive Start Selection */
#define SSC_RCMR_START_MASK       (15 << SSC_RCMR_START_SHIFT)
#  define SSC_RCMR_START_CONT     (0 << SSC_RCMR_START_SHIFT) /* Continuous */
#  define SSC_RCMR_START_TRANSMIT (1 << SSC_RCMR_START_SHIFT) /* Transmit start */
#  define SSC_RCMR_START_LOW      (2 << SSC_RCMR_START_SHIFT) /* Detection of a low level on RF signal */
#  define SSC_RCMR_START_HIGH     (3 << SSC_RCMR_START_SHIFT) /* Detection of a high level on RF signal */
#  define SSC_RCMR_START_FALLING  (4 << SSC_RCMR_START_SHIFT) /* Detection of a falling edge on RF signal */
#  define SSC_RCMR_START_RISING   (5 << SSC_RCMR_START_SHIFT) /* Detection of a rising edge on RF signal */
#  define SSC_RCMR_START_LEVEL    (6 << SSC_RCMR_START_SHIFT) /* Detection of any level change on RF signal */
#  define SSC_RCMR_START_EDGE     (7 << SSC_RCMR_START_SHIFT) /* Detection of any edge on RF signal */
#  define SSC_RCMR_START_CMP0     (8 << SSC_RCMR_START_SHIFT) /* Compare 0 */
#define SSC_RCMR_STOP             (1 << 12) /* Bit 12: Receive Stop Selection */
#define SSC_RCMR_STTDLY_SHIFT     (16)      /* Bits 16-23: Receive Start Delay */
#define SSC_RCMR_STTDLY_MASK      (0xff << SSC_RCMR_STTDLY_SHIFT)
#  define SSC_RCMR_STTDLY(n)      ((uint32_t)(n) << SSC_RCMR_STTDLY_SHIFT)
#define SSC_RCMR_PERIOD_SHIFT     (24)       /* Bits 24-31: Receive Period Divider Selection */
#define SSC_RCMR_PERIOD_MASK      (0xff << SSC_RCMR_PERIOD_SHIFT)
#  define SSC_RCMR_PERIOD(n)      ((uint32_t)(n) << SSC_RCMR_PERIOD_SHIFT)

/* Receive Frame Mode Register */

#define SSC_RFMR_DATLEN_SHIFT     (0)       /* Bits 0-4: Data Length */
#define SSC_RFMR_DATLEN_MASK      (15 << SSC_RFMR_DATLEN_SHIFT)
#  define SSC_RFMR_DATLEN(n)      ((uint32_t)(n) << SSC_RFMR_DATLEN_SHIFT)
#define SSC_RFMR_LOOP             (1 << 5)  /* Bit 5:  Loop Mode */
#define SSC_RFMR_MSBF             (1 << 7)  /* Bit 7:  Most Significant Bit First */
#define SSC_RFMR_DATNB_SHIFT      (8)       /* Bits 8-11: Data Number per Frame */
#define SSC_RFMR_DATNB_MASK       (15 << SSC_RFMR_DATNB_SHIFT)
#  define SSC_RFMR_DATNB(n)       ((uint32_t)(n) << SSC_RFMR_DATNB_SHIFT)
#define SSC_RFMR_FSLEN_SHIFT      (16)      /* Bits 16-19: Receive Frame Sync Length */
#define SSC_RFMR_FSLEN_MASK       (15 << SSC_RFMR_FSLEN_SHIFT)
#  define SSC_RFMR_FSLEN(n)       ((uint32_t)(n) << SSC_RFMR_FSLEN_SHIFT)
#define SSC_RFMR_FSOS_SHIFT       (20)      /* Bits 20-22: Receive Frame Sync Output Selection */
#define SSC_RFMR_FSOS_MASK        (7 << SSC_RFMR_FSOS_SHIFT)
#  define SSC_RFMR_FSOS_NONE      (0 << SSC_RFMR_FSOS_SHIFT) /* None, RF pin is an input */
#  define SSC_RFMR_FSOS_NEGATIVE  (1 << SSC_RFMR_FSOS_SHIFT) /* Negative Pulse, RF pin is an output */
#  define SSC_RFMR_FSOS_POSITIVE  (2 << SSC_RFMR_FSOS_SHIFT) /* Positive Pulse, RF pin is an output */
#  define SSC_RFMR_FSOS_LOW       (3 << SSC_RFMR_FSOS_SHIFT) /* Low during transfer, RF pin is an output */
#  define SSC_RFMR_FSOS_HIGH      (4 << SSC_RFMR_FSOS_SHIFT) /* High during transfer, RF pin is an output */
#  define SSC_RFMR_FSOS_TOGGLING  (5 << SSC_RFMR_FSOS_SHIFT) /* Toggling each transfer, RF pin is an output */
#define SSC_RFMR_FSEDGE           (1 << 24) /* Bit 24: Frame Sync Edge Detection */
#  define SSC_RFMR_FSEDGE_POS     (0)       /* Bit 24: 0=Positive Edge Detection */
#  define SSC_RFMR_FSEDGE_NEG     (1 << 24) /* Bit 24: 1=Negative Edge Detection */
#define SSC_RFMR_FSLENEXT_SHIFT   (28)      /* Bits 28-31: FSLEN Field Extension */
#define SSC_RFMR_FSLENEXT_MASK    (15 << SSC_RFMR_FSLENEXT_SHIFT)
#  define SSC_RFMR_FSLENEXT(n)    ((uint32_t)(n) << SSC_RFMR_FSLENEXT_SHIFT)

/* Transmit Clock Mode Register */

#define SSC_TCMR_CKS_SHIFT        (0)       /* Bits 0-1: Transmit Clock Selection */
#define SSC_TCMR_CKS_MASK         (3 << SSC_TCMR_CKS_SHIFT)
#  define SSC_TCMR_CKS_MCK        (0 << SSC_TCMR_CKS_SHIFT) /* Divided Clock */
#  define SSC_TCMR_CKS_RK         (1 << SSC_TCMR_CKS_SHIFT) /* RK Clock signal */
#  define SSC_TCMR_CKS_TK         (2 << SSC_TCMR_CKS_SHIFT) /* TK pin */
#define SSC_TCMR_CKO_SHIFT        (2)       /* Bits 2-4: Transmit Clock Output Mode Selection */
#define SSC_TCMR_CKO_MASK         (7 << SSC_TCMR_CKO_SHIFT)
#  define SSC_TCMR_CKO_NONE       (0 << SSC_TCMR_CKO_SHIFT) /* None, TK pin is an input */
#  define SSC_TCMR_CKO_CONT       (1 << SSC_TCMR_CKO_SHIFT) /* Continuous Transmit Clock, TK pin is an output */
#  define SSC_TCMR_CKO_TRANSFER   (2 << SSC_TCMR_CKO_SHIFT) /* Transmit Clock during transfers, TK pin is an output */
#define SSC_TCMR_CKI              (1 << 5)  /* Bit 5:  Transmit Clock Inversion */
#define SSC_TCMR_CKG_SHIFT        (6)       /* Bits 6-7: Transmit Clock Gating Selection */
#define SSC_TCMR_CKG_MASK         (3 << SSC_TCMR_CKG_SHIFT)
#  define SSC_TCMR_CKG_CONT       (0 << SSC_TCMR_CKG_SHIFT) /* None */
#  define SSC_TCMR_CKG_ENTFLOW    (1 << SSC_TCMR_CKG_SHIFT) /* Transmit Clock enabled only if TF pin is Low */
#  define SSC_TCMR_CKG_ENTFHIGH   (2 << SSC_TCMR_CKG_SHIFT) /*Transmit Clock enabled only if TF pin is High */
#define SSC_TCMR_START_SHIFT      (8)       /* Bits 8-11: Transmit Start Selection */
#define SSC_TCMR_START_MASK       (15 << SSC_TCMR_START_SHIFT)
#  define SSC_TCMR_START_CONT     (0 << SSC_TCMR_START_SHIFT) /* Continuous */
#  define SSC_TCMR_START_RECEIVE  (1 << SSC_TCMR_START_SHIFT) /* Receive start */
#  define SSC_TCMR_START_LOW      (2 << SSC_TCMR_START_SHIFT) /* Detection of a low level on TF signal */
#  define SSC_TCMR_START_HIGH     (3 << SSC_TCMR_START_SHIFT) /* Detection of a high level on TF signal */
#  define SSC_TCMR_START_FALLING  (4 << SSC_TCMR_START_SHIFT) /* Detection of a falling edge on TF signal */
#  define SSC_TCMR_START_RISING   (5 << SSC_TCMR_START_SHIFT) /* Detection of a rising edge on TF signal */
#  define SSC_TCMR_START_LEVEL    (6 << SSC_TCMR_START_SHIFT) /* Detection of any level change on TF signal */
#  define SSC_TCMR_START_EDGE     (7 << SSC_TCMR_START_SHIFT) /* Detection of any edge on TF signal */
#define SSC_TCMR_STTDLY_SHIFT     (16)       /* Bits 15-23: Transmit Start Delay */
#define SSC_TCMR_STTDLY_MASK      (0xff << SSC_TCMR_STTDLY_SHIFT)
#  define SSC_TCMR_STTDLY(n)      ((uint32_t)(n) << SSC_TCMR_STTDLY_SHIFT)
#define SSC_TCMR_PERIOD_SHIFT     (24)       /* Bits 24-31: Transmit Period Divider Selection */
#define SSC_TCMR_PERIOD_MASK      (0xff << SSC_TCMR_PERIOD_SHIFT)
#  define SSC_TCMR_PERIOD(n)      ((uint32_t)(n) << SSC_TCMR_PERIOD_SHIFT)

/* Transmit Frame Mode Register */

#define SSC_TFMR_DATLEN_SHIFT     (0)       /* Bits 0-4: Data Length */
#define SSC_TFMR_DATLEN_MASK      (31 << SSC_TFMR_DATLEN_SHIFT)
#  define SSC_TFMR_DATLEN(n)      ((uint32_t)(n) << SSC_TFMR_DATLEN_SHIFT)
#define SSC_TFMR_DATDEF           (1 << 5)  /* Bit 5:  Data Default Value */
#define SSC_TFMR_MSBF             (1 << 7)  /* Bit 7:  Most Significant Bit First */
#define SSC_TFMR_DATNB_SHIFT      (8)       /* Bits 8-11: Data Number per frame */
#define SSC_TFMR_DATNB_MASK       (15 << SSC_TFMR_DATNB_SHIFT)
#  define SSC_TFMR_DATNB(n)       ((uint32_t)(n) << SSC_TFMR_DATNB_SHIFT)
#define SSC_TFMR_FSLEN_SHIFT      (16)      /* Bits 16-19: Transmit Frame Sync Length */
#define SSC_TFMR_FSLEN_MASK       (15 << SSC_TFMR_FSLEN_SHIFT)
#  define SSC_TFMR_FSLEN(n)       ((uint32_t)(n) << SSC_TFMR_FSLEN_SHIFT)
#define SSC_TFMR_FSOS_SHIFT       (20)      /* Bits 20-22: Transmit Frame Sync Output Selection */
#define SSC_TFMR_FSOS_MASK        (7 << SSC_TFMR_FSOS_SHIFT)
#  define SSC_TFMR_FSOS_NONE      (0 << SSC_TFMR_FSOS_SHIFT) /* None, TF pin is an input */
#  define SSC_TFMR_FSOS_NEGATIVE  (1 << SSC_TFMR_FSOS_SHIFT) /* Negative Pulse, TF pin is an output */
#  define SSC_TFMR_FSOS_POSITIVE  (2 << SSC_TFMR_FSOS_SHIFT) /* Positive Pulse, TF pin is an output */
#  define SSC_TFMR_FSOS_LOW       (3 << SSC_TFMR_FSOS_SHIFT) /* TF pin Driven Low during data transfer */
#  define SSC_TFMR_FSOS_HIGH      (4 << SSC_TFMR_FSOS_SHIFT) /* TF pin Driven High during data transfer */
#  define SSC_TFMR_FSOS_TOGGLING  (5 << SSC_TFMR_FSOS_SHIFT) /* TF pin Toggles at each start of data transfer */
#define SSC_TFMR_FSDEN            (1 << 23) /* Bit 23: Frame Sync Data Enable */
#define SSC_TFMR_FSEDGE           (1 << 24) /* Bit 24: Frame Sync Edge Detection */
#  define SSC_TFMR_FSEDGE_POS     (0)       /* Bit 24: 0=Positive Edge Detection */
#  define SSC_TFMR_FSEDGE_NEG     (1 << 24) /* Bit 24: 1=Negative Edge Detection */
#define SSC_TFMR_FSLENEXT_SHIFT   (28)      /* Bits 28-31: FSLEN Field Extension */
#define SSC_TFMR_FSLENEXT_MASK    (15 << SSC_TFMR_FSLENEXT_SHIFT)
#  define SSC_TFMR_FSLENEXT(n)    ((uint32_t)(n) << SSC_TFMR_FSLENEXT_SHIFT)

/* Receive Holding Register (32-bit data value) */
/* Transmit Holding Register (32-bit data value) */

/* Receive Sync. Holding Register */

#define SSC_RSHR_MASK             (0x0000ffff) /* Bit 0-15: Receive Synchronization Data */

/* Transmit Sync. Holding Register */

#define SSC_TSHR_MASK             (0x0000ffff) /* Bit 0-15: Transmit Synchronization Data */

/* Receive Compare 0 Register */

#define SSC_RC0R_MASK             (0x0000ffff) /* Bit 0-15: Receive Compare Data 0 */

/* Receive Compare 1 Register */

#define SSC_RC1R_MASK             (0x0000ffff) /* Bit 0-15: Receive Compare Data 1 */

/* Status Register , Interrupt Enable Register, Interrupt Disable Register, and
 * Interrupt Mask Register
 */

#define SSC_INT_TXRDY             (1 << 0)  /* Bit 0:  Transmit Ready */
#define SSC_INT_TXEMPTY           (1 << 1)  /* Bit 1:  Transmit Empty */
#define SSC_INT_RXRDY             (1 << 4)  /* Bit 4:  Receive Ready */
#define SSC_INT_OVRUN             (1 << 5)  /* Bit 5:  Receive Overrun */
#define SSC_INT_CP0               (1 << 8)  /* Bit 8:  Compare 0 */
#define SSC_INT_CP1               (1 << 9)  /* Bit 9:  Compare 1 */
#define SSC_INT_TXSYN             (1 << 10) /* Bit 10: Transmit Sync */
#define SSC_INT_RXSYN             (1 << 11) /* Bit 11: Receive Sync */

#define SSC_SR_TXEN               (1 << 16) /* Bit 16: Transmit Enable (SR only) */
#define SSC_SR_RXEN               (1 << 17) /* Bit 17: Receive Enable (SR only) */

/* Write Protect Mode Register */

#define SSC_WPMR_WPEN             (1 << 0)  /* Bit 0:  Write Protect Enable */
#define SSC_WPMR_WPKEY_SHIFT      (8)       /* Bits 8-31: Write Protect KEY */
#define SSC_WPMR_WPKEY_MASK       (0x00ffffff << SSC_WPMR_WPKEY_SHIFT)
#  define SSC_WPMR_WPKEY          (0x00535343 << SSC_WPMR_WPKEY_SHIFT) /* "SSC" in ASCII */

/* Write Protect Status Register */

#define SSC_WPSR_WPVS              (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define SSC_WPSR_WPVSRC_SHIFT      (8)       /* Bits 8-23: Write Protect Violation Source */
#define SSC_WPSR_WPVSRC_MASK       (0xffff << SSC_WPSR_WPVSRC_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SSC_H */
