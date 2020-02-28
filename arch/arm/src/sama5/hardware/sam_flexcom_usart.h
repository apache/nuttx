/************************************************************************************************
 * arch/arm/src/sama5/hardware/sam3u_uart.h
 * Universal Synchronous Asynchronous Receiver Transmitter (FLEXUS) definitions for the SAMA5D2
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_FLEXUS_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_FLEXUS_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* USART register offsets ***********************************************************************/

#define SAM_FLEXUS_CR_OFFSET           0x0200 /* USART Control Register */
#define SAM_FLEXUS_MR_OFFSET           0x0204 /* USART Mode Register */
#define SAM_FLEXUS_IER_OFFSET          0x0208 /* USART Interrupt Enable Register */
#define SAM_FLEXUS_IDR_OFFSET          0x020c /* USART Interrupt Disable Register */
#define SAM_FLEXUS_IMR_OFFSET          0x0210 /* USART Interrupt Mask Register */
#define SAM_FLEXUS_CSR_OFFSET          0x0214 /* USART Channel Status Register */
#define SAM_FLEXUS_RHR_OFFSET          0x0218 /* USART Receive Holding Register */
#define SAM_FLEXUS_THR_OFFSET          0x021c /* USART Transmit Holding Register */
#define SAM_FLEXUS_BRGR_OFFSET         0x0220 /* USART Baud Rate Generator Register */
#define SAM_FLEXUS_RTOR_OFFSET         0x0224 /* USART Receiver Time-out Register */
#define SAM_FLEXUS_TTGR_OFFSET         0x0228 /* USART Transmitter Timeguard Register */
                                              /* 0x022c-0x023c: Reserved */
#define SAM_FLEXUS_FIDI_OFFSET         0x0240 /* USART FI DI Ratio Register */
#define SAM_FLEXUS_NER_OFFSET          0x0244 /* USART Number of Errors Register */
                                              /* 0x0248: Reserved (FLEXUS) */
#define SAM_FLEXUS_IF_OFFSET           0x024c /* USART IrDA Filter Register */
#define SAM_FLEXUS_MAN_OFFSET          0x0250 /* USART Manchester Configuration Register */
#define SAM_FLEXUS_LINMR_OFFSET        0x0254 /* USART LIN Mode Register */
#define SAM_FLEXUS_LINIR_OFFSET        0x0258 /* USART LIN Identifier Register */
#define SAM_FLEXUS_LINBRR_OFFSET       0x025c /* USART LIN Baud Rate Register */
                                              /* 0x0260-0x028c: Reserved (FLEXUS) */
#define SAM_FLEXUS_CMPR_OFFSET         0x0290 /* USART Comparison Register */
#define SAM_FLEXUS_FMR_OFFSET          0x02a0 /* USART FIFO Mode Register */
#define SAM_FLEXUS_FLR_OFFSET          0x02a4 /* USART FIFO Level Register */
#define SAM_FLEXUS_FIER_OFFSET         0x02a8 /* USART FIFO Interrupt Enable Register */
#define SAM_FLEXUS_FIDR_OFFSET         0x02ac /* USART FIFO Interrupt Disable Register */
#define SAM_FLEXUS_FIMR_OFFSET         0x02b0 /* USART FIFO Interrupt Mask Register */
#define SAM_FLEXUS_FESR_OFFSET         0x02b4 /* USART FIFO Event Status Register */
                                              /* 0x02b8-0x02e: Reserved */
#define SAM_FLEXUS_WPMR_OFFSET         0x02e4 /* Write Protect Mode Register (4) */
#define SAM_FLEXUS_WPSR_OFFSET         0x02e8 /* Write Protect Status Register (4) */
                                              /* 0x02ec-0x02fc: Reserved (USART) */

/* USART register addresses **********************************************************************/

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM0
#  define SAM_FLEXUS0_CR               (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_CR_OFFSET)
#  define SAM_FLEXUS0_MR               (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_MR_OFFSET)
#  define SAM_FLEXUS0_IER              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_IER_OFFSET)
#  define SAM_FLEXUS0_IDR              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_IDR_OFFSET)
#  define SAM_FLEXUS0_IMR              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_IMR_OFFSET)
#  define SAM_FLEXUS0_CSR              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_CSR_OFFSET)
#  define SAM_FLEXUS0_RHR              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_RHR_OFFSET)
#  define SAM_FLEXUS0_THR              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_THR_OFFSET)
#  define SAM_FLEXUS0_BRGR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_BRGR_OFFSET)
#  define SAM_FLEXUS0_RTOR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_RTOR_OFFSET)
#  define SAM_FLEXUS0_TTGR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_TTGR_OFFSET)
#  define SAM_FLEXUS0_FIDI             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_FIDI_OFFSET)
#  define SAM_FLEXUS0_NER              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_NER_OFFSET)
#  define SAM_FLEXUS0_IF               (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_IF_OFFSET)
#  define SAM_FLEXUS0_MAN              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS0_LINMR            (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_LINMR_OFFSET)
#  define SAM_FLEXUS0_LINIR            (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_LINIR_OFFSET)
#  define SAM_FLEXUS0_LINBRR           (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_LINBRR_OFFSET)
#  define SAM_FLEXUS0_CMPR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_CMPR_OFFSET)
#  define SAM_FLEXUS0_FMR              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_FMR_OFFSET)
#  define SAM_FLEXUS0_FLR              (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_FLR_OFFSET)
#  define SAM_FLEXUS0_FIER             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_FIER_OFFSET)
#  define SAM_FLEXUS0_FIDR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_FIDR_OFFSET)
#  define SAM_FLEXUS0_FIMR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_FIMR_OFFSET)
#  define SAM_FLEXUS0_FESR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_FESR_OFFSET)
#  define SAM_FLEXUS0_WPMR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_WPMR_OFFSET)
#  define SAM_FLEXUS0_WPSR             (SAM_FLEXCOM0_VBASE+SAM_FLEXUS_WPSR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM1
#  define SAM_FLEXUS1_CR               (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_CR_OFFSET)
#  define SAM_FLEXUS1_MR               (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_MR_OFFSET)
#  define SAM_FLEXUS1_IER              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_IER_OFFSET)
#  define SAM_FLEXUS1_IDR              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_IDR_OFFSET)
#  define SAM_FLEXUS1_IMR              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_IMR_OFFSET)
#  define SAM_FLEXUS1_CSR              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_CSR_OFFSET)
#  define SAM_FLEXUS1_RHR              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_RHR_OFFSET)
#  define SAM_FLEXUS1_THR              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_THR_OFFSET)
#  define SAM_FLEXUS1_BRGR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_BRGR_OFFSET)
#  define SAM_FLEXUS1_RTOR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_RTOR_OFFSET)
#  define SAM_FLEXUS1_TTGR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_TTGR_OFFSET)
#  define SAM_FLEXUS1_FIDI             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_FIDI_OFFSET)
#  define SAM_FLEXUS1_NER              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_NER_OFFSET)
#  define SAM_FLEXUS1_IF               (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_IF_OFFSET)
#  define SAM_FLEXUS1_MAN              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS1_MAN              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS1_LINMR            (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_LINMR_OFFSET)
#  define SAM_FLEXUS1_LINIR            (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_LINIR_OFFSET)
#  define SAM_FLEXUS1_LINBRR           (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_LINBRR_OFFSET)
#  define SAM_FLEXUS1_CMPR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_CMPR_OFFSET)
#  define SAM_FLEXUS1_FMR              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_FMR_OFFSET)
#  define SAM_FLEXUS1_FLR              (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_FLR_OFFSET)
#  define SAM_FLEXUS1_FIER             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_FIER_OFFSET)
#  define SAM_FLEXUS1_FIDR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_FIDR_OFFSET)
#  define SAM_FLEXUS1_FIMR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_FIMR_OFFSET)
#  define SAM_FLEXUS1_FESR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_FESR_OFFSET)
#  define SAM_FLEXUS1_WPMR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_WPMR_OFFSET)
#  define SAM_FLEXUS1_WPSR             (SAM_FLEXCOM1_VBASE+SAM_FLEXUS_WPSR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM2
#  define SAM_FLEXUS2_CR               (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_CR_OFFSET)
#  define SAM_FLEXUS2_MR               (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_MR_OFFSET)
#  define SAM_FLEXUS2_IER              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_IER_OFFSET)
#  define SAM_FLEXUS2_IDR              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_IDR_OFFSET)
#  define SAM_FLEXUS2_IMR              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_IMR_OFFSET)
#  define SAM_FLEXUS2_CSR              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_CSR_OFFSET)
#  define SAM_FLEXUS2_RHR              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_RHR_OFFSET)
#  define SAM_FLEXUS2_THR              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_THR_OFFSET)
#  define SAM_FLEXUS2_BRGR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_BRGR_OFFSET)
#  define SAM_FLEXUS2_RTOR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_RTOR_OFFSET)
#  define SAM_FLEXUS2_TTGR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_TTGR_OFFSET)
#  define SAM_FLEXUS2_FIDI             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_FIDI_OFFSET)
#  define SAM_FLEXUS2_NER              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_NER_OFFSET)
#  define SAM_FLEXUS2_IF               (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_IF_OFFSET)
#  define SAM_FLEXUS2_MAN              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS2_MAN              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS2_LINMR            (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_LINMR_OFFSET)
#  define SAM_FLEXUS2_LINIR            (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_LINIR_OFFSET)
#  define SAM_FLEXUS2_LINBRR           (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_LINBRR_OFFSET)
#  define SAM_FLEXUS2_CMPR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_CMPR_OFFSET)
#  define SAM_FLEXUS2_FMR              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_FMR_OFFSET)
#  define SAM_FLEXUS2_FLR              (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_FLR_OFFSET)
#  define SAM_FLEXUS2_FIER             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_FIER_OFFSET)
#  define SAM_FLEXUS2_FIDR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_FIDR_OFFSET)
#  define SAM_FLEXUS2_FIMR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_FIMR_OFFSET)
#  define SAM_FLEXUS2_FESR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_FESR_OFFSET)
#  define SAM_FLEXUS2_WPMR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_WPMR_OFFSET)
#  define SAM_FLEXUS2_WPSR             (SAM_FLEXCOM2_VBASE+SAM_FLEXUS_WPSR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM3
#  define SAM_FLEXUS3_CR               (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_CR_OFFSET)
#  define SAM_FLEXUS3_MR               (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_MR_OFFSET)
#  define SAM_FLEXUS3_IER              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_IER_OFFSET)
#  define SAM_FLEXUS3_IDR              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_IDR_OFFSET)
#  define SAM_FLEXUS3_IMR              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_IMR_OFFSET)
#  define SAM_FLEXUS3_CSR              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_CSR_OFFSET)
#  define SAM_FLEXUS3_RHR              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_RHR_OFFSET)
#  define SAM_FLEXUS3_THR              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_THR_OFFSET)
#  define SAM_FLEXUS3_BRGR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_BRGR_OFFSET)
#  define SAM_FLEXUS3_RTOR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_RTOR_OFFSET)
#  define SAM_FLEXUS3_TTGR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_TTGR_OFFSET)
#  define SAM_FLEXUS3_FIDI             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_FIDI_OFFSET)
#  define SAM_FLEXUS3_NER              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_NER_OFFSET)
#  define SAM_FLEXUS3_IF               (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_IF_OFFSET)
#  define SAM_FLEXUS3_MAN              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS3_MAN              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS3_LINMR            (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_LINMR_OFFSET)
#  define SAM_FLEXUS3_LINIR            (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_LINIR_OFFSET)
#  define SAM_FLEXUS3_LINBRR           (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_LINBRR_OFFSET)
#  define SAM_FLEXUS3_CMPR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_CMPR_OFFSET)
#  define SAM_FLEXUS3_FMR              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_FMR_OFFSET)
#  define SAM_FLEXUS3_FLR              (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_FLR_OFFSET)
#  define SAM_FLEXUS3_FIER             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_FIER_OFFSET)
#  define SAM_FLEXUS3_FIDR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_FIDR_OFFSET)
#  define SAM_FLEXUS3_FIMR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_FIMR_OFFSET)
#  define SAM_FLEXUS3_FESR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_FESR_OFFSET)
#  define SAM_FLEXUS3_WPMR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_WPMR_OFFSET)
#  define SAM_FLEXUS3_WPSR             (SAM_FLEXCOM3_VBASE+SAM_FLEXUS_WPSR_OFFSET)
#endif

#ifdef CONFIG_SAMA5_HAVE_FLEXCOM4
#  define SAM_FLEXUS4_CR               (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_CR_OFFSET)
#  define SAM_FLEXUS4_MR               (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_MR_OFFSET)
#  define SAM_FLEXUS4_IER              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_IER_OFFSET)
#  define SAM_FLEXUS4_IDR              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_IDR_OFFSET)
#  define SAM_FLEXUS4_IMR              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_IMR_OFFSET)
#  define SAM_FLEXUS4_CSR              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_CSR_OFFSET)
#  define SAM_FLEXUS4_RHR              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_RHR_OFFSET)
#  define SAM_FLEXUS4_THR              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_THR_OFFSET)
#  define SAM_FLEXUS4_BRGR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_BRGR_OFFSET)
#  define SAM_FLEXUS4_RTOR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_RTOR_OFFSET)
#  define SAM_FLEXUS4_TTGR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_TTGR_OFFSET)
#  define SAM_FLEXUS4_FIDI             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_FIDI_OFFSET)
#  define SAM_FLEXUS4_NER              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_NER_OFFSET)
#  define SAM_FLEXUS4_IF               (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_IF_OFFSET)
#  define SAM_FLEXUS4_MAN              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS4_MAN              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_MAN_OFFSET)
#  define SAM_FLEXUS4_LINMR            (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_LINMR_OFFSET)
#  define SAM_FLEXUS4_LINIR            (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_LINIR_OFFSET)
#  define SAM_FLEXUS4_LINBRR           (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_LINBRR_OFFSET)
#  define SAM_FLEXUS4_CMPR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_CMPR_OFFSET)
#  define SAM_FLEXUS4_FMR              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_FMR_OFFSET)
#  define SAM_FLEXUS4_FLR              (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_FLR_OFFSET)
#  define SAM_FLEXUS4_FIER             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_FIER_OFFSET)
#  define SAM_FLEXUS4_FIDR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_FIDR_OFFSET)
#  define SAM_FLEXUS4_FIMR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_FIMR_OFFSET)
#  define SAM_FLEXUS4_FESR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_FESR_OFFSET)
#  define SAM_FLEXUS4_WPMR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_WPMR_OFFSET)
#  define SAM_FLEXUS4_WPSR             (SAM_FLEXCOM4_VBASE+SAM_FLEXUS_WPSR_OFFSET)
#endif

/* USART register bit definitions ****************************************************************/
/* USART Control Register */

#define FLEXUS_CR_RSTRX                (1 << 2)  /* Bit 2:  Reset Receiver */
#define FLEXUS_CR_RSTTX                (1 << 3)  /* Bit 3:  Reset Transmitter */
#define FLEXUS_CR_RXEN                 (1 << 4)  /* Bit 4:  Receiver Enable */
#define FLEXUS_CR_RXDIS                (1 << 5)  /* Bit 5:  Receiver Disable */
#define FLEXUS_CR_TXEN                 (1 << 6)  /* Bit 6:  Transmitter Enable */
#define FLEXUS_CR_TXDIS                (1 << 7)  /* Bit 7:  Transmitter Disable */
#define FLEXUS_CR_RSTSTA               (1 << 8)  /* Bit 8:  Reset Status Bits */
#define FLEXUS_CR_STTBRK               (1 << 9)  /* Bit 9:  Start Break */
#define FLEXUS_CR_STPBRK               (1 << 10) /* Bit 10: Stop Break */
#define FLEXUS_CR_STTTO                (1 << 11) /* Bit 11: Start Time-out */
#define FLEXUS_CR_SENDA                (1 << 12) /* Bit 12: Send Address */
#define FLEXUS_CR_RSTIT                (1 << 13) /* Bit 13: Reset Iterations */
#define FLEXUS_CR_RSTNACK              (1 << 14) /* Bit 14: Reset Non Acknowledge */
#define FLEXUS_CR_RETTO                (1 << 15) /* Bit 15: Rearm Time-out */
#define FLEXUS_CR_RTSEN                (1 << 18) /* Bit 18: Request to Send Enable */
#define FLEXUS_CR_RTSDIS               (1 << 19) /* Bit 19: Request to Send Disable */
#define FLEXUS_CR_LINABT               (1 << 20) /* Bit 20: Abort LIN Transmission */
#define FLEXUS_CR_LINWKUP              (1 << 21) /* Bit 21: Send LIN Wake-up Signal */
#define FLEXUS_CR_TXFCLR               (1 << 24) /* Bit 24: Transmit FIFO Clear */
#define FLEXUS_CR_RXFCLR               (1 << 25) /* Bit 25: Receive FIFO Clear */
#define FLEXUS_CR_TXFLCLR              (1 << 26) /* Bit 26: Transmit FIFO Lock CLEAR */
#define FLEXUS_CR_REQCLR               (1 << 28) /* Bit 28: Request to Clear the Comparison Trigger */
#define FLEXUS_CR_FIFOEN               (1 << 30) /* Bit 30: FIFO Enable */
#define FLEXUS_CR_FIFODIS              (1 << 31) /* Bit 31: FIFO Disable */

/* USART Mode Register and FLEXUS Mode Register */

#define FLEXUS_MR_MODE_SHIFT           (0)       /* Bits 0-3: USART Mode of Operation */
#define FLEXUS_MR_MODE_MASK            (15 << FLEXUS_MR_MODE_SHIFT)
#  define FLEXUS_MR_MODE_NORMAL        (0  << FLEXUS_MR_MODE_SHIFT) /* Normal */
#  define FLEXUS_MR_MODE_RS485         (1  << FLEXUS_MR_MODE_SHIFT) /* RS485 */
#  define FLEXUS_MR_MODE_HWHS          (2  << FLEXUS_MR_MODE_SHIFT) /* Hardware Handshaking */
#  define FLEXUS_MR_MODE_ISO7816_0     (4  << FLEXUS_MR_MODE_SHIFT) /* IS07816 Protocol: T = 0 */
#  define FLEXUS_MR_MODE_ISO7816_1     (6  << FLEXUS_MR_MODE_SHIFT) /* IS07816 Protocol: T = 1 */
#  define FLEXUS_MR_MODE_IRDA          (8  << FLEXUS_MR_MODE_SHIFT) /* IrDA */
#  define FLEXUS_MR_MODE_LINMASTER     (10 << FLEXUS_MR_MODE_SHIFT) /* LIN master */
#  define FLEXUS_MR_MODE_LINSLAVE      (11 << FLEXUS_MR_MODE_SHIFT) /* LIN slave */
#define FLEXUS_MR_USCLKS_SHIFT         (4)       /* Bits 4-5: Clock Selection */
#define FLEXUS_MR_USCLKS_MASK          (3 << FLEXUS_MR_USCLKS_SHIFT)
#  define FLEXUS_MR_USCLKS_MCK         (0 << FLEXUS_MR_USCLKS_SHIFT) /* MCK */
#  define FLEXUS_MR_USCLKS_MCKDIV      (1 << FLEXUS_MR_USCLKS_SHIFT) /* MCK/DIV (DIV = 8) */
#  define FLEXUS_MR_USCLKS_PMCPCK      (2 << FLEXUS_MR_USCLKS_SHIFT) /* PMC Programmable clock */
#  define FLEXUS_MR_USCLKS_SCK         (3 << FLEXUS_MR_USCLKS_SHIFT) /* ExtenaleSCK */
#define FLEXUS_MR_CHRL_SHIFT           (6)       /* Bits 6-7: Character Length */
#define FLEXUS_MR_CHRL_MASK            (3 << FLEXUS_MR_CHRL_SHIFT)
#  define FLEXUS_MR_CHRL_5BITS         (0 << FLEXUS_MR_CHRL_SHIFT) /* 5 bits */
#  define FLEXUS_MR_CHRL_6BITS         (1 << FLEXUS_MR_CHRL_SHIFT) /* 6 bits */
#  define FLEXUS_MR_CHRL_7BITS         (2 << FLEXUS_MR_CHRL_SHIFT) /* 7 bits */
#  define FLEXUS_MR_CHRL_8BITS         (3 << FLEXUS_MR_CHRL_SHIFT) /* 8 bits */
#define FLEXUS_MR_SYNC                 (1 << 8)  /* Bit 8: Synchronous Mode Select */
#define FLEXUS_MR_PAR_SHIFT            (9)       /* Bits 9-11: Parity Type */
#define FLEXUS_MR_PAR_MASK             (7 << FLEXUS_MR_PAR_SHIFT)
#  define FLEXUS_MR_PAR_EVEN           (0 << FLEXUS_MR_PAR_SHIFT) /* Even parity */
#  define FLEXUS_MR_PAR_ODD            (1 << FLEXUS_MR_PAR_SHIFT) /* Odd parity */
#  define FLEXUS_MR_PAR_SPACE          (2 << FLEXUS_MR_PAR_SHIFT) /* Space: parity forced to 0 */
#  define FLEXUS_MR_PAR_MARK           (3 << FLEXUS_MR_PAR_SHIFT) /* Mark: parity forced to 1 */
#  define FLEXUS_MR_PAR_NONE           (4 << FLEXUS_MR_PAR_SHIFT) /* No parity */
#  define FLEXUS_MR_PAR_MULTIDROP      (6 << FLEXUS_MR_PAR_SHIFT) /* Multidrop mode */
#define FLEXUS_MR_NBSTOP_SHIFT         (12)      /* Bits 12-13: Number of Stop Bits */
#define FLEXUS_MR_NBSTOP_MASK          (3 << FLEXUS_MR_NBSTOP_SHIFT)
#  define FLEXUS_MR_NBSTOP_1           (0 << FLEXUS_MR_NBSTOP_SHIFT) /* 1 stop bit 1 stop bit */
#  define FLEXUS_MR_NBSTOP_1p5         (1 << FLEXUS_MR_NBSTOP_SHIFT) /* 1.5 stop bits */
#  define FLEXUS_MR_NBSTOP_2           (2 << FLEXUS_MR_NBSTOP_SHIFT) /* 2 stop bits 2 stop bits */
#define FLEXUS_MR_CHMODE_SHIFT         (14)      /* Bits 14-15: Channel Mode */
#define FLEXUS_MR_CHMODE_MASK          (3 << FLEXUS_MR_CHMODE_SHIFT)
#  define FLEXUS_MR_CHMODE_NORMAL      (0 << FLEXUS_MR_CHMODE_SHIFT) /* Normal Mode */
#  define FLEXUS_MR_CHMODE_ECHO        (1 << FLEXUS_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define FLEXUS_MR_CHMODE_LLPBK       (2 << FLEXUS_MR_CHMODE_SHIFT) /* Local Loopback */
#  define FLEXUS_MR_CHMODE_RLPBK       (3 << FLEXUS_MR_CHMODE_SHIFT) /* Remote Loopback */
#define FLEXUS_MR_MSBF                 (1 << 16) /* Bit 16: Most Significant Bit first */
#define FLEXUS_MR_MODE9                (1 << 17) /* Bit 17: 9-bit Character Length */
#define FLEXUS_MR_CLKO                 (1 << 18) /* Bit 18: Clock Output Select */
#define FLEXUS_MR_OVER                 (1 << 19) /* Bit 19: Oversampling Mode */
#define FLEXUS_MR_INACK                (1 << 20) /* Bit 20: Inhibit Non Acknowledge */
#define FLEXUS_MR_DSNACK               (1 << 21) /* Bit 21: Disable Successive NACK */
#define FLEXUS_MR_VARSYNC              (1 << 22) /* Bit 22: Variable Synchronization of Command/Data Sync Start Frame Delimiter */
#define FLEXUS_MR_INVDATA              (1 << 23) /* Bit 23: Inverted Data */
#define FLEXUS_MR_MAXITER_SHIFT        (24)      /* Bits 24-26: Max iterations (ISO7816 T=0 */
#define FLEXUS_MR_MAXITER_MASK         (7 << FLEXUS_MR_MAXITER_SHIFT)
#  define FLEXUS_MR_MAXITER(n)         ((uint32_t)(n) << FLEXUS_MR_MAXITER_SHIFT)
#define FLEXUS_MR_FILTER               (1 << 28) /* Bit 28: Infrared Receive Line Filter */
#define FLEXUS_MR_MAN                  (1 << 29) /* Bit 29: Manchester Encoder/Decoder Enable () */
#define FLEXUS_MR_MODSYNC              (1 << 30) /* Bit 30: Manchester Synchronization Mode */
#define FLEXUS_MR_ONEBIT               (1 << 31) /* Bit 31: Start Frame Delimiter Selector */

/* USART Interrupt Enable Register, USART Interrupt Disable Register, USART Interrupt Mask
 * Register, and USART Status Register common bit field definitions
 */

#define FLEXUS_INT_RXRDY               (1 << 0)  /* Bit 0:  RXRDY Interrupt */
#define FLEXUS_INT_TXRDY               (1 << 1)  /* Bit 1:  TXRDY Interrupt */
#define FLEXUS_INT_RXBRK               (1 << 2)  /* Bit 2:  Break Received/End of Break (USART mode) */
#define FLEXUS_INT_OVRE                (1 << 5)  /* Bit 5:  Overrun Error Interrupt */
#define FLEXUS_INT_FRAME               (1 << 6)  /* Bit 6:  Framing Error Interrupt */
#define FLEXUS_INT_PARE                (1 << 7)  /* Bit 7:  Parity Error Interrupt */
#define FLEXUS_INT_TIMEOUT             (1 << 8)  /* Bit 8:  Time-out Interrupt */
#define FLEXUS_INT_TXEMPTY             (1 << 9)  /* Bit 9:  TXEMPTY Interrupt */
#define FLEXUS_INT_ITER                (1 << 10) /* Bit 10: Iteration Interrupt (USART mode) */
#define FLEXUS_INT_NACK                (1 << 13) /* Bit 13: Non Acknowledge Interrupt (USART mode) */
#define FLEXUS_INT_LINBK               (1 << 13) /* Bit 13: LIN Break Sent or LIN Break Received Interrupt (LIN mode) */
#define FLEXUS_INT_LINID               (1 << 14) /* Bit 14: LIN Identifier Sent or LIN Identifier Received Interrupt (LIN mode) */
#define FLEXUS_INT_LINTC               (1 << 15) /* Bit 15: LIN Transfer Completed Interrupt (Lin mode) */
#define FLEXUS_INT_CTSIC               (1 << 19) /* Bit 19: Clear to Send Input Change Interrupt (USART mode) */
#define FLEXUS_INT_CMP                 (1 << 22) /* Bit 22: Comparison Interrupt (USART mode) */
#define FLEXUS_CSR_CTS                 (1 << 23) /* Bit 23: Image of CTS Input (CSR only, UART mode) */
#define FLEXUS_CSR_LINBLS              (1 << 23) /* Bit 23: LIN Bus Line Status (CSR only, LIN mode) */
#define FLEXUS_INT_MANE                (1 << 24) /* Bit 24: Manchester Error Interrupt (USART mode) */
#define FLEXUS_INT_LINBE               (1 << 25) /* Bit 25: LIN Bus Error Interrupt (LIN mode) */
#define FLEXUS_INT_LINISFE             (1 << 26) /* Bit 26: LIN Inconsistent Synch Field Error Interrupt (LIN mode) */
#define FLEXUS_INT_LINIPE              (1 << 27) /* Bit 27: LIN Identifier Parity Interrupt (LIN mode) */
#define FLEXUS_INT_LINCE               (1 << 28) /* Bit 28: LIN Checksum Error Interrupt (LIN mode) */
#define FLEXUS_INT_LINSNRE             (1 << 29) /* Bit 29: LIN Slave Not Responding Error Interrupt (LIN mode) */
#define FLEXUS_INT_LINSTE              (1 << 30) /* Bit 30: LIN Synch Tolerance Error Interrupt (LIN mode) */
#define FLEXUS_INT_LINHTE              (1 << 31) /* Bit 31: LIN Header Timeout Error Interrupt (LIN mode) */

#define FLEXUS_INT_ALLINTS             0xff48e7e7

/* USART Receiver Holding Register */

#define FLEXUS_RHR_RXCHR_SHIFT         (0)       /* Bits 0-8: Received Character */
#define FLEXUS_RHR_RXCHR_MASK          (0x1ff << FLEXUS_RHR_RXCHR_SHIFT)
#  define FLEXUS_RHR_RXCHR(n)          ((uint32_t)(n) << FLEXUS_RHR_RXCHR_SHIFT)
#define FLEXUS_RHR_RXSYNH              (1 << 15) /* Bit 15: Received Sync */

#define FLEXUS_RHR_RXCHR0_SHIFT        (0)       /* Bits 0-7: Received Character */
#define FLEXUS_RHR_RXCHR0_MASK         (0xff << FLEXUS_RHR_RXCHR0_SHIFT)
#  define FLEXUS_RHR_RXCHR0(n)         ((uint32_t)(n) << FLEXUS_RHR_RXCHR0_SHIFT)
#define FLEXUS_RHR_RXCHR1_SHIFT        (8)       /* Bits 8-15: Received Character */
#define FLEXUS_RHR_RXCHR1_MASK         (0xff << FLEXUS_RHR_RXCHR1_SHIFT)
#  define FLEXUS_RHR_RXCHR1(n)         ((uint32_t)(n) << FLEXUS_RHR_RXCHR1_SHIFT)
#define FLEXUS_RHR_RXCHR2_SHIFT        (16)      /* Bits 16-23: Received Character */
#define FLEXUS_RHR_RXCHR2_MASK         (0xff << FLEXUS_RHR_RXCHR2_SHIFT)
#  define FLEXUS_RHR_RXCHR2(n)         ((uint32_t)(n) << FLEXUS_RHR_RXCHR2_SHIFT)
#define FLEXUS_RHR_RXCHR3_SHIFT        (24)      /* Bits 24-31: Received Character */
#define FLEXUS_RHR_RXCHR3_MASK         (0xff << FLEXUS_RHR_RXCHR3_SHIFT)
#  define FLEXUS_RHR_RXCHR3(n)         ((uint32_t)(n) << FLEXUS_RHR_RXCHR3_SHIFT)

/* USART Transmit Holding Register */

#define FLEXUS_THR_TXCHR_SHIFT         (0)       /* Bits 0-8: Character to be transmitted */
#define FLEXUS_THR_TXCHR_MASK          (0x1ff << FLEXUS_THR_TXCHR_SHIFT)
#define FLEXUS_THR_TXSYNH              (1 << 15) /* Bit 15: Sync Field to be tran */

#define FLEXUS_THR_TXCHR0_SHIFT        (0)       /* Bits 0-7: Character to be transmitted */
#define FLEXUS_THR_TXCHR0_MASK         (0xff << FLEXUS_THR_TXCHR0_SHIFT)
#  define FLEXUS_THR_TXCHR0(n)         ((uint32_t)(n) << FLEXUS_THR_TXCHR0_SHIFT)
#define FLEXUS_THR_TXCHR1_SHIFT        (8)       /* Bits 8-15: Character to be transmitted */
#define FLEXUS_THR_TXCHR1_MASK         (0xff << FLEXUS_THR_TXCHR1_SHIFT)
#  define FLEXUS_THR_TXCHR1(n)         ((uint32_t)(n) << FLEXUS_THR_TXCHR1_SHIFT)
#define FLEXUS_THR_TXCHR2_SHIFT        (16)      /* Bits 16-23: Character to be transmitted */
#define FLEXUS_THR_TXCHR2_MASK         (0xff << FLEXUS_THR_TXCHR2_SHIFT)
#  define FLEXUS_THR_TXCHR2(n)         ((uint32_t)(n) << FLEXUS_THR_TXCHR2_SHIFT)
#define FLEXUS_THR_TXCHR3_SHIFT        (24)      /* Bits 24-31: Character to be transmitted */
#define FLEXUS_THR_TXCHR3_MASK         (0xff << FLEXUS_THR_TXCHR3_SHIFT)
#  define FLEXUS_THR_TXCHR3(n)         ((uint32_t)(n) << FLEXUS_THR_TXCHR3_SHIFT)

/* USART Baud Rate Generator Register */

#define FLEXUS_BRGR_CD_SHIFT           (0)       /* Bits 0-15: Clock Divisor */
#define FLEXUS_BRGR_CD_MASK            (0xffff << FLEXUS_BRGR_CD_SHIFT)
#  define FLEXUS_BRGR_CD(n)            ((uint32_t)(n) << FLEXUS_BRGR_CD_SHIFT)
#define FLEXUS_BRGR_FP_SHIFT           (16)      /* Bits 16-18: Fractional Part */
#define FLEXUS_BRGR_FP_MASK            (7 << FLEXUS_BRGR_FP_SHIFT)
#  define FLEXUS_BRGR_FP(n)            ((uint32_t)(n) << FLEXUS_BRGR_FP_SHIFT)

/* FLEXUS Receiver Time-out Register */

#define FLEXUS_RTOR_TO_SHIFT           (0)       /* Bits 0-16: Time-out Value */
#define FLEXUS_RTOR_TO_MASK            (0x1ffff << FLEXUS_RTOR_TO_SHIFT)

/* FLEXUS Transmitter Timeguard Register */

#define FLEXUS_TTGR_TG_SHIFT           (0)       /* Bits 0-7: Timeguard Value */
#define FLEXUS_TTGR_TG_MASK            (0xff << FLEXUS_TTGR_TG_SHIFT)

/* FLEXUS FI DI RATIO Register */

#define FLEXUS_FIDI_RATIO_SHIFT        (0)       /* Bits 0-15: FI Over DI Ratio Value */
#define FLEXUS_FIDI_RATIO_MASK         (0xffff << FLEXUS_FIDI_RATIO_SHIFT)

/* FLEXUS Number of Errors Register */

#define FLEXUS_NER_NBERRORS_SHIFT      (0)       /* Bits 0-7: Number of Errors */
#define FLEXUS_NER_NBERRORS_MASK       (0xff << FLEXUS_NER_NBERRORS_SHIFT)

/* FLEXUS IrDA FILTER Register */

#define FLEXUS_IF_IRDAFILTER_SHIFT     (0)       /* Bits 0-7: IrDA Filter */
#define FLEXUS_IF_IRDAFILTER_MASK      (0xff << FLEXUS_IF_IRDAFILTER_SHIFT)

/* FLEXUS Manchester Configuration Register */

#define FLEXUS_MAN_TXPL_SHIFT          (0)       /* Bits 0-3: Transmitter Preamble Length */
#define FLEXUS_MAN_TXPL_MASK           (15 << FLEXUS_MAN_TXPL_SHIFT)
#  define FLEXUS_MAN_TXPL(n)           ((uint32_t)(n) << FLEXUS_MAN_TXPL_SHIFT)
#define FLEXUS_MAN_TXPP_SHIFT          (8)       /* Bits 8-9: Transmitter Preamble Pattern */
#define FLEXUS_MAN_TXPP_MASK           (3 << FLEXUS_MAN_TXPP_SHIFT)
#  define FLEXUS_MAN_TXPP_ALLONE       (0 << FLEXUS_MAN_TXPP_SHIFT) /* ALL_ONE */
#  define FLEXUS_MAN_TXPP_ALLZERO      (1 << FLEXUS_MAN_TXPP_SHIFT) /* ALL_ZERO */
#  define FLEXUS_MAN_TXPP_ZEROONE      (2 << FLEXUS_MAN_TXPP_SHIFT) /* ZERO_ONE */
#  define FLEXUS_MAN_TXPP_ONEZERO      (3 << FLEXUS_MAN_TXPP_SHIFT) /* ONE_ZERO */
#define FLEXUS_MAN_TXMPOL              (1 << 12) /* Bit 12: Transmitter Manchester Polarity */
#define FLEXUS_MAN_RXPL_SHIFT          (16)      /* Bits 16-19: Receiver Preamble Length */
#define FLEXUS_MAN_RXPL_MASK           (15 << FLEXUS_MAN_RXPL_SHIFT)
#  define FLEXUS_MAN_RXPL(n)           ((uint32_t)(n) << FLEXUS_MAN_RXPL_SHIFT)
#define FLEXUS_MAN_RXPP_SHIFT          (24)      /* Bits 24-25: Receiver Preamble Pattern detected */
#define FLEXUS_MAN_RXPP_MASK           (3 << FLEXUS_MAN_RXPP_SHIFT)
#  define FLEXUS_MAN_RXPP_ALLONE       (0 << FLEXUS_MAN_RXPP_SHIFT) /* ALL_ONE */
#  define FLEXUS_MAN_RXPP_ALLZERO      (1 << FLEXUS_MAN_RXPP_SHIFT) /* ALL_ZERO */
#  define FLEXUS_MAN_RXPP_ZEROONE      (2 << FLEXUS_MAN_RXPP_SHIFT) /* ZERO_ONE */
#  define FLEXUS_MAN_RXPP_ONEZERO      (3 << FLEXUS_MAN_RXPP_SHIFT) /* ONE_ZERO */
#define FLEXUS_MAN_RXMPOL              (1 << 28) /* Bit 28: Receiver Manchester Polarity */
#define FLEXUS_MAN_ONE                 (1 << 29) /* Bit 29: Must Be Set to 1 */
#define FLEXUS_MAN_DRIFT               (1 << 30) /* Bit 30: Drift compensation */
#define FLEXUS_MAN_RXIDLEV             (1 << 31) /* Bit 31: Receiver idle value */

/* USART LIN Mode Register */

#define FLEXUS_LINMR_NACT_SHIFT        (0)       /* Bits 0-1: LIN Node Action */
#define FLEXUS_LINMR_NACT_MASK         (3 << FLEXUS_LINMR_NACT_SHIFT)
#  define FLEXUS_LINMR_NACT_PUBLISH    (0 << FLEXUS_LINMR_NACT_SHIFT) /* USART transmits response */
#  define FLEXUS_LINMR_NACT_SUBSCRIBE  (1 << FLEXUS_LINMR_NACT_SHIFT) /* USART receives response */
#  define FLEXUS_LINMR_NACT_IGNORE     (2 << FLEXUS_LINMR_NACT_SHIFT) /* USART does not transmit or receive response */
#define FLEXUS_LINMR_PARDIS            (1 << 2)  /* Bit 2:  Parity Disable */
#define FLEXUS_LINMR_CHKDIS            (1 << 3)  /* Bit 3:  Checksum Disable */
#define FLEXUS_LINMR_CHKTYP            (1 << 4)  /* Bit 4:  Checksum Type */
#define FLEXUS_LINMR_DLM               (1 << 5)  /* Bit 5:  Data Length Mode */
#define FLEXUS_LINMR_FSDIS             (1 << 6)  /* Bit 6:  Frame Slot Mode Disable */
#define FLEXUS_LINMR_WKUPTYP           (1 << 7)  /* Bit 7:  Wakeup Signal Type */
#define FLEXUS_LINMR_DLC_SHIFT         (8)       /* Bits 8-15: Data Length Control */
#define FLEXUS_LINMR_DLC_MASK          (0xff << FLEXUS_LINMR_DLC_SHIFT)
#  define FLEXUS_LINMR_DLC(n)          ((uint32_t)(n) << FLEXUS_LINMR_DLC_SHIFT)
#define FLEXUS_LINMR_PDCM              (1 << 16) /* Bit 16: DMAC Mode */
#define FLEXUS_LINMR_SYNCDIS           (1 << 17) /* Bit 17: Synchronization Disable */

/* USART LIN Identifier Register */

#define FLEXUS_LINIR_MASK              0xff      /* Bits 0-7: Identifier Character */

/* USART LIN Baud Rate Register */

#define FLEXUS_LINBRR_LINCD_SHIFT      (0)       /* Bits 0-15: Clock Divider after Synchronization */
#define FLEXUS_LINBRR_LINCD_MASK       (0xffff << FLEXUS_LINBRR_LINCD_SHIFT)
#  define FLEXUS_LINBRR_LINCD(n)       ((uint32_t)(n) << FLEXUS_LINBRR_LINCD_SHIFT)
#define FLEXUS_LINBRR_LINFP_SHIFT      (16)      /* Bits 16-18: Fractional Part after Synchronization */
#define FLEXUS_LINBRR_LINFP_MASK       (7 << FLEXUS_LINBRR_LINFP_SHIFT)
#  define FLEXUS_LINBRR_LINFP(n)     ((uint32_t)(n) << FLEXUS_LINBRR_LINFP_SHIFT)

/* USART Comparison Register */

#  define FLEXUS_CMPR_VAL1_SHIFT       (0)       /* Bits 0-8: First Comparison Value for Received Character */
#  define FLEXUS_CMPR_VAL1_MASK        (0x1ff << FLEXUS_CMPR_VAL1_SHIFT)
#    define FLEXUS_CMPR_VAL1(n)        ((uint32_t)(n) << FLEXUS_CMPR_VAL1_SHIFT)
#  define FLEXUS_CMPR_CMPMODE          (1 << 12) /* Bit 12: Comparison Mode */
#    define FLEXUS_CMPR_FLAG           (0)       /* Bit 12: 0=Any character + comparison function */
#    define FLEXUS_CMPR_START          (1 << 12) /* Bit 12: 1=Comparison condition required to start */
#  define FLEXUS_CMPR_CMPPAR           (1 << 14) /* Bit 14: Compare Parity */
#  define FLEXUS_CMPR_VAL2_SHIFT       (16)      /* Bits 16-24: Second Comparison Value for Received Character */
#  define FLEXUS_CMPR_VAL2_MASK        (0x1ff << FLEXUS_CMPR_VAL2_SHIFT)
#    define FLEXUS_CMPR_VAL2(n)        ((uint32_t)(n) << FLEXUS_CMPR_VAL2_SHIFT)

/* USART FIFO Mode Register */

#define FLEXUS_FMR_TXRDYM_SHIFT        (0)       /* Bits 0-1: Transmitter Ready Mode */
#define FLEXUS_FMR_TXRDYM_MASK         (3 << FLEXUS_FMR_TXRDYM_SHIFT)
#  define FLEXUS_FMR_TXRDYM_ONE        (0 << FLEXUS_FMR_TXRDYM_SHIFT) /* TXRDY level 1 when can write one data */
#  define FLEXUS_FMR_TXRDYM_TWO        (1 << FLEXUS_FMR_TXRDYM_SHIFT) /* TXRDY level 1 when can write two data */
#  define FLEXUS_FMR_TXRDYM_FOUR       (2 << FLEXUS_FMR_TXRDYM_SHIFT) /* TXRDY level 1 when can write four data */
#define FLEXUS_FMR_RXRDYM_SHIFT        (4)       /* Bits 4-5: Receiver Ready Mode */
#define FLEXUS_FMR_RXRDYM_MASK         (3 << FLEXUS_FMR_RXRDYM_SHIFT)
#  define FLEXUS_FMR_RXRDYM_ONE        (0 << FLEXUS_FMR_RXRDYM_SHIFT) /* TXRDY level 1 when can read one data */
#  define FLEXUS_FMR_RXRDYM_TWO        (1 << FLEXUS_FMR_RXRDYM_SHIFT) /* TXRDY level 1 when can read two data */
#  define FLEXUS_FMR_RXRDYM_FOUR       (2 << FLEXUS_FMR_RXRDYM_SHIFT) /* TXRDY level 1 when can read four data */
#define FLEXUS_FMR_FRTSC               (1 << 7)  /* Bit 7: FIFO RTS pin Control enable */
#define FLEXUS_FMR_TXFTHRES_SHIFT      (8)       /* Bits 8-13: Transmit FIFO Threshold */
#define FLEXUS_FMR_TXFTHRES_MASK       (0x3f << FLEXUS_FMR_TXFTHRES_SHIFT)
#  define FLEXUS_FMR_TXFTHRES(n)       ((uint32_t)(n) << FLEXUS_FMR_TXFTHRES_SHIFT)
#define FLEXUS_FMR_RXFTHRES_SHIFT      (16)      /* Bits 16-21: Receive FIFO Threshold */
#define FLEXUS_FMR_RXFTHRES_MASK       (0x3f << FLEXUS_FMR_RXFTHRES_SHIFT)
#  define FLEXUS_FMR_RXFTHRES(n)       (0x(uint32_t)(n)3f << FLEXUS_FMR_RXFTHRES_SHIFT)
#define FLEXUS_FMR_RXFTHRES2_SHIFT     (24)      /* Bits 24-29: Receive FIFO Threshold 2 */
#define FLEXUS_FMR_RXFTHRES2_MASK      (0x3f << FLEXUS_FMR_RXFTHRES2_SHIFT)
#  define FLEXUS_FMR_RXFTHRES2(n)      ((uint32_t)(n) << FLEXUS_FMR_RXFTHRES2_SHIFT)

/* USART FIFO Level Register */

#define FLEXUS_FLR_TXFL_SHIFT          (0)       /* Bits 0-5: Transmit FIFO Level */
#define FLEXUS_FLR_TXFL_MASK           (0x3f << FLEXUS_FLR_TXFL_SHIFT)
#  define FLEXUS_FLR_TXFL(n)           ((uint32_t)(n) << FLEXUS_FLR_TXFL_SHIFT)
#define FLEXUS_FLR_RXFL_SHIFT          (16)      /* Bits 0-21: Receive FIFO Level */
#define FLEXUS_FLR_RXFL_MASK           (0x3f << FLEXUS_FLR_RXFL_SHIFT)
#  define FLEXUS_FLR_RXFL(n)           ((uint32_t)(n) << FLEXUS_FLR_RXFL_SHIFT)

/* USART FIFO Interrupt Enable Register,  USART FIFO Interrupt Disable Register, USART FIFO
 * Interrupt Mask Register, and USART FIFO Event Status Register.
 */

#define FLEXUS_FINT_TXFEF              (1 << 0)  /* Bit 0:  Transmit FIFO Empty Interrupt */
#define FLEXUS_FINT_TXFFF              (1 << 1)  /* Bit 1:  Transmit FIFO Full Interrupt */
#define FLEXUS_FINT_TXFTHF             (1 << 2)  /* Bit 2:  Transmit FIFO Threshold Interrupt */
#define FLEXUS_FINT_RXFEF              (1 << 3)  /* Bit 3:  Receive FIFO Empty FInterruptlag */
#define FLEXUS_FINT_RXFFF              (1 << 4)  /* Bit 4:  Receive FIFO Full Interrupt */
#define FLEXUS_FINT_RXFTHF             (1 << 5)  /* Bit 5:  Receive FIFO Threshold Interrupt */
#define FLEXUS_FINT_TXFPTEF            (1 << 6)  /* Bit 6:  Transmit FIFO Pointer Error Interrupt */
#define FLEXUS_FINT_RXFPTEF            (1 << 7)  /* Bit 7:  Receive FIFO Pointer Error Interrupt */
#define FLEXUS_FISR_TXFLOCK            (1 << 8)  /* Bit 8:  Transmit FIFO Lock (FESR only) */
#define FLEXUS_FINT_RXFTHF2            (1 << 9)  /* Bit 9:  Receive FIFO Threshold Interrupt 2 */

/* USART Write Protect Mode Register (USART only) */

#define FLEXUS_WPMR_WPEN               (1 << 0)  /* Bit 0: Write Protect Enable (USART only) */
#define FLEXUS_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY (USART only) */
#define FLEXUS_WPMR_WPKEY_MASK         (0x00ffffff << FLEXUS_WPMR_WPKEY_SHIFT)
#  define FLEXUSWPMR_WPKEY             (0x00555341 << FLEXUS_WPMR_WPKEY_SHIFT) /* "USA" */

/* USART Write Protect Status Register (USART only) */

#define FLEXUS_WPSR_WPVS               (1 << 0)  /* Bit 0: Write Protect Violation Status (USART only) */
#define FLEXUS_WPSR_WPVSRC_SHIFT       (8)       /* Bits 8-23: Write Protect Violation Source (USART only) */
#define FLEXUS_WPSR_WPVSRC_MASK        (0xffff << FLEXUS_WPSR_WPVSRC_SHIFT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_FLEXCOM_FLEXUS_H */
