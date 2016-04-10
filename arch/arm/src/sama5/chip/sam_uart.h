/************************************************************************************************
 * arch/arm/src/sama5/chip/sam_uart.h
 * Universal Asynchronous Receiver Transmitter (UART) for the SAMA5D2, SAMA5D3, and SAMA5D4 and
 * Universal Synchronous Asynchronous Receiver Transmitter (USART) definitions for the SAMA5D3
 * and SAMAD4
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

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_UART_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_UART_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* UART register offsets ************************************************************************/
/* Key:
 *   (1) Common to both UART and SAMA5D3/4 USART
 *   (2) SAMA5D3/4 USART only
 *   (3) SAMA5D2 UART only
 *   (4) SAMA5D3/4 USART and SAMA5D2 UART only.
 */

#define SAM_UART_CR_OFFSET           0x0000 /* Control Register (1) */
#define SAM_UART_MR_OFFSET           0x0004 /* Mode Register (1) */
#define SAM_UART_IER_OFFSET          0x0008 /* Interrupt Enable Register (1) */
#define SAM_UART_IDR_OFFSET          0x000c /* Interrupt Disable Register (1) */
#define SAM_UART_IMR_OFFSET          0x0010 /* Interrupt Mask Register (1) */
#define SAM_UART_SR_OFFSET           0x0014 /* [Channel] Status Register (1) */
#define SAM_UART_RHR_OFFSET          0x0018 /* Receive Holding Register (1) */
#define SAM_UART_THR_OFFSET          0x001c /* Transmit Holding Register (1) */
#define SAM_UART_BRGR_OFFSET         0x0020 /* Baud Rate Generator Register (1) */
                                            /* 0x0024-0x003c: Reserved (UART) */

#if defined(ATSAMA5D2)
#  define SAM_UART_CMPR_OFFSET       0x0024 /* Comparison Register (3) */
#  define SAM_UART_RTOR_OFFSET       0x0028 /* Receiver Time-out Register (4) */
#elif defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define SAM_UART_RTOR_OFFSET       0x0024 /* Receiver Time-out Register (4) */
#  define SAM_UART_TTGR_OFFSET       0x0028 /* Transmitter Timeguard Register (2) */
                                            /* 0x002c-0x003c: Reserved */
#  define SAM_UART_FIDI_OFFSET       0x0040 /* FI DI Ratio Register (2) */
#  define SAM_UART_NER_OFFSET        0x0044 /* Number of Errors Register (2) */
                                            /* 0x0048: Reserved (USART) */
#  define SAM_UART_IFR_OFFSET        0x004c /* IrDA Filter Register (2) */
#  define SAM_UART_MAN_OFFSET        0x0050 /* Manchester Encoder Decoder Register (2) */
#endif

#define SAM_UART_WPMR_OFFSET         0x00e4 /* Write Protect Mode Register (4) */

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define SAM_UART_WPSR_OFFSET       0x00e8 /* Write Protect Status Register (4) */
#endif
                                            /* 0x00ec-0xfc: Reserved (USART) */

/* UART register adresses ***********************************************************************/

#define SAM_UART0_CR                 (SAM_UART0_VBASE+SAM_UART_CR_OFFSET)
#define SAM_UART0_MR                 (SAM_UART0_VBASE+SAM_UART_MR_OFFSET)
#define SAM_UART0_IER                (SAM_UART0_VBASE+SAM_UART_IER_OFFSET)
#define SAM_UART0_IDR                (SAM_UART0_VBASE+SAM_UART_IDR_OFFSET)
#define SAM_UART0_IMR                (SAM_UART0_VBASE+SAM_UART_IMR_OFFSET)
#define SAM_UART0_SR                 (SAM_UART0_VBASE+SAM_UART_SR_OFFSET)
#define SAM_UART0_RHR                (SAM_UART0_VBASE+SAM_UART_RHR_OFFSET)
#define SAM_UART0_THR                (SAM_UART0_VBASE+SAM_UART_THR_OFFSET)
#define SAM_UART0_BRGR               (SAM_UART0_VBASE+SAM_UART_BRGR_OFFSET)
#if defined(ATSAMA5D2)
#  define SAM_UART0_CMPR             (SAM_UART0_VBASE+SAM_UART_CMPR_OFFSET)
#  define SAM_UART0_RTOR             (SAM_UART0_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_UART0_WPMR             (SAM_UART0_VBASE+SAM_UART_WPMR_OFFSET)
#endif

#define SAM_UART1_CR                 (SAM_UART1_VBASE+SAM_UART_CR_OFFSET)
#define SAM_UART1_MR                 (SAM_UART1_VBASE+SAM_UART_MR_OFFSET)
#define SAM_UART1_IER                (SAM_UART1_VBASE+SAM_UART_IER_OFFSET)
#define SAM_UART1_IDR                (SAM_UART1_VBASE+SAM_UART_IDR_OFFSET)
#define SAM_UART1_IMR                (SAM_UART1_VBASE+SAM_UART_IMR_OFFSET)
#define SAM_UART1_SR                 (SAM_UART1_VBASE+SAM_UART_SR_OFFSET)
#define SAM_UART1_RHR                (SAM_UART1_VBASE+SAM_UART_RHR_OFFSET)
#define SAM_UART1_THR                (SAM_UART1_VBASE+SAM_UART_THR_OFFSET)
#define SAM_UART1_BRGR               (SAM_UART1_VBASE+SAM_UART_BRGR_OFFSET)
#if defined(ATSAMA5D2)
#  define SAM_UART1_CMPR             (SAM_UART1_VBASE+SAM_UART_CMPR_OFFSET)
#  define SAM_UART1_RTOR             (SAM_UART1_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_UART1_WPMR             (SAM_UART1_VBASE+SAM_UART_WPMR_OFFSET)
#endif

#if defined(ATSAMA5D2)
#  define SAM_UART2_CR               (SAM_UART2_VBASE+SAM_UART_CR_OFFSET)
#  define SAM_UART2_MR               (SAM_UART2_VBASE+SAM_UART_MR_OFFSET)
#  define SAM_UART2_IER              (SAM_UART2_VBASE+SAM_UART_IER_OFFSET)
#  define SAM_UART2_IDR              (SAM_UART2_VBASE+SAM_UART_IDR_OFFSET)
#  define SAM_UART2_IMR              (SAM_UART2_VBASE+SAM_UART_IMR_OFFSET)
#  define SAM_UART2_SR               (SAM_UART2_VBASE+SAM_UART_SR_OFFSET)
#  define SAM_UART2_RHR              (SAM_UART2_VBASE+SAM_UART_RHR_OFFSET)
#  define SAM_UART2_THR              (SAM_UART2_VBASE+SAM_UART_THR_OFFSET)
#  define SAM_UART2_BRGR             (SAM_UART2_VBASE+SAM_UART_BRGR_OFFSET)
#  define SAM_UART2_CMPR             (SAM_UART2_VBASE+SAM_UART_CMPR_OFFSET)
#  define SAM_UART2_RTOR             (SAM_UART2_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_UART2_WPMR             (SAM_UART2_VBASE+SAM_UART_WPMR_OFFSET)

#  define SAM_UART3_CR               (SAM_UART3_VBASE+SAM_UART_CR_OFFSET)
#  define SAM_UART3_MR               (SAM_UART3_VBASE+SAM_UART_MR_OFFSET)
#  define SAM_UART3_IER              (SAM_UART3_VBASE+SAM_UART_IER_OFFSET)
#  define SAM_UART3_IDR              (SAM_UART3_VBASE+SAM_UART_IDR_OFFSET)
#  define SAM_UART3_IMR              (SAM_UART3_VBASE+SAM_UART_IMR_OFFSET)
#  define SAM_UART3_SR               (SAM_UART3_VBASE+SAM_UART_SR_OFFSET)
#  define SAM_UART3_RHR              (SAM_UART3_VBASE+SAM_UART_RHR_OFFSET)
#  define SAM_UART3_THR              (SAM_UART3_VBASE+SAM_UART_THR_OFFSET)
#  define SAM_UART3_BRGR             (SAM_UART3_VBASE+SAM_UART_BRGR_OFFSET)
#  define SAM_UART3_CMPR             (SAM_UART3_VBASE+SAM_UART_CMPR_OFFSET)
#  define SAM_UART3_RTOR             (SAM_UART3_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_UART3_WPMR             (SAM_UART3_VBASE+SAM_UART_WPMR_OFFSET)

#  define SAM_UART4_CR               (SAM_UART4_VBASE+SAM_UART_CR_OFFSET)
#  define SAM_UART4_MR               (SAM_UART4_VBASE+SAM_UART_MR_OFFSET)
#  define SAM_UART4_IER              (SAM_UART4_VBASE+SAM_UART_IER_OFFSET)
#  define SAM_UART4_IDR              (SAM_UART4_VBASE+SAM_UART_IDR_OFFSET)
#  define SAM_UART4_IMR              (SAM_UART4_VBASE+SAM_UART_IMR_OFFSET)
#  define SAM_UART4_SR               (SAM_UART4_VBASE+SAM_UART_SR_OFFSET)
#  define SAM_UART4_RHR              (SAM_UART4_VBASE+SAM_UART_RHR_OFFSET)
#  define SAM_UART4_THR              (SAM_UART4_VBASE+SAM_UART_THR_OFFSET)
#  define SAM_UART4_BRGR             (SAM_UART4_VBASE+SAM_UART_BRGR_OFFSET)
#  define SAM_UART4_CMPR             (SAM_UART4_VBASE+SAM_UART_CMPR_OFFSET)
#  define SAM_UART4_RTOR             (SAM_UART4_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_UART4_WPMR             (SAM_UART4_VBASE+SAM_UART_WPMR_OFFSET)
#endif

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define SAM_USART_CR(n)            (SAM_USARTN_VBASE(n)+SAM_UART_CR_OFFSET)
#  define SAM_USART_MR(n)            (SAM_USARTN_VBASE(n)+SAM_UART_MR_OFFSET)
#  define SAM_USART_IER(n)           (SAM_USARTN_VBASE(n)+SAM_UART_IER_OFFSET)
#  define SAM_USART_IDR(n)           (SAM_USARTN_VBASE(n)+SAM_UART_IDR_OFFSET)
#  define SAM_USART_IMR(n)           (SAM_USARTN_VBASE(n)+SAM_UART_IMR_OFFSET)
#  define SAM_USART_SR(n)            (SAM_USARTN_VBASE(n)+SAM_UART_SR_OFFSET)
#  define SAM_USART_RHR(n)           (SAM_USARTN_VBASE(n)+SAM_UART_RHR_OFFSET)
#  define SAM_USART_THR(n)           (SAM_USARTN_VBASE(n)+SAM_UART_THR_OFFSET)
#  define SAM_USART_BRGR(n)          (SAM_USARTN_VBASE(n)+SAM_UART_BRGR_OFFSET)
#  define SAM_USART_RTOR(n)          (SAM_USARTN_VBASE(n)+SAM_UART_RTOR_OFFSET)
#  define SAM_USART_TTGR(n)          (SAM_USARTN_VBASE(n)+SAM_UART_TTGR_OFFSET)
#  define SAM_USART_FIDI(n)          (SAM_USARTN_VBASE(n)+SAM_UART_FIDI_OFFSET)
#  define SAM_USART_NER(n)           (SAM_USARTN_VBASE(n)+SAM_UART_NER_OFFSET)
#  define SAM_USART_IFR(n)           (SAM_USARTN_VBASE(n)+SAM_UART_IFR_OFFSET)
#  define SAM_USART_MAN(n)           (SAM_USARTN_VBASE(n)+SAM_UART_MAN_OFFSET)
#  define SAM_USART_WPMR(n)          (SAM_USARTN_VBASE(n)+SAM_UART_WPMR_OFFSET)
#  define SAM_USART_WPSR(n)          (SAM_USARTN_VBASE(n)+SAM_UART_WPSR_OFFSET)

#  define SAM_USART0_CR              (SAM_USART0_VBASE+SAM_UART_CR_OFFSET)
#  define SAM_USART0_MR              (SAM_USART0_VBASE+SAM_UART_MR_OFFSET)
#  define SAM_USART0_IER             (SAM_USART0_VBASE+SAM_UART_IER_OFFSET)
#  define SAM_USART0_IDR             (SAM_USART0_VBASE+SAM_UART_IDR_OFFSET)
#  define SAM_USART0_IMR             (SAM_USART0_VBASE+SAM_UART_IMR_OFFSET)
#  define SAM_USART0_SR              (SAM_USART0_VBASE+SAM_UART_SR_OFFSET)
#  define SAM_USART0_RHR             (SAM_USART0_VBASE+SAM_UART_RHR_OFFSET)
#  define SAM_USART0_THR             (SAM_USART0_VBASE+SAM_UART_THR_OFFSET)
#  define SAM_USART0_BRGR            (SAM_USART0_VBASE+SAM_UART_BRGR_OFFSET)
#  define SAM_USART0_RTOR            (SAM_USART0_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_USART0_TTGR            (SAM_USART0_VBASE+SAM_UART_TTGR_OFFSET)
#  define SAM_USART0_FIDI            (SAM_USART0_VBASE+SAM_UART_FIDI_OFFSET)
#  define SAM_USART0_NER             (SAM_USART0_VBASE+SAM_UART_NER_OFFSET)
#  define SAM_USART0_IFR             (SAM_USART0_VBASE+SAM_UART_IFR_OFFSET)
#  define SAM_USART0_MAN             (SAM_USART0_VBASE+SAM_UART_MAN_OFFSET)
#  define SAM_USART0_WPMR            (SAM_USART0_VBASE+SAM_UART_WPMR_OFFSET)
#  define SAM_USART0_WPSR            (SAM_USART0_VBASE+SAM_UART_WPSR_OFFSET)

#  define SAM_USART1_CR              (SAM_USART1_VBASE+SAM_UART_CR_OFFSET)
#  define SAM_USART1_MR              (SAM_USART1_VBASE+SAM_UART_MR_OFFSET)
#  define SAM_USART1_IER             (SAM_USART1_VBASE+SAM_UART_IER_OFFSET)
#  define SAM_USART1_IDR             (SAM_USART1_VBASE+SAM_UART_IDR_OFFSET)
#  define SAM_USART1_IMR             (SAM_USART1_VBASE+SAM_UART_IMR_OFFSET)
#  define SAM_USART1_SR              (SAM_USART1_VBASE+SAM_UART_SR_OFFSET)
#  define SAM_USART1_RHR             (SAM_USART1_VBASE+SAM_UART_RHR_OFFSET)
#  define SAM_USART1_THR             (SAM_USART1_VBASE+SAM_UART_THR_OFFSET)
#  define SAM_USART1_BRGR            (SAM_USART1_VBASE+SAM_UART_BRGR_OFFSET)
#  define SAM_USART1_RTOR            (SAM_USART1_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_USART1_TTGR            (SAM_USART1_VBASE+SAM_UART_TTGR_OFFSET)
#  define SAM_USART1_FIDI            (SAM_USART1_VBASE+SAM_UART_FIDI_OFFSET)
#  define SAM_USART1_NER             (SAM_USART1_VBASE+SAM_UART_NER_OFFSET)
#  define SAM_USART1_IFR             (SAM_USART1_VBASE+SAM_UART_IFR_OFFSET)
#  define SAM_USART1_MAN             (SAM_USART1_VBASE+SAM_UART_MAN_OFFSET)
#  define SAM_USART1_WPMR            (SAM_USART1_VBASE+SAM_UART_WPMR_OFFSET)
#  define SAM_USART1_WPSR            (SAM_USART1_VBASE+SAM_UART_WPSR_OFFSET)

#  define SAM_USART2_CR              (SAM_USART2_VBASE+SAM_UART_CR_OFFSET)
#  define SAM_USART2_MR              (SAM_USART2_VBASE+SAM_UART_MR_OFFSET)
#  define SAM_USART2_IER             (SAM_USART2_VBASE+SAM_UART_IER_OFFSET)
#  define SAM_USART2_IDR             (SAM_USART2_VBASE+SAM_UART_IDR_OFFSET)
#  define SAM_USART2_IMR             (SAM_USART2_VBASE+SAM_UART_IMR_OFFSET)
#  define SAM_USART2_SR              (SAM_USART2_VBASE+SAM_UART_SR_OFFSET)
#  define SAM_USART2_RHR             (SAM_USART2_VBASE+SAM_UART_RHR_OFFSET)
#  define SAM_USART2_THR             (SAM_USART2_VBASE+SAM_UART_THR_OFFSET)
#  define SAM_USART2_BRGR            (SAM_USART2_VBASE+SAM_UART_BRGR_OFFSET)
#  define SAM_USART2_RTOR            (SAM_USART2_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_USART2_TTGR            (SAM_USART2_VBASE+SAM_UART_TTGR_OFFSET)
#  define SAM_USART2_FIDI            (SAM_USART2_VBASE+SAM_UART_FIDI_OFFSET)
#  define SAM_USART2_NER             (SAM_USART2_VBASE+SAM_UART_NER_OFFSET)
#  define SAM_USART2_IFR             (SAM_USART2_VBASE+SAM_UART_IFR_OFFSET)
#  define SAM_USART2_MAN             (SAM_USART2_VBASE+SAM_UART_MAN_OFFSET)
#  define SAM_USART2_WPMR            (SAM_USART2_VBASE+SAM_UART_WPMR_OFFSET)
#  define SAM_USART2_WPSR            (SAM_USART2_VBASE+SAM_UART_WPSR_OFFSET)

#  define SAM_USART3_CR              (SAM_USART3_VBASE+SAM_UART_CR_OFFSET)
#  define SAM_USART3_MR              (SAM_USART3_VBASE+SAM_UART_MR_OFFSET)
#  define SAM_USART3_IER             (SAM_USART3_VBASE+SAM_UART_IER_OFFSET)
#  define SAM_USART3_IDR             (SAM_USART3_VBASE+SAM_UART_IDR_OFFSET)
#  define SAM_USART3_IMR             (SAM_USART3_VBASE+SAM_UART_IMR_OFFSET)
#  define SAM_USART3_SR              (SAM_USART3_VBASE+SAM_UART_SR_OFFSET)
#  define SAM_USART3_RHR             (SAM_USART3_VBASE+SAM_UART_RHR_OFFSET)
#  define SAM_USART3_THR             (SAM_USART3_VBASE+SAM_UART_THR_OFFSET)
#  define SAM_USART3_BRGR            (SAM_USART3_VBASE+SAM_UART_BRGR_OFFSET)
#  define SAM_USART3_RTOR            (SAM_USART3_VBASE+SAM_UART_RTOR_OFFSET)
#  define SAM_USART3_TTGR            (SAM_USART3_VBASE+SAM_UART_TTGR_OFFSET)
#  define SAM_USART3_FIDI            (SAM_USART3_VBASE+SAM_UART_FIDI_OFFSET)
#  define SAM_USART3_NER             (SAM_USART3_VBASE+SAM_UART_NER_OFFSET)
#  define SAM_USART3_IFR             (SAM_USART3_VBASE+SAM_UART_IFR_OFFSET)
#  define SAM_USART3_MAN             (SAM_USART3_VBASE+SAM_UART_MAN_OFFSET)
#  define SAM_USART3_WPMR            (SAM_USART3_VBASE+SAM_UART_WPMR_OFFSET)
#  define SAM_USART3_WPSR            (SAM_USART3_VBASE+SAM_UART_WPSR_OFFSET)

#  ifdef CONFIG_SAMA5_HAVE_USART4
#    define SAM_USART4_CR            (SAM_USART4_VBASE+SAM_UART_CR_OFFSET)
#    define SAM_USART4_MR            (SAM_USART4_VBASE+SAM_UART_MR_OFFSET)
#    define SAM_USART4_IER           (SAM_USART4_VBASE+SAM_UART_IER_OFFSET)
#    define SAM_USART4_IDR           (SAM_USART4_VBASE+SAM_UART_IDR_OFFSET)
#    define SAM_USART4_IMR           (SAM_USART4_VBASE+SAM_UART_IMR_OFFSET)
#    define SAM_USART4_SR            (SAM_USART4_VBASE+SAM_UART_SR_OFFSET)
#    define SAM_USART4_RHR           (SAM_USART4_VBASE+SAM_UART_RHR_OFFSET)
#    define SAM_USART4_THR           (SAM_USART4_VBASE+SAM_UART_THR_OFFSET)
#    define SAM_USART4_BRGR          (SAM_USART4_VBASE+SAM_UART_BRGR_OFFSET)
#    define SAM_USART4_RTOR          (SAM_USART4_VBASE+SAM_UART_RTOR_OFFSET)
#    define SAM_USART4_TTGR          (SAM_USART4_VBASE+SAM_UART_TTGR_OFFSET)
#    define SAM_USART4_FIDI          (SAM_USART4_VBASE+SAM_UART_FIDI_OFFSET)
#    define SAM_USART4_NER           (SAM_USART4_VBASE+SAM_UART_NER_OFFSET)
#    define SAM_USART4_IFR           (SAM_USART4_VBASE+SAM_UART_IFR_OFFSET)
#    define SAM_USART4_MAN           (SAM_USART4_VBASE+SAM_UART_MAN_OFFSET)
#    define SAM_USART4_WPMR          (SAM_USART4_VBASE+SAM_UART_WPMR_OFFSET)
#    define SAM_USART4_WPSR          (SAM_USART4_VBASE+SAM_UART_WPSR_OFFSET)
#  endif
#endif

/* UART register bit definitions ****************************************************************/

/* UART Control Register */

#define UART_CR_RSTRX                (1 << 2)  /* Bit 2:  Reset Receiver (Common) */
#define UART_CR_RSTTX                (1 << 3)  /* Bit 3:  Reset Transmitter (Common) */
#define UART_CR_RXEN                 (1 << 4)  /* Bit 4:  Receiver Enable (Common) */
#define UART_CR_RXDIS                (1 << 5)  /* Bit 5:  Receiver Disable (Common) */
#define UART_CR_TXEN                 (1 << 6)  /* Bit 6:  Transmitter Enable (Common) */
#define UART_CR_TXDIS                (1 << 7)  /* Bit 7:  Transmitter Disable (Common) */
#define UART_CR_RSTSTA               (1 << 8)  /* Bit 8:  Reset Status Bits (Common) */

#if defined(ATSAMA5D2)
#  define UART_CR_STTTO              (1 << 11) /* Bit 11: Start Time-out (USART UART mode only) */
#  define UART_REQCLR_STTBRK         (1 << 12) /* Bit 12:  Request Clear */
#elif defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_CR_STTBRK             (1 << 9)  /* Bit 9:  Start Break (USART UART mode only) */
#  define UART_CR_STPBRK             (1 << 10) /* Bit 10: Stop Break (USART UART mode only) */
#  define UART_CR_STTTO              (1 << 11) /* Bit 11: Start Time-out (USART UART mode only) */
#  define UART_CR_SENDA              (1 << 12) /* Bit 12: Send Address (USART UART mode only) */
#  define UART_CR_RSTIT              (1 << 13) /* Bit 13: Reset Iterations (USART UART mode only) */
#  define UART_CR_RSTNACK            (1 << 14) /* Bit 14: Reset Non Acknowledge (USART UART mode only) */
#  define UART_CR_RETTO              (1 << 15) /* Bit 15: Rearm Time-out (USART UART mode only) */
#  define UART_CR_RTSEN              (1 << 18) /* Bit 18: Request to Send Enable (USART UART mode only) */
#  define UART_CR_FCS                (1 << 18) /* Bit 18: Force SPI Chip Select (USART SPI mode only) */
#  define UART_CR_RTSDIS             (1 << 19) /* Bit 19: Request to Send Disable (USART UART mode only) */
#  define UART_CR_RCS                (1 << 19) /* Bit 19: Release SPI Chip Select (USART SPI mode only) */
#endif

/* UART Mode Register and USART Mode Register (UART MODE)
 *
 * NOTES:
 * (1) Common to UART and USART (all modes)
 * (2) Common to UART and USART (UART mode only)
 * (3) USART only (all modes)
 * (4) USART only (UART mode only)
 * (5) USART only (SPI mode only)
 */

#if defined(ATSAMA5D2)
#  define UART_MR_FILTER             (1 << 4)  /* Bit 4: Receiver Digital Filter */
#elif defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_MR_MODE_SHIFT         (0)       /* Bits 0-3: (3) */
#  define UART_MR_MODE_MASK          (15 << UART_MR_MODE_SHIFT)
#    define UART_MR_MODE_NORMAL      (0  << UART_MR_MODE_SHIFT) /* Normal */
#    define UART_MR_MODE_RS485       (1  << UART_MR_MODE_SHIFT) /* RS485 */
#    define UART_MR_MODE_HWHS        (2  << UART_MR_MODE_SHIFT) /* Hardware Handshaking */
#    define UART_MR_MODE_ISO7816_0   (4  << UART_MR_MODE_SHIFT) /* IS07816 Protocol: T = 0 */
#    define UART_MR_MODE_ISO7816_1   (6  << UART_MR_MODE_SHIFT) /* IS07816 Protocol: T = 1 */
#    define UART_MR_MODE_IRDA        (8  << UART_MR_MODE_SHIFT) /* IrDA */
#    define UART_MR_MODE_SPIMSTR     (14 << UART_MR_MODE_SHIFT) /* SPI Master (SPI mode only) */
#    define UART_MR_MODE_SPISLV      (15 << UART_MR_MODE_SHIFT) /* SPI Slave (SPI mode only) */
#  define UART_MR_USCLKS_SHIFT       (4)       /* Bits 4-5: Clock Selection (3) */
#  define UART_MR_USCLKS_MASK        (3 << UART_MR_USCLKS_SHIFT)
#    define UART_MR_USCLKS_MCK       (0 << UART_MR_USCLKS_SHIFT) /* MCK */
#    define UART_MR_USCLKS_MCKDIV    (1 << UART_MR_USCLKS_SHIFT) /* MCK/DIV (DIV = 8) */
#    define UART_MR_USCLKS_SCK       (3 << UART_MR_USCLKS_SHIFT) /* SCK */
#  define UART_MR_CHRL_SHIFT         (6)       /* Bits 6-7: Character Length (3) */
#  define UART_MR_CHRL_MASK          (3 << UART_MR_CHRL_SHIFT)
#    define UART_MR_CHRL_5BITS       (0 << UART_MR_CHRL_SHIFT) /* 5 bits */
#    define UART_MR_CHRL_6BITS       (1 << UART_MR_CHRL_SHIFT) /* 6 bits */
#    define UART_MR_CHRL_7BITS       (2 << UART_MR_CHRL_SHIFT) /* 7 bits */
#    define UART_MR_CHRL_8BITS       (3 << UART_MR_CHRL_SHIFT) /* 8 bits */
#  define UART_MR_SYNC               (1 << 8)  /* Bit 8: Synchronous Mode Select (4) */
#  define UART_MR_CPHA               (1 << 8)  /* Bit 8: SPI Clock Phase (5) */
#endif

#define UART_MR_PAR_SHIFT            (9)       /* Bits 9-11: Parity Type (2) */
#define UART_MR_PAR_MASK             (7 << UART_MR_PAR_SHIFT)
#  define UART_MR_PAR_EVEN           (0 << UART_MR_PAR_SHIFT) /* Even parity (1) */
#  define UART_MR_PAR_ODD            (1 << UART_MR_PAR_SHIFT) /* Odd parity (1) */
#  define UART_MR_PAR_SPACE          (2 << UART_MR_PAR_SHIFT) /* Space: parity forced to 0 (1) */
#  define UART_MR_PAR_MARK           (3 << UART_MR_PAR_SHIFT) /* Mark: parity forced to 1 (1) */
#  define UART_MR_PAR_NONE           (4 << UART_MR_PAR_SHIFT) /* No parity (1) */
#  if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#    define UART_MR_PAR_MULTIDROP    (6 << UART_MR_PAR_SHIFT) /* Multidrop mode (2) */
#  endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
#  define UART_MR_BRSRCCK            (1 << 12) /* Bit 12: Baud Rate Source Clock (UART only) */
#    define UART_MR_PERIPHCLK        (0)       /* Bit 12: 0=Peripheral clock */
#    define UART_MR_PMCPCK           (1 << 12) /* Bit 12: 1=PMC programmable clock */
#endif

#if defined(ATSAMA5D3) || defined(ATSAMA5D4)
#  define UART_MR_NBSTOP_SHIFT       (12)      /* Bits 12-13: Number of Stop Bits (2) */
#  define UART_MR_NBSTOP_MASK        (3 << UART_MR_NBSTOP_SHIFT)
#    define UART_MR_NBSTOP_1         (0 << UART_MR_NBSTOP_SHIFT) /* 1 stop bit 1 stop bit */
#    define UART_MR_NBSTOP_1p5       (1 << UART_MR_NBSTOP_SHIFT) /* 1.5 stop bits */
#    define UART_MR_NBSTOP_2         (2 << UART_MR_NBSTOP_SHIFT) /* 2 stop bits 2 stop bits */
#endif

#define UART_MR_CHMODE_SHIFT         (14)      /* Bits 14-15: Channel Mode (2) */
#define UART_MR_CHMODE_MASK          (3 << UART_MR_CHMODE_SHIFT)
#  define UART_MR_CHMODE_NORMAL      (0 << UART_MR_CHMODE_SHIFT) /* Normal Mode */
#  define UART_MR_CHMODE_ECHO        (1 << UART_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define UART_MR_CHMODE_LLPBK       (2 << UART_MR_CHMODE_SHIFT) /* Local Loopback */
#  define UART_MR_CHMODE_RLPBK       (3 << UART_MR_CHMODE_SHIFT) /* Remote Loopback */

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_MR_MSBF               (1 << 16) /* Bit 16: Most Significant Bit first (2) */
#  define UART_MR_CPOL               (1 << 16) /* Bit 16: SPI Clock Polarity (5) */
#  define UART_MR_MODE9              (1 << 17) /* Bit 17: 9-bit Character Length (4) */
#  define UART_MR_CLKO               (1 << 18) /* Bit 18: Clock Output Select (4) */
#  define UART_MR_OVER               (1 << 19) /* Bit 19: Oversampling Mode (4) */
#  define UART_MR_INACK              (1 << 20) /* Bit 20: Inhibit Non Acknowledge (4) */
#  define UART_MR_WRDBT              (1 << 20) /* Bit 20: Wait Read Data Before Transfer (5) */
#  define UART_MR_DSNACK             (1 << 21) /* Bit 21: Disable Successive NACK (4) */
#  define UART_MR_VARSYNC            (1 << 22) /* Bit 22: Variable Synchronization of Command/Data Sync Start Frame Delimiter (4) */
#  define UART_MR_INVDATA            (1 << 23) /* Bit 23: Inverted Data (4) */
#  define UART_MR_MAXITER_SHIFT      (24)      /* Bits 24-26: Max iterations (ISO7816 T=0 (4) */
#  define UART_MR_MAXITER_MASK       (7 << UART_MR_MAXITER_SHIFT)
#  define UART_MR_FILTER             (1 << 28) /* Bit 28: Infrared Receive Line Filter (4) */
#  define UART_MR_MAN                (1 << 29) /* Bit 29: Manchester Encoder/Decoder Enable () */
#  define UART_MR_MODSYNC            (1 << 30) /* Bit 30: Manchester Synchronization Mode (4) */
#  define UART_MR_ONEBIT             (1 << 31) /* Bit 31: Start Frame Delimiter Selector (4) */
#endif

/* UART Interrupt Enable Register, UART Interrupt Disable Register, UART Interrupt Mask
 * Register, and UART Status Register common bit field definitions
 *
 * NOTES:
 * (1) Common to UART and USART (all modes)
 * (2) Common to UART and USART (UART mode only)
 * (3) USART only (all modes)
 * (4) USART only (UART mode only)
 * (5) USART only (SPI mode only)
 */

#define UART_INT_RXRDY               (1 << 0)  /* Bit 0:  RXRDY Interrupt (1) */
#define UART_INT_TXRDY               (1 << 1)  /* Bit 1:  TXRDY Interrupt (1) */

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_INT_RXBRK             (1 << 2)  /* Bit 2:  Break Received/End of Break (2) */
#endif

#define UART_INT_OVRE                (1 << 5)  /* Bit 5:  Overrun Error Interrupt (2) */
#define UART_INT_FRAME               (1 << 6)  /* Bit 6:  Framing Error Interrupt (2) */
#define UART_INT_PARE                (1 << 7)  /* Bit 7:  Parity Error Interrupt (2) */
#define UART_INT_TIMEOUT             (1 << 8)  /* Bit 8:  Time-out Interrupt (4) */
#define UART_INT_TXEMPTY             (1 << 9)  /* Bit 9:  TXEMPTY Interrupt (1) */

#if defined(ATSAMA5D2)
#  define UART_INT_CMP               (1 << 15) /* Bit 15:  Comparison Interrupt */

#  define UART_INT_ALLINTS           0x000083e3
#elif defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_INT_ITER              (1 << 10) /* Bit 10: Iteration Interrupt (4) */
#  define UART_INT_UNRE              (1 << 10) /* Bit 10: SPI Underrun Error Interrupt (5) */
#  define UART_INT_NACK              (1 << 13) /* Bit 13: Non Acknowledge Interrupt (4) */
#  define UART_INT_CTSIC             (1 << 19) /* Bit 19: Clear to Send Input Change Interrupt (4) */
#  define UART_SR_CTS                (1 << 23) /* Bit 23: Image of CTS Input (SR only) */
#  define UART_INT_MANE              (1 << 24) /* Bit 24: Manchester Error Interrupt (4) */

#  define UART_INT_ALLINTS           0x018827e7
#endif

/* UART Receiver Holding Register */

#if defined(ATSAMA5D2)
#  define UART_RHR_RXCHR_SHIFT       (0)       /* Bits 0-7: Received Character (UART only) */
#  define UART_RHR_RXCHR_MASK        (0xff << UART_RHR_RXCHR_SHIFT)
#elif defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_RHR_RXCHR_SHIFT       (0)       /* Bits 0-8: Received Character (USART only) */
#  define UART_RHR_RXCHR_MASK        (0x1ff << UART_RHR_RXCHR_SHIFT)
#  define UART_RHR_RXSYNH            (1 << 15) /* Bit 15: Received Sync (USART only) */
#endif

/* UART Transmit Holding Register */

#if defined(ATSAMA5D2)
#  define UART_THR_TXCHR_SHIFT       (0)       /* Bits 0-7: Character to be Transmitted (UART only) */
#  define UART_THR_TXCHR_MASK        (0xff << UART_THR_TXCHR_SHIFT)
#elif defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_THR_TXCHR_SHIFT       (0)       /* Bits 0-8: Character to be Transmitted (USART only) */
#  define UART_THR_TXCHR_MASK        (0x1ff << UART_THR_TXCHR_SHIFT)
#  define UART_THR_TXSYNH            (1 << 15) /* Bit 15: Sync Field to be tran (USART only) */
#endif

/* UART Baud Rate Generator Register */

#define UART_BRGR_CD_SHIFT           (0)       /* Bits 0-15: Clock Divisor (Common) */
#define UART_BRGR_CD_MASK            (0xffff << UART_BRGR_CD_SHIFT)

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_BRGR_FP_SHIFT         (16)      /* Bits 16-18: Fractional Part (USART only) */
#  define UART_BRGR_FP_MASK          (7 << UART_BRGR_FP_SHIFT)
#endif

#if defined(ATSAMA5D2)
/* Comparison Register */

#  define UART_CMPR_VAL1_SHIFT       (0)       /* Bits 0-7: First Comparison Value for Received Character */
#  define UART_CMPR_VAL1_MASK        (0xff << UART_CMPR_VAL1_SHIFT)
#    define UART_CMPR_VAL1(n)        ((uint32_t)(n) << UART_CMPR_VAL1_SHIFT)
#  define UART_CMPR_CMPMODE          (1 << 12) /* Bit 12: Comparison Mode */
#    define UART_CMPR_FLAG           (0)       /* Bit 12: 0=Any character + comparison function */
#    define UART_CMPR_START          (1 << 12) /* Bit 12: 1=Comparison condition required to start */
#  define UART_CMPR_CMPPAR           (1 << 14) /* Bit 14: Compare Parity */
#  define UART_CMPR_VAL2_SHIFT       (16)      /* Bits 16-23: Second Comparison Value for Received Character */
#  define UART_CMPR_VAL2_MASK        (0xff << UART_CMPR_VAL2_SHIFT)
#    define UART_CMPR_VAL2(n)        ((uint32_t)(n) << UART_CMPR_VAL2_SHIFT)
#endif

/* USART Receiver Time-out Register (USART only) */

#if defined(ATSAMA5D2)
#  define UART_RTOR_TO_SHIFT         (0)       /* Bits 0-7: Time-out Value */
#  define UART_RTOR_TO_MASK          (0xff << UART_RTOR_TO_SHIFT)
#elif defined(ATSAMA5D3) ||defined(ATSAMA5D4)
#  define UART_RTOR_TO_SHIFT         (0)       /* Bits 0-15: Time-out Value (USART only) */
#  define UART_RTOR_TO_MASK          (0xffff << UART_RTOR_TO_SHIFT)
#endif

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
/* USART Transmitter Timeguard Register (USART only) */

#  define UART_TTGR_TG_SHIFT         (0)       /* Bits 0-7: Timeguard Value (USART only) */
#  define UART_TTGR_TG_MASK          (0xff << UART_TTGR_TG_SHIFT)
#endif

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
/* USART FI DI RATIO Register (USART only) */

#  define UART_FIDI_RATIO_SHIFT      (0)       /* Bits 0-15: FI Over DI Ratio Value (USART only) */
#  define UART_FIDI_RATIO_MASK       (0xffff << UART_FIDI_RATIO_SHIFT)
#endif

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
/* USART Number of Errors Register (USART only) */

#  define UART_NER_NBERRORS_SHIFT    (0)       /* Bits 0-7: Number of Errrors (USART only) */
#  define UART_NER_NBERRORS_MASK     (0xff << UART_NER_NBERRORS_SHIFT)
#endif

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
/* USART IrDA FILTER Register (USART only) */

#  define UART_IFR_IRDAFILTER_SHIFT  (0)       /* Bits 0-7: IrDA Filter (USART only) */
#  define UART_IFR_IRDAFILTER_MASK   (0xff << UART_IFR_IRDAFILTER_SHIFT)
#endif

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
/* USART Manchester Configuration Register (USART only) */

#  define UART_MAN_TXPL_SHIFT        (0)       /* Bits 0-3: Transmitter Preamble Length (USART only) */
#  define UART_MAN_TXPL_MASK         (15 << UART_MAN_TXPL_SHIFT)
#  define UART_MAN_TXPP_SHIFT        (8)       /* Bits 8-9: Transmitter Preamble Pattern (USART only) */
#  define UART_MAN_TXPP_MASK         (3 << UART_MAN_TXPP_SHIFT)
#    define UART_MAN_TXPP_ALLONE     (0 << UART_MAN_TXPP_SHIFT) /* ALL_ONE */
#    define UART_MAN_TXPP_ALLZERO    (1 << UART_MAN_TXPP_SHIFT) /* ALL_ZERO */
#    define UART_MAN_TXPP_ZEROONE    (2 << UART_MAN_TXPP_SHIFT) /* ZERO_ONE */
#    define UART_MAN_TXPP_ONEZERO    (3 << UART_MAN_TXPP_SHIFT) /* ONE_ZERO */
#  define UART_MAN_TXMPOL            (1 << 12) /* Bit 12: Transmitter Manchester Polarity (USART only) */
#  define UART_MAN_RXPL_SHIFT        (16)      /* Bits 16-19: Receiver Preamble Length (USART only) */
#  define UART_MAN_RXPL_MASK         (15 << UART_MAN_RXPL_SHIFT)
#  define UART_MAN_RXPP_SHIFT        (24)      /* Bits 24-25: Receiver Preamble Pattern detected (USART only) */
#  define UART_MAN_RXPP_MASK         (3 << UART_MAN_RXPP_SHIFT)
#    define UART_MAN_RXPP_ALLONE     (0 << UART_MAN_RXPP_SHIFT) /* ALL_ONE */
#    define UART_MAN_RXPP_ALLZERO    (1 << UART_MAN_RXPP_SHIFT) /* ALL_ZERO */
#    define UART_MAN_RXPP_ZEROONE    (2 << UART_MAN_RXPP_SHIFT) /* ZERO_ONE */
#    define UART_MAN_RXPP_ONEZERO    (3 << UART_MAN_RXPP_SHIFT) /* ONE_ZERO */
#  define UART_MAN_RXMPOL            (1 << 28) /* Bit 28: Receiver Manchester Polarity (USART only) */
#  define UART_MAN_ONE               (1 << 29) /* Bit 29: Must Be Set to 1 */
#  define UART_MAN_DRIFT             (1 << 30) /* Bit 30: Drift compensation (USART only) */
#  endif

/* USART Write Protect Mode Register (USART only) */

#define UART_WPMR_WPEN               (1 << 0)  /* Bit 0: Write Protect Enable (USART only) */
#define UART_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY (USART only) */
#define UART_WPMR_WPKEY_MASK         (0x00ffffff << UART_WPMR_WPKEY_SHIFT)
#  define USART_WPMR_WPKEY           (0x00555341 << UART_WPMR_WPKEY_SHIFT) /* "USA" */
#  define UART_WPMR_WPKEY            (0x00554152 << UART_WPMR_WPKEY_SHIFT) /* "UAR" */

#if defined(ATSAMA5D3) ||defined(ATSAMA5D4)
/* USART Write Protect Status Register (USART only) */

#  define UART_WPSR_WPVS             (1 << 0)  /* Bit 0: Write Protect Violation Status (USART only) */
#  define UART_WPSR_WPVSRC_SHIFT     (8)       /* Bits 8-23: Write Protect Violation Source (USART only) */
#  define UART_WPSR_WPVSRC_MASK      (0xffff << UART_WPSR_WPVSRC_SHIFT)
#endif

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_UART_H */
