/************************************************************************************************
 * arch/arm/src/samv7/hardware/sam_uart.h
 * Universal Asynchronous Receiver Transmitter (UART) and Universal Synchronous Asynchronous
 * Receiver Transmitter (USART) definitions for the SAMV71.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UART_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UART_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include "arch/samv7/chip.h"
#include "hardware/sam_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* UART register offsets ************************************************************************/

#define SAM_UART_CR_OFFSET           0x0000 /* Control Register (Common) */
#define SAM_UART_MR_OFFSET           0x0004 /* Mode Register (Common) */
#define SAM_UART_IER_OFFSET          0x0008 /* Interrupt Enable Register (Common) */
#define SAM_UART_IDR_OFFSET          0x000c /* Interrupt Disable Register (Common) */
#define SAM_UART_IMR_OFFSET          0x0010 /* Interrupt Mask Register (Common) */
#define SAM_UART_SR_OFFSET           0x0014 /* [Channel] Status Register (Common) */
#define SAM_UART_RHR_OFFSET          0x0018 /* Receive Holding Register (Common) */
#define SAM_UART_THR_OFFSET          0x001c /* Transmit Holding Register (Common) */
#define SAM_UART_BRGR_OFFSET         0x0020 /* Baud Rate Generator Register (Common) */
#define SAM_UART_CMPR_OFFSET         0x0024 /* Comparison Register (UART only) */
#define SAM_UART_RTOR_OFFSET         0x0024 /* Receiver Time-out Register (USART only) */
#define SAM_UART_TTGR_OFFSET         0x0028 /* Transmitter Timeguard Register (USART only) */
                                            /* 0x0028-0x00e0: Reserved (UART) */
                                            /* 0x002c-0x004c: Reserved (USART) */
#define SAM_UART_IFR_OFFSET          0x004c /* IrDA Filter Register (USART only) */
#define SAM_UART_MAN_OFFSET          0x0050 /* Manchester Encoder Decoder Register (USART only) */
#define SAM_UART_LINMR_OFFSET        0x0054 /* LIN Mode Register (USART only) */
#define SAM_UART_LINIR_OFFSET        0x0058 /* LIN Identifier Register (USART only) */
#define SAM_UART_LINBRR_OFFSET       0x005c /* LIN Baud Rate Register (USART only) */
#define SAM_UART_LONMR_OFFSET        0x0060 /* LON Mode Register (USART only) */
#define SAM_UART_LONPR_OFFSET        0x0064 /* LON Preamble Register (USART only) */
#define SAM_UART_LONDL_OFFSET        0x0068 /* LON Data Length Register (USART only) */
#define SAM_UART_LONL2HDR_OFFSET     0x006c /* LON L2HDR Register (USART only) */
#define SAM_UART_LONBL_OFFSET        0x0070 /* LON Backlog Register (USART only) */
#define SAM_UART_LONB1TX_OFFSET      0x0074 /* LON Beta1 Tx Register (USART only) */
#define SAM_UART_LONB1RX_OFFSET      0x0078 /* LON Beta1 Rx Register (USART only) */
#define SAM_UART_LONPRIO_OFFSET      0x007c /* LON Priority Register (USART only) */
#define SAM_UART_IDTTX_OFFSET        0x0080 /* LON IDT Tx Register (USART only) */
#define SAM_UART_IDTRX_OFFSET        0x0084 /* LON IDT Rx Register (USART only) */
#define SAM_UART_ICDIFF_OFFSET       0x0088 /* IC DIFF Register (USART only) */
                                            /* 0x008c-0x00e0: Reserved (USART) */
#define SAM_UART_WPMR_OFFSET         0x00e4 /* Write Protect Mode Register (common) */
#define SAM_UART_WPSR_OFFSET         0x00e8 /* Write Protect Status Register (USART only) */
                                            /* 0x00ec-0x00fc: Reserved (USART) */

/* UART register addresses **********************************************************************/

#if SAMV7_NUART > 0
#  define SAM_UART0_CR               (SAM_UART0_BASE+SAM_UART_CR_OFFSET)
#  define SAM_UART0_MR               (SAM_UART0_BASE+SAM_UART_MR_OFFSET)
#  define SAM_UART0_IER              (SAM_UART0_BASE+SAM_UART_IER_OFFSET)
#  define SAM_UART0_IDR              (SAM_UART0_BASE+SAM_UART_IDR_OFFSET)
#  define SAM_UART0_IMR              (SAM_UART0_BASE+SAM_UART_IMR_OFFSET)
#  define SAM_UART0_SR               (SAM_UART0_BASE+SAM_UART_SR_OFFSET)
#  define SAM_UART0_RHR              (SAM_UART0_BASE+SAM_UART_RHR_OFFSET)
#  define SAM_UART0_THR              (SAM_UART0_BASE+SAM_UART_THR_OFFSET)
#  define SAM_UART0_BRGR             (SAM_UART0_BASE+SAM_UART_BRGR_OFFSET)
#  define SAM_UART0_CMPR             (SAM_UART0_BASE+SAM_UART_CMPR_OFFSET)
#endif

#if SAMV7_NUART > 1
#  define SAM_UART1_CR               (SAM_UART1_BASE+SAM_UART_CR_OFFSET)
#  define SAM_UART1_MR               (SAM_UART1_BASE+SAM_UART_MR_OFFSET)
#  define SAM_UART1_IER              (SAM_UART1_BASE+SAM_UART_IER_OFFSET)
#  define SAM_UART1_IDR              (SAM_UART1_BASE+SAM_UART_IDR_OFFSET)
#  define SAM_UART1_IMR              (SAM_UART1_BASE+SAM_UART_IMR_OFFSET)
#  define SAM_UART1_SR               (SAM_UART1_BASE+SAM_UART_SR_OFFSET)
#  define SAM_UART1_RHR              (SAM_UART1_BASE+SAM_UART_RHR_OFFSET)
#  define SAM_UART1_THR              (SAM_UART1_BASE+SAM_UART_THR_OFFSET)
#  define SAM_UART1_BRGR             (SAM_UART1_BASE+SAM_UART_BRGR_OFFSET)
#  define SAM_UART1_CMPR             (SAM_UART1_BASE+SAM_UART_CMPR_OFFSET)
#endif

#if SAMV7_NUART > 2
#  define SAM_UART2_CR               (SAM_UART2_BASE+SAM_UART_CR_OFFSET)
#  define SAM_UART2_MR               (SAM_UART2_BASE+SAM_UART_MR_OFFSET)
#  define SAM_UART2_IER              (SAM_UART2_BASE+SAM_UART_IER_OFFSET)
#  define SAM_UART2_IDR              (SAM_UART2_BASE+SAM_UART_IDR_OFFSET)
#  define SAM_UART2_IMR              (SAM_UART2_BASE+SAM_UART_IMR_OFFSET)
#  define SAM_UART2_SR               (SAM_UART2_BASE+SAM_UART_SR_OFFSET)
#  define SAM_UART2_RHR              (SAM_UART2_BASE+SAM_UART_RHR_OFFSET)
#  define SAM_UART2_THR              (SAM_UART2_BASE+SAM_UART_THR_OFFSET)
#  define SAM_UART2_BRGR             (SAM_UART2_BASE+SAM_UART_BRGR_OFFSET)
#  define SAM_UART2_CMPR             (SAM_UART2_BASE+SAM_UART_CMPR_OFFSET)
#endif

#if SAMV7_NUART > 3
#  define SAM_UART3_CR               (SAM_UART3_BASE+SAM_UART_CR_OFFSET)
#  define SAM_UART3_MR               (SAM_UART3_BASE+SAM_UART_MR_OFFSET)
#  define SAM_UART3_IER              (SAM_UART3_BASE+SAM_UART_IER_OFFSET)
#  define SAM_UART3_IDR              (SAM_UART3_BASE+SAM_UART_IDR_OFFSET)
#  define SAM_UART3_IMR              (SAM_UART3_BASE+SAM_UART_IMR_OFFSET)
#  define SAM_UART3_SR               (SAM_UART3_BASE+SAM_UART_SR_OFFSET)
#  define SAM_UART3_RHR              (SAM_UART3_BASE+SAM_UART_RHR_OFFSET)
#  define SAM_UART3_THR              (SAM_UART3_BASE+SAM_UART_THR_OFFSET)
#  define SAM_UART3_BRGR             (SAM_UART3_BASE+SAM_UART_BRGR_OFFSET)
#  define SAM_UART3_CMPR             (SAM_UART3_BASE+SAM_UART_CMPR_OFFSET)
#endif

#if SAMV7_NUART > 4
#  define SAM_UART4_CR               (SAM_UART4_BASE+SAM_UART_CR_OFFSET)
#  define SAM_UART4_MR               (SAM_UART4_BASE+SAM_UART_MR_OFFSET)
#  define SAM_UART4_IER              (SAM_UART4_BASE+SAM_UART_IER_OFFSET)
#  define SAM_UART4_IDR              (SAM_UART4_BASE+SAM_UART_IDR_OFFSET)
#  define SAM_UART4_IMR              (SAM_UART4_BASE+SAM_UART_IMR_OFFSET)
#  define SAM_UART4_SR               (SAM_UART4_BASE+SAM_UART_SR_OFFSET)
#  define SAM_UART4_RHR              (SAM_UART4_BASE+SAM_UART_RHR_OFFSET)
#  define SAM_UART4_THR              (SAM_UART4_BASE+SAM_UART_THR_OFFSET)
#  define SAM_UART4_BRGR             (SAM_UART4_BASE+SAM_UART_BRGR_OFFSET)
#  define SAM_UART4_CMPR             (SAM_UART4_BASE+SAM_UART_CMPR_OFFSET)
#endif

#if SAMV7_NUSART > 0
#  define SAM_USART0_CR              (SAM_USART0_BASE+SAM_UART_CR_OFFSET)
#  define SAM_USART0_MR              (SAM_USART0_BASE+SAM_UART_MR_OFFSET)
#  define SAM_USART0_IER             (SAM_USART0_BASE+SAM_UART_IER_OFFSET)
#  define SAM_USART0_IDR             (SAM_USART0_BASE+SAM_UART_IDR_OFFSET)
#  define SAM_USART0_IMR             (SAM_USART0_BASE+SAM_UART_IMR_OFFSET)
#  define SAM_USART0_SR              (SAM_USART0_BASE+SAM_UART_SR_OFFSET)
#  define SAM_USART0_RHR             (SAM_USART0_BASE+SAM_UART_RHR_OFFSET)
#  define SAM_USART0_THR             (SAM_USART0_BASE+SAM_UART_THR_OFFSET)
#  define SAM_USART0_BRGR            (SAM_USART0_BASE+SAM_UART_BRGR_OFFSET)
#  define SAM_USART0_RTOR            (SAM_USART0_BASE+SAM_UART_RTOR_OFFSET)
#  define SAM_USART0_TTGR            (SAM_USART0_BASE+SAM_UART_TTGR_OFFSET)
#  define SAM_USART0_IFR             (SAM_USART0_BASE+SAM_UART_IFR_OFFSET)
#  define SAM_USART0_MAN             (SAM_USART0_BASE+SAM_UART_MAN_OFFSET)
#  define SAM_USART0_LINMR           (SAM_USART0_BASE+SAM_UART_LINMR_OFFSET)
#  define SAM_USART0_LINIR           (SAM_USART0_BASE+SAM_UART_LINIR_OFFSET)
#  define SAM_USART0_LINBRR          (SAM_USART0_BASE+SAM_UART_LINBRR_OFFSET)
#  define SAM_USART0_LONMR           (SAM_USART0_BASE+SAM_UART_LONMR_OFFSET)
#  define SAM_USART0_LONPR           (SAM_USART0_BASE+SAM_UART_LONPR_OFFSET)
#  define SAM_USART0_LONDL           (SAM_USART0_BASE+SAM_UART_LONDL_OFFSET)
#  define SAM_USART0_LONL2HDR        (SAM_USART0_BASE+SAM_UART_LONL2HDR_OFFSET)
#  define SAM_USART0_LONBL           (SAM_USART0_BASE+SAM_UART_LONBL_OFFSET)
#  define SAM_USART0_LONB1TX         (SAM_USART0_BASE+SAM_UART_LONB1TX_OFFSET)
#  define SAM_USART0_LONB1RX         (SAM_USART0_BASE+SAM_UART_LONB1RX_OFFSET)
#  define SAM_USART0_LONPRIO         (SAM_USART0_BASE+SAM_UART_LONPRIO_OFFSET)
#  define SAM_USART0_IDTTX           (SAM_USART0_BASE+SAM_UART_IDTTX_OFFSET)
#  define SAM_USART0_IDTRX           (SAM_USART0_BASE+SAM_UART_IDTRX_OFFSET)
#  define SAM_USART0_ICDIFF          (SAM_USART0_BASE+SAM_UART_ICDIFF_OFFSET)
#  define SAM_USART0_WPMR            (SAM_USART0_BASE+SAM_UART_WPMR_OFFSET)
#  define SAM_USART0_WPSR            (SAM_USART0_BASE+SAM_UART_WPSR_OFFSET)
#endif

#if SAMV7_NUSART > 1
#  define SAM_USART1_CR              (SAM_USART1_BASE+SAM_UART_CR_OFFSET)
#  define SAM_USART1_MR              (SAM_USART1_BASE+SAM_UART_MR_OFFSET)
#  define SAM_USART1_IER             (SAM_USART1_BASE+SAM_UART_IER_OFFSET)
#  define SAM_USART1_IDR             (SAM_USART1_BASE+SAM_UART_IDR_OFFSET)
#  define SAM_USART1_IMR             (SAM_USART1_BASE+SAM_UART_IMR_OFFSET)
#  define SAM_USART1_SR              (SAM_USART1_BASE+SAM_UART_SR_OFFSET)
#  define SAM_USART1_RHR             (SAM_USART1_BASE+SAM_UART_RHR_OFFSET)
#  define SAM_USART1_THR             (SAM_USART1_BASE+SAM_UART_THR_OFFSET)
#  define SAM_USART1_BRGR            (SAM_USART1_BASE+SAM_UART_BRGR_OFFSET)
#  define SAM_USART1_RTOR            (SAM_USART1_BASE+SAM_UART_RTOR_OFFSET)
#  define SAM_USART1_TTGR            (SAM_USART1_BASE+SAM_UART_TTGR_OFFSET)
#  define SAM_USART1_IFR             (SAM_USART1_BASE+SAM_UART_IFR_OFFSET)
#  define SAM_USART1_MAN             (SAM_USART1_BASE+SAM_UART_MAN_OFFSET)
#  define SAM_USART1_LINMR           (SAM_USART1_BASE+SAM_UART_LINMR_OFFSET)
#  define SAM_USART1_LINIR           (SAM_USART1_BASE+SAM_UART_LINIR_OFFSET)
#  define SAM_USART1_LINBRR          (SAM_USART1_BASE+SAM_UART_LINBRR_OFFSET)
#  define SAM_USART1_LONMR           (SAM_USART1_BASE+SAM_UART_LONMR_OFFSET)
#  define SAM_USART1_LONPR           (SAM_USART1_BASE+SAM_UART_LONPR_OFFSET)
#  define SAM_USART1_LONDL           (SAM_USART1_BASE+SAM_UART_LONDL_OFFSET)
#  define SAM_USART1_LONL2HDR        (SAM_USART1_BASE+SAM_UART_LONL2HDR_OFFSET)
#  define SAM_USART1_LONBL           (SAM_USART1_BASE+SAM_UART_LONBL_OFFSET)
#  define SAM_USART1_LONB1TX         (SAM_USART1_BASE+SAM_UART_LONB1TX_OFFSET)
#  define SAM_USART1_LONB1RX         (SAM_USART1_BASE+SAM_UART_LONB1RX_OFFSET)
#  define SAM_USART1_LONPRIO         (SAM_USART1_BASE+SAM_UART_LONPRIO_OFFSET)
#  define SAM_USART1_IDTTX           (SAM_USART1_BASE+SAM_UART_IDTTX_OFFSET)
#  define SAM_USART1_IDTRX           (SAM_USART1_BASE+SAM_UART_IDTRX_OFFSET)
#  define SAM_USART1_ICDIFF          (SAM_USART1_BASE+SAM_UART_ICDIFF_OFFSET)
#  define SAM_USART1_WPMR            (SAM_USART1_BASE+SAM_UART_WPMR_OFFSET)
#  define SAM_USART1_WPSR            (SAM_USART1_BASE+SAM_UART_WPSR_OFFSET)
#endif

#if SAMV7_NUSART > 2
#  define SAM_USART2_CR              (SAM_USART2_BASE+SAM_UART_CR_OFFSET)
#  define SAM_USART2_MR              (SAM_USART2_BASE+SAM_UART_MR_OFFSET)
#  define SAM_USART2_IER             (SAM_USART2_BASE+SAM_UART_IER_OFFSET)
#  define SAM_USART2_IDR             (SAM_USART2_BASE+SAM_UART_IDR_OFFSET)
#  define SAM_USART2_IMR             (SAM_USART2_BASE+SAM_UART_IMR_OFFSET)
#  define SAM_USART2_SR              (SAM_USART2_BASE+SAM_UART_SR_OFFSET)
#  define SAM_USART2_RHR             (SAM_USART2_BASE+SAM_UART_RHR_OFFSET)
#  define SAM_USART2_THR             (SAM_USART2_BASE+SAM_UART_THR_OFFSET)
#  define SAM_USART2_BRGR            (SAM_USART2_BASE+SAM_UART_BRGR_OFFSET)
#  define SAM_USART2_RTOR            (SAM_USART2_BASE+SAM_UART_RTOR_OFFSET)
#  define SAM_USART2_TTGR            (SAM_USART2_BASE+SAM_UART_TTGR_OFFSET)
#  define SAM_USART2_IFR             (SAM_USART2_BASE+SAM_UART_IFR_OFFSET)
#  define SAM_USART2_MAN             (SAM_USART2_BASE+SAM_UART_MAN_OFFSET)
#  define SAM_USART2_LINMR           (SAM_USART2_BASE+SAM_UART_LINMR_OFFSET)
#  define SAM_USART2_LINIR           (SAM_USART2_BASE+SAM_UART_LINIR_OFFSET)
#  define SAM_USART2_LINBRR          (SAM_USART2_BASE+SAM_UART_LINBRR_OFFSET)
#  define SAM_USART2_LONMR           (SAM_USART2_BASE+SAM_UART_LONMR_OFFSET)
#  define SAM_USART2_LONPR           (SAM_USART2_BASE+SAM_UART_LONPR_OFFSET)
#  define SAM_USART2_LONDL           (SAM_USART2_BASE+SAM_UART_LONDL_OFFSET)
#  define SAM_USART2_LONL2HDR        (SAM_USART2_BASE+SAM_UART_LONL2HDR_OFFSET)
#  define SAM_USART2_LONBL           (SAM_USART2_BASE+SAM_UART_LONBL_OFFSET)
#  define SAM_USART2_LONB1TX         (SAM_USART2_BASE+SAM_UART_LONB1TX_OFFSET)
#  define SAM_USART2_LONB1RX         (SAM_USART2_BASE+SAM_UART_LONB1RX_OFFSET)
#  define SAM_USART2_LONPRIO         (SAM_USART2_BASE+SAM_UART_LONPRIO_OFFSET)
#  define SAM_USART2_IDTTX           (SAM_USART2_BASE+SAM_UART_IDTTX_OFFSET)
#  define SAM_USART2_IDTRX           (SAM_USART2_BASE+SAM_UART_IDTRX_OFFSET)
#  define SAM_USART2_ICDIFF          (SAM_USART2_BASE+SAM_UART_ICDIFF_OFFSET)
#  define SAM_USART2_WPMR            (SAM_USART2_BASE+SAM_UART_WPMR_OFFSET)
#  define SAM_USART2_WPSR            (SAM_USART2_BASE+SAM_UART_WPSR_OFFSET)
#endif

/* UART register bit definitions ****************************************************************/

/* UART Control Register */

#define UART_CR_RSTRX                (1 << 2)  /* Bit 2:  Reset Receiver (Common) */
#define UART_CR_RSTTX                (1 << 3)  /* Bit 3:  Reset Transmitter (Common) */
#define UART_CR_RXEN                 (1 << 4)  /* Bit 4:  Receiver Enable (Common) */
#define UART_CR_RXDIS                (1 << 5)  /* Bit 5:  Receiver Disable (Common) */
#define UART_CR_TXEN                 (1 << 6)  /* Bit 6:  Transmitter Enable (Common) */
#define UART_CR_TXDIS                (1 << 7)  /* Bit 7:  Transmitter Disable (Common) */
#define UART_CR_RSTSTA               (1 << 8)  /* Bit 8:  Reset Status Bits (Common, not SPI mode) */
#define UART_CR_STTBRK               (1 << 9)  /* Bit 9:  Start Break (USART, UART mode only) */
#define UART_CR_STPBRK               (1 << 10) /* Bit 10: Stop Break (USART, UART mode only) */
#define UART_CR_STTTO                (1 << 11) /* Bit 11: Start Time-out (USART, UART mode only) */
#define UART_CR_REQCLR               (1 << 12) /* Bit 12:Request Clear (UART only) */
#define UART_CR_SENDA                (1 << 12) /* Bit 12: Send Address (USART, UART mode only) */
#define UART_CR_RETTO                (1 << 15) /* Bit 15: Rearm Time-out (USART, UART mode only) */
#define UART_CR_RTSEN                (1 << 18) /* Bit 18: Request to Send Enable (USART, UART mode only) */
#define UART_CR_FCS                  (1 << 18) /* Bit 18: Force SPI Chip Select (USART, SPI mode only) */
#define UART_CR_RTSDIS               (1 << 19) /* Bit 19: Request to Send Disable (USART, UART mode only) */
#define UART_CR_RCS                  (1 << 19) /* Bit 19: Release SPI Chip Select (USART, SPI mode only) */
#define UART_CR_LINABT               (1 << 20) /* Bit 20: Abort LIN Transmission (USART, LIN mode only) */
#define UART_CR_LINWKUP              (1 << 21) /* Bit 21: Send LIN Wakeup Signal (USART, LIN mode only) */

/* UART Mode Register and USART Mode Register (UART MODE) */

#define UART_MR_MODE_SHIFT           (0)       /* Bits 0-3: (USART only) */
#define UART_MR_MODE_MASK            (15 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_NORMAL        (0  << UART_MR_MODE_SHIFT) /* Normal */
#  define UART_MR_MODE_RS485         (1  << UART_MR_MODE_SHIFT) /* RS485 */
#  define UART_MR_MODE_HWHS          (2  << UART_MR_MODE_SHIFT) /* Hardware Handshaking */
#  define UART_MR_MODE_LON           (9  << UART_MR_MODE_SHIFT) /* LON */
#  define UART_MR_MODE_SPIMSTR       (14 << UART_MR_MODE_SHIFT) /* SPI Master (SPI mode only) */
#  define UART_MR_MODE_SPISLV        (15 << UART_MR_MODE_SHIFT) /* SPI Slave (SPI mode only) */
#define UART_MR_DFILTER              (1 << 4)  /* Bit 4: Receiver Digital Filter (UART only) */
#define UART_MR_USCLKS_SHIFT         (4)       /* Bits 4-5: Clock Selection (USART only) */
#define UART_MR_USCLKS_MASK          (3 << UART_MR_USCLKS_SHIFT)
#  define UART_MR_USCLKS_MCK         (0 << UART_MR_USCLKS_SHIFT) /* MCK */
#  define UART_MR_USCLKS_MCKDIV      (1 << UART_MR_USCLKS_SHIFT) /* MCK/DIV (DIV = 8) */
#  define UART_MR_USCLKS_PCK         (2 << UART_MR_USCLKS_SHIFT) /* PMC programmable clock (PCK), UART mode */
#  define UART_MR_USCLKS_SCK         (3 << UART_MR_USCLKS_SHIFT) /* SCK */
#define UART_MR_CHRL_SHIFT           (6)       /* Bits 6-7: Character Length (USART only) */
#define UART_MR_CHRL_MASK            (3 << UART_MR_CHRL_SHIFT)
#  define UART_MR_CHRL_5BITS         (0 << UART_MR_CHRL_SHIFT) /* 5 bits), UART mode only */
#  define UART_MR_CHRL_6BITS         (1 << UART_MR_CHRL_SHIFT) /* 6 bits), UART mode only */
#  define UART_MR_CHRL_7BITS         (2 << UART_MR_CHRL_SHIFT) /* 7 bits), UART mode only */
#  define UART_MR_CHRL_8BITS         (3 << UART_MR_CHRL_SHIFT) /* 8 bits */
#define UART_MR_SYNC                 (1 << 8)  /* Bit 8: Synchronous Mode Select (USART, UART mode only) */
#define UART_MR_CPHA                 (1 << 8)  /* Bit 8: SPI Clock Phase (USART, SPI mode only) */
#define UART_MR_PAR_SHIFT            (9)       /* Bits 9-11: Parity Type (Common, UART mode) */
#define UART_MR_PAR_MASK             (7 << UART_MR_PAR_SHIFT)
#  define UART_MR_PAR_EVEN           (0 << UART_MR_PAR_SHIFT) /* Even parity (Common) */
#  define UART_MR_PAR_ODD            (1 << UART_MR_PAR_SHIFT) /* Odd parity (Common) */
#  define UART_MR_PAR_SPACE          (2 << UART_MR_PAR_SHIFT) /* Space: parity forced to 0 (Common) */
#  define UART_MR_PAR_MARK           (3 << UART_MR_PAR_SHIFT) /* Mark: parity forced to 1 (Common) */
#  define UART_MR_PAR_NONE           (4 << UART_MR_PAR_SHIFT) /* No parity (Common) */
#  define UART_MR_PAR_MULTIDROP      (6 << UART_MR_PAR_SHIFT) /* Multidrop mode (USART only) */
#define UART_MR_BRSRCCK              (1 << 12) /* Bit 12: Baud Rate Source Clock (UART only) */
#define UART_MR_NBSTOP_SHIFT         (12)      /* Bits 12-13: Number of Stop Bits (USART, UART mode only) */
#define UART_MR_NBSTOP_MASK          (3 << UART_MR_NBSTOP_SHIFT)
#  define UART_MR_NBSTOP_1           (0 << UART_MR_NBSTOP_SHIFT) /* 1 stop bit 1 stop bit */
#  define UART_MR_NBSTOP_1p5         (1 << UART_MR_NBSTOP_SHIFT) /* 1.5 stop bits */
#  define UART_MR_NBSTOP_2           (2 << UART_MR_NBSTOP_SHIFT) /* 2 stop bits 2 stop bits */
#define UART_MR_CHMODE_SHIFT         (14)      /* Bits 14-15: Channel Mode (Common, UART mode) */
#define UART_MR_CHMODE_MASK          (3 << UART_MR_CHMODE_SHIFT)
#  define UART_MR_CHMODE_NORMAL      (0 << UART_MR_CHMODE_SHIFT) /* Normal Mode */
#  define UART_MR_CHMODE_ECHO        (1 << UART_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define UART_MR_CHMODE_LLPBK       (2 << UART_MR_CHMODE_SHIFT) /* Local Loopback */
#  define UART_MR_CHMODE_RLPBK       (3 << UART_MR_CHMODE_SHIFT) /* Remote Loopback */
#define UART_MR_MSBF                 (1 << 16) /* Bit 16: Most Significant Bit first (USART, UART mode only) */
#define UART_MR_CPOL                 (1 << 16) /* Bit 16: SPI Clock Polarity (USART, SPI mode only) */
#define UART_MR_MODE9                (1 << 17) /* Bit 17: 9-bit Character Length (USART, UART mode only) */
#define UART_MR_CLKO                 (1 << 18) /* Bit 18: Clock Output Select (USART only) */
#define UART_MR_OVER                 (1 << 19) /* Bit 19: Oversampling Mode (USART, UART mode only) */
#define UART_MR_WRDBT                (1 << 20) /* Bit 20: Wait Read Data Before Transfer (USART, SPI mode only) */
#define UART_MR_VARSYNC              (1 << 22) /* Bit 22: Variable Synchronization of Command/Data Sync Start Frame Delimiter (USART, UART mode only) */
#define UART_MR_IRFILTER             (1 << 28) /* Bit 28: Infrared Receive Line Filter (USART only) */
#define UART_MR_MAN                  (1 << 29) /* Bit 29: Manchester Encoder/Decoder Enable (USART only) */
#define UART_MR_MODSYNC              (1 << 30) /* Bit 30: Manchester Synchronization Mode (USART only) */
#define UART_MR_ONEBIT               (1 << 31) /* Bit 31: Start Frame Delimiter Selector (USART only) */

/* UART Interrupt Enable Register, UART Interrupt Disable Register, UART Interrupt Mask
 * Register, and UART Status Register common bit field definitions
 */

#define UART_INT_RXRDY               (1 << 0)  /* Bit 0:  RXRDY Interrupt (Common) */
#define UART_INT_TXRDY               (1 << 1)  /* Bit 1:  TXRDY Interrupt (Common) */
#define UART_INT_RXBRK               (1 << 2)  /* Bit 2:  Break Received/End of Break (USART, UART mode only) */
#define UART_INT_OVRE                (1 << 5)  /* Bit 5:  Overrun Error Interrupt (Common) */
#define UART_INT_FRAME               (1 << 6)  /* Bit 6:  Framing Error Interrupt (Common, UART mode) */
#define UART_INT_LSF                 (1 << 6)  /* Bit 6:   LON Short Frame Error Interrupt Enablee (USART, LON mode only) */
#define UART_INT_PARE                (1 << 7)  /* Bit 7:  Parity Error Interrupt (Common, UART mode) */
#define UART_INT_LCRCE               (1 << 7)  /* Bit 7:  LON CRC Error Interrupt Enablee (USART, LON mode only) */
#define UART_INT_TIMEOUT             (1 << 8)  /* Bit 8:  Time-out Interrupt (USART, UART mode only) */
#define UART_INT_TXEMPTY             (1 << 9)  /* Bit 9:  TXEMPTY Interrupt (Common) */
#define UART_INT_UNRE                (1 << 10) /* Bit 10: SPI Underrun Error Interrupt (USART SPI mode only) */
#define UART_INT_LINBK               (1 << 13) /* Bit 13: LIN Break Sent or Break Received Interrupt (USART, LIN mode only) */
#define UART_INT_LINID               (1 << 14) /* Bit 14: LIN Identifier Sent or Identifier Received Interrupt (USART, LIN mode only) */
#define UART_INT_CMP                 (1 << 15) /* Bit 15: Enable Comparison Interrupt (UART only) */
#define UART_INT_LINTC               (1 << 15) /* Bit 15: LIN Transfer Completed Interrupt (USART, LIN mode only) */
#define UART_INT_CTSIC               (1 << 19) /* Bit 19: Clear to Send Input Change Interrupt (USART, UART mode only) */
#define UART_INT_MANE                (1 << 24) /* Bit 24: Manchester Error Interrupt (USART. UART mode only) */
#define UART_INT_LTXD                (1 << 24) /* Bit 24: LON Transmission Done Interrupt Enable (USART, LON mode only) */
#define UART_INT_LINBE               (1 << 25) /* Bit 25: LIN Bus Error Interrupt (USART, LIN mode only)*/
#define UART_INT_LCOL                (1 << 25) /* Bit 25: LON Collision Interrupt Enable (USART, LON mode only) */
#define UART_INT_LINISFE             (1 << 26) /* Bit 26: LIN Inconsistent Synch Field Error Interrupt (USART, LIN mode only) */
#define UART_INT_LFET                (1 << 26) /* Bit 26: LON Frame Early Termination Interrupt Enable (USART, LON mode only) */
#define UART_INT_LINIPE              (1 << 27) /* Bit 27: LIN Identifier Parity Interrupt (USART, LIN mode only) */
#define UART_INT_LRXD                (1 << 27) /* Bit 27: LON Reception Done Interrupt Enable (USART, LON mode only) */
#define UART_INT_LINCE               (1 << 28) /* Bit 28: LIN Checksum Error Interrupt (USART, LIN mode only) */
#define UART_INT_LBLOVFE             (1 << 28) /* Bit 28: LON Backlog Overflow Error Interrupt Enable (USART, LON mode only) */
#define UART_INT_LINSNRE             (1 << 29) /* Bit 29: LIN Slave Not Responding Error Interrupt (USART, LIN mode only) */

#define UART_INT_ALLINTS             0x3f08e7e7

/* UART Receiver Holding Register */

#if 0
#  define UART_RHR_RXCHR_SHIFT       (0)       /* Bits 0-7: Received Character (UART only) */
#  define UART_RHR_RXCHR_MASK        (0xff << UART_RHR_RXCHR_SHIFT)
#endif
#define UART_RHR_RXCHR_SHIFT         (0)       /* Bits 0-8: Received Character (USART only) */
#define UART_RHR_RXCHR_MASK          (0x1ff << UART_RHR_RXCHR_SHIFT)
#define UART_RHR_RXSYNH              (1 << 15) /* Bit 15: Received Sync (USART only) */

/* UART Transmit Holding Register */

#if 0
#  define UART_THR_TXCHR_SHIFT       (0)       /* Bits 0-7: Character to be Transmitted (UART only) */
#  define UART_THR_TXCHR_MASK        (0xff << UART_THR_TXCHR_SHIFT)
#endif
#define UART_THR_TXCHR_SHIFT         (0)       /* Bits 0-8: Character to be Transmitted (USART only) */
#define UART_THR_TXCHR_MASK          (0x1ff << UART_THR_TXCHR_SHIFT)
#define UART_THR_TXSYNH              (1 << 15) /* Bit 15: Sync Field to be tran (USART only) */

/* UART Baud Rate Generator Register */

#define UART_BRGR_CD_SHIFT           (0)       /* Bits 0-15: Clock Divisor (Common) */
#define UART_BRGR_CD_MASK            (0xffff << UART_BRGR_CD_SHIFT)
#  define UART_BRGR_CD(n)            ((uint32_t)(n) << UART_BRGR_CD_SHIFT)
#define UART_BRGR_FP_SHIFT           (16)      /* Bits 16-18: Fractional Part (USART only) */
#define UART_BRGR_FP_MASK            (7 << UART_BRGR_FP_SHIFT)
#  define UART_BRGR_FP(n)            ((uint32_t)(n) << UART_BRGR_FP_SHIFT)

/* Comparison Register (UART only) */

#define UART_CMPR_VAL1_SHIFT         (0)       /* Bits 0-7: First Comparison Value for Received Character */
#define UART_CMPR_VAL1_MASK          (0xff << UART_CMPR_VAL1_SHIFT)
#  define UART_CMPR_VAL1(n)          ((uint32_t)(n) << UART_CMPR_VAL1_SHIFT)
#define UART_CMPR_CMPMODE            (1 << 12) /* Bit 12: Comparison Mode */
#define UART_CMPR_CMPPAR             (1 << 14) /* Bit 14: Compare Parity */
#define UART_CMPR_VAL2_SHIFT         (16)      /* Bits 16-23: Second Comparison Value for Received Character */
#define UART_CMPR_VAL2_MASK          (0xff << UART_CMPR_VAL2_SHIFT0
#  define UART_CMPR_VAL2(n)          ((uint32_t)(n) << UART_CMPR_VAL2_SHIFT0

/* USART Receiver Time-out Register (USART only) */

#define UART_RTOR_TO_SHIFT           (0)       /* Bits 0-16: Time-out Value (USART only) */
#define UART_RTOR_TO_MASK            (0x1ffff << UART_RTOR_TO_SHIFT)

/* USART Transmitter Timeguard Register (USART only) */

#define UART_TTGR_TG_SHIFT           (0)       /* Bits 0-7: Timeguard Value (USART only) */
#define UART_TTGR_TG_MASK            (0xff << UART_TTGR_TG_SHIFT)

#define UART_TTGR_PCYCLE_SHIFT       (0)       /* Bits 0-23: LON PCYCLE Length (USART, LON mode only) */
#define UART_TTGR_PCYCLE_MASK        (0xffffff << UART_TTGR_PCYCLE_SHIFT)

/* USART FI DI RATIO Register (LON_MODE)
 * REVISIT: In the preliminary datasheet, these bit fields are identified, but there no no
 * defined address for the FIDL register.
 */

#define UART_FIDL_BETA2_SHIFT        (0)       /* Bits 0-23: LON BETA2 Length (USART, LON mode only) */
#define UART_FIDL_BETA2_MASK         (0xffffff << UART_FIDL_BETA2_SHIFT)

/* USART Manchester Configuration Register (USART only) */

#define UART_MAN_TXPL_SHIFT          (0)       /* Bits 0-3: Transmitter Preamble Length (USART only) */
#define UART_MAN_TXPL_MASK           (15 << UART_MAN_TXPL_SHIFT)
#  define UART_MAN_TXPL(n)           ((uint32_t)(n) << UART_MAN_TXPL_SHIFT)
#define UART_MAN_TXPP_SHIFT          (8)       /* Bits 8-9: Transmitter Preamble Pattern (USART only) */
#define UART_MAN_TXPP_MASK           (3 << UART_MAN_TXPP_SHIFT)
#  define UART_MAN_TXPP_ALLONE       (0 << UART_MAN_TXPP_SHIFT) /* ALL_ONE */
#  define UART_MAN_TXPP_ALLZERO      (1 << UART_MAN_TXPP_SHIFT) /* ALL_ZERO */
#  define UART_MAN_TXPP_ZEROONE      (2 << UART_MAN_TXPP_SHIFT) /* ZERO_ONE */
#  define UART_MAN_TXPP_ONEZERO      (3 << UART_MAN_TXPP_SHIFT) /* ONE_ZERO */
#define UART_MAN_TXMPOL              (1 << 12) /* Bit 12: Transmitter Manchester Polarity (USART only) */
#define UART_MAN_RXPL_SHIFT          (16)      /* Bits 16-19: Receiver Preamble Length (USART only) */
#define UART_MAN_RXPL_MASK           (15 << UART_MAN_RXPL_SHIFT)
#  define UART_MAN_RXPL(n)           ((uint32_t)(n) << UART_MAN_RXPL_SHIFT)
#define UART_MAN_RXPP_SHIFT          (24)      /* Bits 24-25: Receiver Preamble Pattern detected (USART only) */
#define UART_MAN_RXPP_MASK           (3 << UART_MAN_RXPP_SHIFT)
#  define UART_MAN_RXPP_ALLONE       (0 << UART_MAN_RXPP_SHIFT) /* ALL_ONE */
#  define UART_MAN_RXPP_ALLZERO      (1 << UART_MAN_RXPP_SHIFT) /* ALL_ZERO */
#  define UART_MAN_RXPP_ZEROONE      (2 << UART_MAN_RXPP_SHIFT) /* ZERO_ONE */
#  define UART_MAN_RXPP_ONEZERO      (3 << UART_MAN_RXPP_SHIFT) /* ONE_ZERO */
#define UART_MAN_RXMPOL              (1 << 28) /* Bit 28: Receiver Manchester Polarity (USART only) */
#define UART_MAN_ONE                 (1 << 29) /* Bit 29: Must Be Set to 1 */
#define UART_MAN_DRIFT               (1 << 30) /* Bit 30: Drift compensation (USART only) */
#define UART_MAN_RXIDLEV             (1 << 31) /* Bit 31: Receiver Idle Value */

/* LIN Mode Register (USART only) */

#define UART_LINMR_NACT_SHIFT        (0)       /* Bits 0-1: LIN Node Action */
#define UART_LINMR_NACT_MASK         (3 << UART_LINMR_NACT_SHIFT)
#  define UART_LINMR_NACT_PUBLISH    (0 << UART_LINMR_NACT_SHIFT) /* USART transmits response */
#  define UART_LINMR_NACT_SUBSCRIBE  (1 << UART_LINMR_NACT_SHIFT) /* USART receives response */
#  define UART_LINMR_NACT_IGNORE     (2 << UART_LINMR_NACT_SHIFT) /* USART does not transmit or receive response */
#define UART_LINMR_PARDIS            (1 << 2)  /* Bit 2:  Parity Disable */
#define UART_LINMR_CHKDIS            (1 << 3)  /* Bit 3:  Checksum Disable */
#define UART_LINMR_CHKTYP            (1 << 4)  /* Bit 4:  Checksum Type */
#define UART_LINMR_DLM               (1 << 5)  /* Bit 5:  Data Length Mode */
#define UART_LINMR_FSDIS             (1 << 6)  /* Bit 6:  Frame Slot Mode Disable */
#define UART_LINMR_WKUPTYP           (1 << 7)  /* Bit 7:  Wakeup Signal Type */
#define UART_LINMR_DLC_SHIFT         (8)       /* Bits 8-15: Data Length Control */
#define UART_LINMR_DLC_MASK          (0xff << UART_LINMR_DLC_SHIFT)
#  define UART_LINMR_DLC(n)          ((uint32_t)(n) << UART_LINMR_DLC_SHIFT)
#define UART_LINMR_PDCM              (1 << 16) /* Bit 16: PDC Mode */

/* LIN Identifier Register (USART only) */

#define UART_LINIR_MASK              0xff      /* Bits 0-7: Identifier Character */

/* LIN Baud Rate Register (USART only) */

#define UART_LINBRR_LINCD_SHIFT      (0)        /* Bits 0-15: Clock Divider after Synchronization */
#define UART_LINBRR_LINCD_MASK       (0xffff << UART_LINBRR_LINCD_SHIFT)
#  define UART_LINBRR_LINCD(n)       ((uint32_t)(n) << UART_LINBRR_LINCD_SHIFT)
#define UART_LINBRR_LINFP_SHIFT      (16)       /* Bits 16-18: Fractional Part after Synchronization */
#define UART_LINBRR_LINFP_MASK       (7 << UART_LINBRR_LINFP_SHIFT)
#  define UART_LINBRR_LINFP(n)       ((uint32_t)(n) << UART_LINBRR_LINFP_SHIFT)

/* LON Mode Register (USART only) */

#define UART_LONMR_COMMT             (1 << 0)  /* Bit 0:  LON comm_type Parameter Value */
#define UART_LONMR_COLDET            (1 << 1)  /* Bit 1:  LON Collision Detection Feature */
#define UART_LONMR_TCOL              (1 << 2)  /* Bit 2:  Terminate Frame upon Collision Notification */
#define UART_LONMR_CDTAIL            (1 << 3)  /* Bit 3:  LON Collision Detection on Frame Tail */
#define UART_LONMR_DMAM              (1 << 4)  /* Bit 4:  LON DMA Mode */
#define UART_LONMR_LCDS              (1 << 5)  /* Bit 5: LON Collision Detection Source */
#define UART_LONMR_EOFS_SHIFT        (16)      /* Bits 16-23: End of Frame Condition Size */
#define UART_LONMR_EOFS_MASK         (0xff << UART_LONMR_EOFS_SHIFT)
#  define UART_LONMR_EOFS(n)         ((uint32_t)(n) << UART_LONMR_EOFS_SHIFT)

/* LON Preamble Register (USART only) */

#define UART_LONPR_MASK              0x0000ffff /* Bits 0-15-LON preamble length */

/* LON Data Length Register (USART only) */

#define UART_LONDL_MASK              0x000000ff /* Bits 0-7: LON data length */

/* LON L2HDR Register (USART only) */

#define UART_LONL2HDR_BLI_SHIFT      (0)        /* Bits 0-5:  LON Backlog Increment */
#define UART_LONL2HDR_BLI_MASK       (63 << UART_LONL2HDR_BLI_SHIFT)
#  define UART_LONL2HDR_BLI(n)       ((uint32_t)(n) << UART_LONL2HDR_BLI_SHIFT)
#define UART_LONL2HDR_ALTP           (1 << 6)  /* Bit 6:  LON Alternate Path Bit */
#define UART_LONL2HDR_PB             (1 << 7)  /* Bit 7:  LON Priority Bit */

/* LON Backlog Register (USART only) */

#define UART_LONLBL_MASK             0x0000003f  /* Bits 0-5:  LON Backlog Value */

/* LON Beta1 Tx Register (USART only) */

#define UART_LONB1TX_MASK            0x00ffffff  /* Bits 0-23: LON Beta1 Length after Transmission */

/* LON Beta1 Rx Register (USART only) */

#define UART_LONB1RX_MASK            0x00ffffff  /* Bits 0-23: LON Beta1 Length after Reception */

/* LON Priority Register (USART only) */

#define UART_LONPRIO_PSNB_SHIFT      (0)         /* Bits 0-6: LON Priority Slot Number */
#define UART_LONPRIO_PSNB_MASK       (0x7f << UART_LONPRIO_PSNB_SHIFT)
#  define UART_LONPRIO_PSNB(n)       ((uint32_t)(n) << UART_LONPRIO_PSNB_SHIFT)
#define UART_LONPRIO_NPS_SHIFT       (8)         /* Bits 8-14: LON Node Priority Slot */
#define UART_LONPRIO_NPS_MASK        (0x7f << UART_LONPRIO_NPS_SHIFT)
#  define UART_LONPRIO_NPS(n)        ((uint32_t)(n) << UART_LONPRIO_NPS_SHIFT)

/* LON IDT Tx Register (USART only) */

#define UART_IDTTX_MASK              0x00ffffff  /* Bits 0-23: LON Indeterminate Time after Transmission */

/* LON IDT Rx Register (USART only) */

#define UART_IDTRX_MASK              0x00ffffff  /* Bits 0-23: LON Indeterminate Time after Reception */

/* IC DIFF Register (USART only) */

#define UART_ICDIFF_MASK             0x0000000f  /* Bit 0-3: IC Differentiator Number */

/* USART Write Protect Mode Register (USART only) */

#define UART_WPMR_WPEN               (1 << 0)  /* Bit 0: Write Protect Enable (USART only) */
#define UART_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY (USART only) */
#define UART_WPMR_WPKEY_MASK         (0x00ffffff << UART_WPMR_WPKEY_SHIFT)
#  define UART_WPMR_WPKEY            (0x00554152 << UART_WPMR_WPKEY_SHIFT)
#  define USART_WPMR_WPKEY           (0x00555341 << UART_WPMR_WPKEY_SHIFT)

/* USART Write Protect Status Register (USART only) */

#define UART_WPSR_WPVS               (1 << 0)  /* Bit 0: Write Protect Violation Status (USART only) */
#define UART_WPSR_WPVSRC_SHIFT       (8)       /* Bits 8-23: Write Protect Violation Source (USART only) */
#define UART_WPSR_WPVSRC_MASK        (0xffff << UART_WPSR_WPVSRC_SHIFT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UART_H */
