/************************************************************************************************
 * arch/arm/src/sam34/chip/sam4l_uart.h
 * Universal Synchronous Asynchronous Receiver Transmitter (USART) definitions for the SAM4L
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_UART_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_UART_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* USART register offsets ***********************************************************************/

#define SAM_UART_CR_OFFSET           0x0000 /* Control Register */
#define SAM_UART_MR_OFFSET           0x0004 /* Mode Register */
#define SAM_UART_IER_OFFSET          0x0008 /* Interrupt Enable Register */
#define SAM_UART_IDR_OFFSET          0x000c /* Interrupt Disable Register */
#define SAM_UART_IMR_OFFSET          0x0010 /* Interrupt Mask Register */
#define SAM_UART_SR_OFFSET           0x0014 /* Channel Status Register */
#define SAM_UART_RHR_OFFSET          0x0018 /* Receive Holding Register */
#define SAM_UART_THR_OFFSET          0x001c /* Transmit Holding Register */
#define SAM_UART_BRGR_OFFSET         0x0020 /* Baud Rate Generator Register */
#define SAM_UART_RTOR_OFFSET         0x0024 /* Receiver Time-out Register */
#define SAM_UART_TTGR_OFFSET         0x0028 /* Transmitter Timeguard Register */
                                            /* 0x002c-0x003c: Reserved */
#define SAM_UART_FIDI_OFFSET         0x0040 /* FI DI Ratio Register */
#define SAM_UART_NER_OFFSET          0x0044 /* Number of Errors Register */
                                            /* 0x0048: Reserved */
#define SAM_UART_IFR_OFFSET          0x004c /* IrDA Filter Register */
#define SAM_UART_MAN_OFFSET          0x0050 /* Manchester Encoder Decoder Register */
#define SAM_UART_LINMR_OFFSET        0x0054 /* LIN Mode Register */
#define SAM_UART_LINIR_OFFSET        0x0058 /* LIN Identifier Register */
#define SAM_UART_LINBR_OFFSET        0x005c /* LIN Baud Rate Register  */
                                            /* 0x0060-0x00e0: Reserved */
#define SAM_UART_WPMR_OFFSET         0x00e4 /* Write Protect Mode Register */
#define SAM_UART_WPSR_OFFSET         0x00e8 /* Write Protect Status Register */
                                            /* 0x005c-0xf008: Reserved */
#define SAM_UART_VERSION_OFFSET      0x00fc /* Version Register */
                                            /* 0x0100-0x0124: PDC Area */

/* USART register addresses *********************************************************************/

#define SAM_USART_CR(n)              (SAM_USARTN_BASE(n)+SAM_UART_CR_OFFSET)
#define SAM_USART_MR(n)              (SAM_USARTN_BASE(n)+SAM_UART_MR_OFFSET)
#define SAM_USART_IER(n)             (SAM_USARTN_BASE(n)+SAM_UART_IER_OFFSET)
#define SAM_USART_IDR(n)             (SAM_USARTN_BASE(n)+SAM_UART_IDR_OFFSET)
#define SAM_USART_IMR(n)             (SAM_USARTN_BASE(n)+SAM_UART_IMR_OFFSET)
#define SAM_USART_SR(n)              (SAM_USARTN_BASE(n)+SAM_UART_SR_OFFSET)
#define SAM_USART_RHR(n)             (SAM_USARTN_BASE(n)+SAM_UART_RHR_OFFSET)
#define SAM_USART_THR(n)             (SAM_USARTN_BASE(n)+SAM_UART_THR_OFFSET)
#define SAM_USART_BRGR(n)            (SAM_USARTN_BASE(n)+SAM_UART_BRGR_OFFSET)
#define SAM_USART_RTOR(n)            (SAM_USARTN_BASE(n)+SAM_UART_RTOR_OFFSET)
#define SAM_USART_TTGR(n)            (SAM_USARTN_BASE(n)+SAM_UART_TTGR_OFFSET)
#define SAM_USART_FIDI(n)            (SAM_USARTN_BASE(n)+SAM_UART_FIDI_OFFSET)
#define SAM_USART_NER(n)             (SAM_USARTN_BASE(n)+SAM_UART_NER_OFFSET)
#define SAM_USART_IFR(n)             (SAM_USARTN_BASE(n)+SAM_UART_IFR_OFFSET)
#define SAM_USART_MAN(n)             (SAM_USARTN_BASE(n)+SAM_UART_MAN_OFFSET)
#define SAM_USART_LINMR(n)           (SAM_USARTN_BASE(n)+SAM_UART_LINMR_OFFSET)
#define SAM_USART_LINIR(n)           (SAM_USARTN_BASE(n)+SAM_UART_LINIR_OFFSET)
#define SAM_USART_LINBR(n)           (SAM_USARTN_BASE(n)+UART_LINBR_OFFSET)
#define SAM_USART_WPMR(n)            (SAM_USARTN_BASE(n)+SAM_UART_WPMR_OFFSET)
#define SAM_USART_WPSR(n)            (SAM_USARTN_BASE(n)+SAM_UART_WPSR_OFFSET)
#define SAM_USART_VERSION(n)         (SAM_USARTN_BASE(n)+SAM_UART_VERSION_OFFSET)

#define SAM_USART0_CR                (SAM_USART0_BASE+SAM_UART_CR_OFFSET)
#define SAM_USART0_MR                (SAM_USART0_BASE+SAM_UART_MR_OFFSET)
#define SAM_USART0_IER               (SAM_USART0_BASE+SAM_UART_IER_OFFSET)
#define SAM_USART0_IDR               (SAM_USART0_BASE+SAM_UART_IDR_OFFSET)
#define SAM_USART0_IMR               (SAM_USART0_BASE+SAM_UART_IMR_OFFSET)
#define SAM_USART0_SR                (SAM_USART0_BASE+SAM_UART_SR_OFFSET)
#define SAM_USART0_RHR               (SAM_USART0_BASE+SAM_UART_RHR_OFFSET)
#define SAM_USART0_THR               (SAM_USART0_BASE+SAM_UART_THR_OFFSET)
#define SAM_USART0_BRGR              (SAM_USART0_BASE+SAM_UART_BRGR_OFFSET)
#define SAM_USART0_RTOR              (SAM_USART0_BASE+SAM_UART_RTOR_OFFSET)
#define SAM_USART0_TTGR              (SAM_USART0_BASE+SAM_UART_TTGR_OFFSET)
#define SAM_USART0_FIDI              (SAM_USART0_BASE+SAM_UART_FIDI_OFFSET)
#define SAM_USART0_NER               (SAM_USART0_BASE+SAM_UART_NER_OFFSET)
#define SAM_USART0_IFR               (SAM_USART0_BASE+SAM_UART_IFR_OFFSET)
#define SAM_USART0_MAN               (SAM_USART0_BASE+SAM_UART_MAN_OFFSET)
#define SAM_USART0_LINMR             (SAM_USART0_BASE+SAM_UART_LINMR_OFFSET)
#define SAM_USART0_LINIR             (SAM_USART0_BASE+SAM_UART_LINIR_OFFSET)
#define SAM_USART0_LINBR             (SAM_USART0_BASE+UART_LINBR_OFFSET)
#define SAM_USART0_WPMR              (SAM_USART0_BASE+SAM_UART_WPMR_OFFSET)
#define SAM_USART0_WPSR              (SAM_USART0_BASE+SAM_UART_WPSR_OFFSET)
#define SAM_USART0_VERSION           (SAM_USART0_BASE+SAM_UART_VERSION_OFFSET)

#define SAM_USART1_CR                (SAM_USART1_BASE+SAM_UART_CR_OFFSET)
#define SAM_USART1_MR                (SAM_USART1_BASE+SAM_UART_MR_OFFSET)
#define SAM_USART1_IER               (SAM_USART1_BASE+SAM_UART_IER_OFFSET)
#define SAM_USART1_IDR               (SAM_USART1_BASE+SAM_UART_IDR_OFFSET)
#define SAM_USART1_IMR               (SAM_USART1_BASE+SAM_UART_IMR_OFFSET)
#define SAM_USART1_SR                (SAM_USART1_BASE+SAM_UART_SR_OFFSET)
#define SAM_USART1_RHR               (SAM_USART1_BASE+SAM_UART_RHR_OFFSET)
#define SAM_USART1_THR               (SAM_USART1_BASE+SAM_UART_THR_OFFSET)
#define SAM_USART1_BRGR              (SAM_USART1_BASE+SAM_UART_BRGR_OFFSET)
#define SAM_USART1_RTOR              (SAM_USART1_BASE+SAM_UART_RTOR_OFFSET)
#define SAM_USART1_TTGR              (SAM_USART1_BASE+SAM_UART_TTGR_OFFSET)
#define SAM_USART1_FIDI              (SAM_USART1_BASE+SAM_UART_FIDI_OFFSET)
#define SAM_USART1_NER               (SAM_USART1_BASE+SAM_UART_NER_OFFSET)
#define SAM_USART1_IFR               (SAM_USART1_BASE+SAM_UART_IFR_OFFSET)
#define SAM_USART1_MAN               (SAM_USART1_BASE+SAM_UART_MAN_OFFSET)
#define SAM_USART1_LINMR             (SAM_USART1_BASE+SAM_UART_LINMR_OFFSET)
#define SAM_USART1_LINIR             (SAM_USART1_BASE+SAM_UART_LINIR_OFFSET)
#define SAM_USART1_LINBR             (SAM_USART1_BASE+UART_LINBR_OFFSET)
#define SAM_USART1_WPMR              (SAM_USART1_BASE+SAM_UART_WPMR_OFFSET)
#define SAM_USART1_WPSR              (SAM_USART1_BASE+SAM_UART_WPSR_OFFSET)
#define SAM_USART1_VERSION           (SAM_USART1_BASE+SAM_UART_VERSION_OFFSET)

#define SAM_USART2_CR                (SAM_USART2_BASE+SAM_UART_CR_OFFSET)
#define SAM_USART2_MR                (SAM_USART2_BASE+SAM_UART_MR_OFFSET)
#define SAM_USART2_IER               (SAM_USART2_BASE+SAM_UART_IER_OFFSET)
#define SAM_USART2_IDR               (SAM_USART2_BASE+SAM_UART_IDR_OFFSET)
#define SAM_USART2_IMR               (SAM_USART2_BASE+SAM_UART_IMR_OFFSET)
#define SAM_USART2_SR                (SAM_USART2_BASE+SAM_UART_SR_OFFSET)
#define SAM_USART2_RHR               (SAM_USART2_BASE+SAM_UART_RHR_OFFSET)
#define SAM_USART2_THR               (SAM_USART2_BASE+SAM_UART_THR_OFFSET)
#define SAM_USART2_BRGR              (SAM_USART2_BASE+SAM_UART_BRGR_OFFSET)
#define SAM_USART2_RTOR              (SAM_USART2_BASE+SAM_UART_RTOR_OFFSET)
#define SAM_USART2_TTGR              (SAM_USART2_BASE+SAM_UART_TTGR_OFFSET)
#define SAM_USART2_FIDI              (SAM_USART2_BASE+SAM_UART_FIDI_OFFSET)
#define SAM_USART2_NER               (SAM_USART2_BASE+SAM_UART_NER_OFFSET)
#define SAM_USART2_IFR               (SAM_USART2_BASE+SAM_UART_IFR_OFFSET)
#define SAM_USART2_MAN               (SAM_USART2_BASE+SAM_UART_MAN_OFFSET)
#define SAM_USART2_LINMR             (SAM_USART2_BASE+SAM_UART_LINMR_OFFSET)
#define SAM_USART2_LINIR             (SAM_USART2_BASE+SAM_UART_LINIR_OFFSET)
#define SAM_USART2_LINBR             (SAM_USART2_BASE+UART_LINBR_OFFSET)
#define SAM_USART2_WPMR              (SAM_USART2_BASE+SAM_UART_WPMR_OFFSET)
#define SAM_USART2_WPSR              (SAM_USART2_BASE+SAM_UART_WPSR_OFFSET)
#define SAM_USART2_VERSION           (SAM_USART2_BASE+SAM_UART_VERSION_OFFSET)

#define SAM_USART3_CR                (SAM_USART3_BASE+SAM_UART_CR_OFFSET)
#define SAM_USART3_MR                (SAM_USART3_BASE+SAM_UART_MR_OFFSET)
#define SAM_USART3_IER               (SAM_USART3_BASE+SAM_UART_IER_OFFSET)
#define SAM_USART3_IDR               (SAM_USART3_BASE+SAM_UART_IDR_OFFSET)
#define SAM_USART3_IMR               (SAM_USART3_BASE+SAM_UART_IMR_OFFSET)
#define SAM_USART3_SR                (SAM_USART3_BASE+SAM_UART_SR_OFFSET)
#define SAM_USART3_RHR               (SAM_USART3_BASE+SAM_UART_RHR_OFFSET)
#define SAM_USART3_THR               (SAM_USART3_BASE+SAM_UART_THR_OFFSET)
#define SAM_USART3_BRGR              (SAM_USART3_BASE+SAM_UART_BRGR_OFFSET)
#define SAM_USART3_RTOR              (SAM_USART3_BASE+SAM_UART_RTOR_OFFSET)
#define SAM_USART3_TTGR              (SAM_USART3_BASE+SAM_UART_TTGR_OFFSET)
#define SAM_USART3_FIDI              (SAM_USART3_BASE+SAM_UART_FIDI_OFFSET)
#define SAM_USART3_NER               (SAM_USART3_BASE+SAM_UART_NER_OFFSET)
#define SAM_USART3_IFR               (SAM_USART3_BASE+SAM_UART_IFR_OFFSET)
#define SAM_USART3_MAN               (SAM_USART3_BASE+SAM_UART_MAN_OFFSET)
#define SAM_USART3_LINMR             (SAM_USART3_BASE+SAM_UART_LINMR_OFFSET)
#define SAM_USART3_LINIR             (SAM_USART3_BASE+SAM_UART_LINIR_OFFSET)
#define SAM_USART3_LINBR             (SAM_USART3_BASE+UART_LINBR_OFFSET)
#define SAM_USART3_WPMR              (SAM_USART3_BASE+SAM_UART_WPMR_OFFSET)
#define SAM_USART3_WPSR              (SAM_USART3_BASE+SAM_UART_WPSR_OFFSET)
#define SAM_USART3_VERSION           (SAM_USART3_BASE+SAM_UART_VERSION_OFFSET)

/* USART register bit definitions ***************************************************************/

/* USART Control Register */

#define UART_CR_RSTRX                (1 << 2)  /* Bit 2:  Reset Receiver */
#define UART_CR_RSTTX                (1 << 3)  /* Bit 3:  Reset Transmitter */
#define UART_CR_RXEN                 (1 << 4)  /* Bit 4:  Receiver Enable */
#define UART_CR_RXDIS                (1 << 5)  /* Bit 5:  Receiver Disable */
#define UART_CR_TXEN                 (1 << 6)  /* Bit 6:  Transmitter Enable */
#define UART_CR_TXDIS                (1 << 7)  /* Bit 7:  Transmitter Disable */
#define UART_CR_RSTSTA               (1 << 8)  /* Bit 8:  Reset Status Bits */
#define UART_CR_STTBRK               (1 << 9)  /* Bit 9:  Start Break */
#define UART_CR_STPBRK               (1 << 10) /* Bit 10: Stop Break */
#define UART_CR_STTTO                (1 << 11) /* Bit 11: Start Time-out */
#define UART_CR_SENDA                (1 << 12) /* Bit 12: Send Address */
#define UART_CR_RSTIT                (1 << 13) /* Bit 13: Reset Iterations */
#define UART_CR_RSTNACK              (1 << 14) /* Bit 14: Reset Non Acknowledge */
#define UART_CR_RETTO                (1 << 15) /* Bit 15: Rearm Time-out */
#define UART_CR_DTREN                (1 << 16) /* Bit 16: Data Terminal Ready Enable */
#define UART_CR_DTRDIS               (1 << 17) /* Bit 17: Data Terminal Ready Disable */
#define UART_CR_RTSEN                (1 << 18) /* Bit 18: Request to Send Enable */
#define UART_CR_FCS                  (1 << 18) /* Bit 18: Force SPI Chip Select */
#define UART_CR_RTSDIS               (1 << 19) /* Bit 19: Request to Send Disable */
#define UART_CR_RCS                  (1 << 19) /* Bit 19: Release SPI Chip Select */
#define UART_CR_LINABT               (1 << 20) /* Bit 20: Abort LIN Transmission */
#define UART_CR_LINWKUP              (1 << 21) /* Bit 21: Send LIN Wakeup Signal */

/* USART Mode Register */

#define UART_MR_MODE_SHIFT           (0)       /* Bits 0-3: */
#define UART_MR_MODE_MASK            (15 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_NORMAL        (0  << UART_MR_MODE_SHIFT) /* Normal */
#  define UART_MR_MODE_RS485         (1  << UART_MR_MODE_SHIFT) /* RS485 */
#  define UART_MR_MODE_HWHS          (2  << UART_MR_MODE_SHIFT) /* Hardware Handshaking */
#  define UART_MR_MODE_ISO7816_0     (4  << UART_MR_MODE_SHIFT) /* IS07816 Protocol: T = 0 */
#  define UART_MR_MODE_ISO7816_1     (6  << UART_MR_MODE_SHIFT) /* IS07816 Protocol: T = 1 */
#  define UART_MR_MODE_IRDA          (8  << UART_MR_MODE_SHIFT) /* IrDA */
#  define UART_MR_MODE_SPIMSTR       (14 << UART_MR_MODE_SHIFT) /* SPI Master */
#  define UART_MR_MODE_SPISLV        (15 << UART_MR_MODE_SHIFT) /* SPI Slave */
#define UART_MR_USCLKS_SHIFT         (4)       /* Bits 4-5: Clock Selection */
#define UART_MR_USCLKS_MASK          (3 << UART_MR_USCLKS_SHIFT)
#  define UART_MR_USCLKS_USART       (0 << UART_MR_USCLKS_SHIFT) /* CLK_USART */
#  define UART_MR_USCLKS_USARTDIV    (0 << UART_MR_USCLKS_SHIFT) /* CLK_USART/DIV(1) */
#  define UART_MR_USCLKS_CLK         (0 << UART_MR_USCLKS_SHIFT) /* CLK */
#define UART_MR_CHRL_SHIFT           (6)       /* Bits 6-7: Character Length */
#define UART_MR_CHRL_MASK            (3 << UART_MR_CHRL_SHIFT)
#  define UART_MR_CHRL_5BITS         (0 << UART_MR_CHRL_SHIFT) /* 5 bits */
#  define UART_MR_CHRL_6BITS         (1 << UART_MR_CHRL_SHIFT) /* 6 bits */
#  define UART_MR_CHRL_7BITS         (2 << UART_MR_CHRL_SHIFT) /* 7 bits */
#  define UART_MR_CHRL_8BITS         (3 << UART_MR_CHRL_SHIFT) /* 8 bits */
#define UART_MR_SYNC                 (1 << 8)  /* Bit 8: Synchronous Mode Select */
#define UART_MR_CPHA                 (1 << 8)  /* Bit 8: SPI Clock Phase */
#define UART_MR_PAR_SHIFT            (9)       /* Bits 9-11: Parity Type */
#define UART_MR_PAR_MASK             (7 << UART_MR_PAR_SHIFT)
#  define UART_MR_PAR_EVEN           (0 << UART_MR_PAR_SHIFT) /* Even parity */
#  define UART_MR_PAR_ODD            (1 << UART_MR_PAR_SHIFT) /* Odd parity */
#  define UART_MR_PAR_SPACE          (2 << UART_MR_PAR_SHIFT) /* Space: parity forced to 0 */
#  define UART_MR_PAR_MARK           (3 << UART_MR_PAR_SHIFT) /* Mark: parity forced to 1 */
#  define UART_MR_PAR_NONE           (4 << UART_MR_PAR_SHIFT) /* No parity */
#  define UART_MR_PAR_MULTIDROP      (6 << UART_MR_PAR_SHIFT) /* Multidrop mode */
#define UART_MR_NBSTOP_SHIFT         (12)      /* Bits 12-13: Number of Stop Bits */
#define UART_MR_NBSTOP_MASK          (3 << UART_MR_NBSTOP_SHIFT)
#  define UART_MR_NBSTOP_1           (0 << UART_MR_NBSTOP_SHIFT) /* 1 stop bit 1 stop bit */
#  define UART_MR_NBSTOP_1p5         (1 << UART_MR_NBSTOP_SHIFT) /* 1.5 stop bits */
#  define UART_MR_NBSTOP_2           (2 << UART_MR_NBSTOP_SHIFT) /* 2 stop bits 2 stop bits */
#define UART_MR_CHMODE_SHIFT         (14)      /* Bits 14-15: Channel Mode */
#define UART_MR_CHMODE_MASK          (3 << UART_MR_CHMODE_SHIFT)
#  define UART_MR_CHMODE_NORMAL      (0 << UART_MR_CHMODE_SHIFT) /* Normal Mode */
#  define UART_MR_CHMODE_ECHO        (1 << UART_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define UART_MR_CHMODE_LLPBK       (2 << UART_MR_CHMODE_SHIFT) /* Local Loopback */
#  define UART_MR_CHMODE_RLPBK       (3 << UART_MR_CHMODE_SHIFT) /* Remote Loopback */
#define UART_MR_MSBF                 (1 << 16) /* Bit 16: Most Significant Bit first */
#define UART_MR_CPOL                 (1 << 16) /* Bit 16: SPI Clock Polarity */
#define UART_MR_MODE9                (1 << 17) /* Bit 17: 9-bit Character Length */
#define UART_MR_CLKO                 (1 << 18) /* Bit 18: Clock Output Select */
#define UART_MR_OVER                 (1 << 19) /* Bit 19: Oversampling Mode */
#define UART_MR_INACK                (1 << 20) /* Bit 20: Inhibit Non Acknowledge */
#define UART_MR_DSNACK               (1 << 21) /* Bit 21: Disable Successive NACK */
#define UART_MR_VARSYNC              (1 << 22) /* Bit 22: Variable Synchronization of Command/Data Sync Start Frame Delimiter */
#define UART_MR_INVDATA              (1 << 23) /* Bit 23: INverted Data */
#define UART_MR_MAXITER_SHIFT        (24)      /* Bits 24-26: Max iterations (ISO7816 T=0 */
#define UART_MR_MAXITER_MASK         (7 << UART_MR_MAXITER_SHIFT)
#define UART_MR_FILTER               (1 << 28) /* Bit 28: Infrared Receive Line Filter */
#define UART_MR_MAN                  (1 << 29) /* Bit 29: Manchester Encoder/Decoder Enable */
#define UART_MR_MODSYNC              (1 << 30) /* Bit 30: Manchester Synchronization Mode */
#define UART_MR_ONEBIT               (1 << 31) /* Bit 31: Start Frame Delimiter Selector */

/* USART Interrupt Enable Register, USART Interrupt Disable Register, USART Interrupt Mask
 * Register, and USART Status Register common bit field definitions.
 *
 * - Bits that provide interrupts with UART_INT_
 * - Bits unique to the USART status register begin with UART_SR_
 */
#define UART_INT_RXRDY               (1 << 0)  /* Bit 0:  RXRDY Interrupt */
#define UART_INT_TXRDY               (1 << 1)  /* Bit 1:  TXRDY Interrupt */
#define UART_INT_RXBRK               (1 << 2)  /* Bit 2:  Break Received/End of Break */
#define UART_INT_OVRE                (1 << 5)  /* Bit 5:  Overrun Error Interrupt */
#define UART_INT_FRAME               (1 << 6)  /* Bit 6:  Framing Error Interrupt */
#define UART_INT_PARE                (1 << 7)  /* Bit 7:  Parity Error Interrupt */
#define UART_INT_TIMEOUT             (1 << 8)  /* Bit 8:  Time-out Interrupt */
#define UART_INT_TXEMPTY             (1 << 9)  /* Bit 9:  TXEMPTY Interrupt */
#define UART_INT_ITER                (1 << 10) /* Bit 10: Iteration Interrupt */
#define UART_INT_UNRE                (1 << 10) /* Bit 10: SPI Underrun Error Interrupt */
#define UART_INT_RXBUFF              (1 << 12) /* Bit 12: Buffer Full Interrupt */
#define UART_INT_NACK                (1 << 13) /* Bit 13: Non Acknowledge Interrupt */
#define UART_INT_LINBK               (1 << 13) /* Bit 13: LIN Break */
#define UART_INT_LINID               (1 << 14) /* Bit 14: LIN Identifier */
#define UART_INT_LINTC               (1 << 15) /* Bit 15: LIN Transfer Completed */
#define UART_INT_RIIC                (1 << 16) /* Bit 16: Ring Indicator Input Change Flag */
#define UART_INT_DSRIC               (1 << 17) /* Bit 17: DSR Input Change Flag */
#define UART_INT_DCDIC               (1 << 18) /* Bit 18: DCD Input Change Flag */
#define UART_INT_CTSIC               (1 << 19) /* Bit 19: CTS Input Change Interrupt */
#define UART_SR_RI                   (1 << 20) /* Bit 20: Image of RI Input (Status only) */
#define UART_SR_DSR                  (1 << 21) /* Bit 21: Image of DSR Input (Status only) */
#define UART_SR_DCD                  (1 << 22) /* Bit 22: Image of DCD Input (Status only) */
#define UART_SR_CTS                  (1 << 23) /* Bit 23: Image of CTS Input (Status only) */
#define UART_SR_LINBLS               (1 << 23) /* Bit 23: ILIN Bus Line Status (Status only) */
#define UART_INT_MANE                (1 << 24) /* Bit 24: Manchester Error Interrupt */
#define UART_INT_LINBE               (1 << 25) /* Bit 25: LIN Bit Error */
#define UART_INT_LINISFE             (1 << 26) /* Bit 26: LIN Inconsistent Sync Field Error */
#define UART_INT_LINIPE              (1 << 27) /* Bit 27: LIN Identifier Parity Error */
#define UART_INT_LINCE               (1 << 28) /* Bit 28: LIN Checksum Error */
#define UART_INT_LINSNRE             (1 << 29) /* Bit 29: LIN Slave Not Responding Error */
#define UART_INT_LINSTE              (1 << 30) /* Bit 30: LIN Sync Tolerance Error */
#define UART_INT_LINHTE              (1 << 31) /* Bit 31: LIN Header Time-out Error */

#define UART_INT_ALLINTS             0xff0ff7e7

/* USART Receiver Holding Register */

#define UART_RHR_RXCHR_SHIFT         (0)       /* Bits 0-8: Received Character */
#define UART_RHR_RXCHR_MASK          (0x1ff << UART_RHR_RXCHR_SHIFT)
#define UART_RHR_RXSYNH              (1 << 15) /* Bit 15: Received Sync */

/* USART Transmit Holding Register */

#define UART_THR_TXCHR_SHIFT         (0)       /* Bits 0-8: Character to be Transmitted */
#define UART_THR_TXCHR_MASK          (0x1ff << UART_THR_TXCHR_SHIFT)
#define UART_THR_TXSYNH              (1 << 15) /* Bit 15: Sync Field to be tran */

/* USART Baud Rate Generator Register */

#define UART_BRGR_CD_SHIFT           (0)      /* Bits 0-15: Clock Divisor */
#define UART_BRGR_CD_MASK            (0xffff << UART_BRGR_CD_SHIFT)
#define UART_BRGR_FP_SHIFT           (16)      /* Bits 16-18: Fractional Part */
#define UART_BRGR_FP_MASK            (7 << UART_BRGR_FP_SHIFT)

/* USART Receiver Time-out Register */

#define UART_RTOR_TO_SHIFT           (0)       /* Bits 0-16: Time-out Value */
#define UART_RTOR_TO_MASK            (0x1ffff << UART_RTOR_TO_SHIFT)

/* USART Transmitter Timeguard Register */

#define UART_TTGR_TG_SHIFT           (0)       /* Bits 0-7: Timeguard Value */
#define UART_TTGR_TG_MASK            (0xff << UART_TTGR_TG_SHIFT)

/* USART FI DI RATIO Register */

#define UART_FIDI_RATIO_SHIFT        (0)       /* Bits 0-10: FI Over DI Ratio Value */
#define UART_FIDI_RATIO_MASK         (0x7ff << UART_FIDI_RATIO_SHIFT)

/* USART Number of Errors Register */

#define UART_NER_NBERRORS_SHIFT      (0)       /* Bits 0-7: Number of Errrors */
#define UART_NER_NBERRORS_MASK       (0xff << UART_NER_NBERRORS_SHIFT)

/* USART IrDA FILTER Register */

#define UART_IFR_IRDAFILTER_SHIFT    (0)       /* Bits 0-7: IrDA Filter */
#define UART_IFR_IRDAFILTER_MASK     (0xff << UART_IFR_IRDAFILTER_SHIFT)

/* USART Manchester Configuration Register */

#define UART_MAN_TXPL_SHIFT          (0)       /* Bits 0-3: Transmitter Preamble Length */
#define UART_MAN_TXPL_MASK           (15 << UART_MAN_TXPL_SHIFT)
#define UART_MAN_TXPP_SHIFT          (8)       /* Bits 8-9: Transmitter Preamble Pattern */
#define UART_MAN_TXPP_MASK           (3 << UART_MAN_TXPP_SHIFT)
#  define UART_MAN_TXPP_ALLONE       (0 << UART_MAN_TXPP_SHIFT) /* ALL_ONE */
#  define UART_MAN_TXPP_ALLZERO      (1 << UART_MAN_TXPP_SHIFT) /* ALL_ZERO */
#  define UART_MAN_TXPP_ZEROONE      (2 << UART_MAN_TXPP_SHIFT) /* ZERO_ONE */
#  define UART_MAN_TXPP_ONEZERO      (3 << UART_MAN_TXPP_SHIFT) /* ONE_ZERO */
#define UART_MAN_TXMPOL              (1 << 12) /* Bit 12: Transmitter Manchester Polarity */
#define UART_MAN_RXPL_SHIFT          (16)      /* Bits 16-19: Receiver Preamble Length */
#define UART_MAN_RXPL_MASK           (15 << UART_MAN_RXPL_SHIFT)
#define UART_MAN_RXPP_SHIFT          (24)      /* Bits 24-25: Receiver Preamble Pattern detected */
#define UART_MAN_RXPP_MASK           (3 << UART_MAN_RXPP_SHIFT)
#  define UART_MAN_RXPP_ALLONE       (0 << UART_MAN_RXPP_SHIFT) /* ALL_ONE */
#  define UART_MAN_RXPP_ALLZERO      (1 << UART_MAN_RXPP_SHIFT) /* ALL_ZERO */
#  define UART_MAN_RXPP_ZEROONE      (2 << UART_MAN_RXPP_SHIFT) /* ZERO_ONE */
#  define UART_MAN_RXPP_ONEZERO      (3 << UART_MAN_RXPP_SHIFT) /* ONE_ZERO */
#define UART_MAN_RXMPOL              (1 << 28) /* Bit 28: Receiver Manchester Polarity */
#define UART_MAN_DRIFT               (1 << 30) /* Bit 30: Drift compensation */

/* USART LIN Mode Register */

#define UART_LINMR_NACT_SHIFT        (0)       /* Bits 0-1: LIN Node Action */
#define UART_LINMR_NACT_MASK         (3 << UART_LINMR_NACT_SHIFT)
#  define UART_LINMR_NACT_PUBLISH    (0 << UART_LINMR_NACT_SHIFT) /* USART transmits response */
#  define UART_LINMR_NACT_SUBSCRIBE  (1 << UART_LINMR_NACT_SHIFT) /* USART receives response */
#  define UART_LINMR_NACT_IGNORE     (2 << UART_LINMR_NACT_SHIFT) /* USART does neither */
#define UART_LINMR_PARDIS            (1 << 2)  /* Bit 0:  Parity Disable */
#define UART_LINMR_CHKDIS            (1 << 3)  /* Bit 0:  Checksum Disable */
#define UART_LINMR_CHKTYP            (1 << 4)  /* Bit 0:  Checksum Type */
#define UART_LINMR_DLM               (1 << 5)  /* Bit 0:  Data Length Mode */
#define UART_LINMR_FSDIS             (1 << 6)  /* Bit 0:  Frame Slot Mode Disable */
#define UART_LINMR_WKUPTYP           (1 << 7)  /* Bit 0:  Wakeup Signal Type */
#define UART_LINMR_DLC_SHIFT         (8)       /* Bits 8-15: Data Length Control */
#define UART_LINMR_DLC_MASK          (0xff << UART_LINMR_DLC_SHIFT)
#define UART_LINMR_PDCM              (1 << 16) /* Bit 16: Peripheral DMA Controller Mode */
#define UART_LINMR_SYNCDIS           (1 << 17) /* Bit 17: Synchronization Disable */

/* USART LIN Identifier Register */

#define UART_LINIR_MASK              0xff      /* Bits 0-7: Identifer character */

/* USART LIN Baud Rate Register  */

#define UART_LINBR_LINCD_SHIFT       (0)       /* Bit 0-15:LIN Clock Divider after Synchronization */
#define UART_LINBR_LINCD_MASK        (0xffff << UART_LINBR_LINCD_SHIFT)
#define UART_LINBR_LINFP_SHIFT       (16)      /* Bits 16-18: LIN Fractional Part after Synchronization */
#define UART_LINBR_LINFP_MASK        (7 << UART_LINBR_LINFP_SHIFT)

/* USART Write Protect Mode Register */

#define UART_WPMR_WPEN               (1 << 0)  /* Bit 0: Write Protect Enable */
#define UART_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY */
#define UART_WPMR_WPKEY_MASK         (0x00ffffff << UART_WPMR_WPKEY_SHIFT)

/* USART Write Protect Status Register */

#define UART_WPSR_WPVS               (1 << 0)  /* Bit 0: Write Protect Violation Status */
#define UART_WPSR_WPVSRC_SHIFT       (8)       /* Bits 8-23: Write Protect Violation Source */
#define UART_WPSR_WPVSRC_MASK        (0xffff << UART_WPSR_WPVSRC_SHIFT)

/* USART Version Register */

#define UART_VERSION_VERSION_SHIFT   (0)       /* Bits 0-11: Macrocell version number */
#define UART_VERSION_VERSION_MASK    (0xfff << UART_VERSION_VERSION_SHIFT)
#define UART_VERSION_MFN_SHIFT       (16)      /* Bits 16-18: Reserved */
#define UART_VERSION_MFN_MASK        (7 << UART_VERSION_MFN_SHIFT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_UART_H */
