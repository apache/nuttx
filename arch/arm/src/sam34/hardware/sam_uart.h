/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_uart.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_UART_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART register offsets ****************************************************/

#define SAM_UART_CR_OFFSET           0x0000 /* Control Register (Common) */
#define SAM_UART_MR_OFFSET           0x0004 /* Mode Register (Common) */
#define SAM_UART_IER_OFFSET          0x0008 /* Interrupt Enable Register (Common) */
#define SAM_UART_IDR_OFFSET          0x000c /* Interrupt Disable Register (Common) */
#define SAM_UART_IMR_OFFSET          0x0010 /* Interrupt Mask Register (Common) */
#define SAM_UART_SR_OFFSET           0x0014 /* [Channel] Status Register (Common) */
#define SAM_UART_RHR_OFFSET          0x0018 /* Receive Holding Register (Common) */
#define SAM_UART_THR_OFFSET          0x001c /* Transmit Holding Register (Common) */
#define SAM_UART_BRGR_OFFSET         0x0020 /* Baud Rate Generator Register (Common) */
                                            /* 0x0024-0x003c: Reserved (UART) */
#define SAM_UART_RTOR_OFFSET         0x0024 /* Receiver Time-out Register (USART only) */
#define SAM_UART_TTGR_OFFSET         0x0028 /* Transmitter Timeguard Register (USART only) */
                                            /* 0x002c-0x003c: Reserved (USART) */
#define SAM_UART_FIDI_OFFSET         0x0040 /* FI DI Ratio Register (USART only) */
#define SAM_UART_NER_OFFSET          0x0044 /* Number of Errors Register ((USART only) */
                                            /* 0x0048: Reserved (USART) */
#define SAM_UART_IFR_OFFSET          0x004c /* IrDA Filter Register (USART only) */
#define SAM_UART_MAN_OFFSET          0x0050 /* Manchester Encoder Decoder Register (USART only) */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_UART_LINMR_OFFSET      0x0054 /* LIN Mode Register (USART only) */
#  define SAM_UART_LINIR_OFFSET      0x0058 /* LIN Identifier Register (USART only) */
#endif

#define SAM_UART_WPMR_OFFSET         0x00e4 /* Write Protect Mode Register (USART only) */
#define SAM_UART_WPSR_OFFSET         0x00e8 /* Write Protect Status Register (USART only) */
                                            /* 0x005c-0xf8: Reserved (USART) */
#define SAM_UART_VERSION_OFFSET      0x00fc /* Version Register (USART only, Not SAM4E) */
                                            /* 0x0100-0x0124: PDC Area (Common) */

/* UART register addresses **************************************************/

#define SAM_UART0_CR                 (SAM_UART0_BASE+SAM_UART_CR_OFFSET)
#define SAM_UART0_MR                 (SAM_UART0_BASE+SAM_UART_MR_OFFSET)
#define SAM_UART0_IER                (SAM_UART0_BASE+SAM_UART_IER_OFFSET)
#define SAM_UART0_IDR                (SAM_UART0_BASE+SAM_UART_IDR_OFFSET)
#define SAM_UART0_IMR                (SAM_UART0_BASE+SAM_UART_IMR_OFFSET)
#define SAM_UART0_SR                 (SAM_UART0_BASE+SAM_UART_SR_OFFSET)
#define SAM_UART0_RHR                (SAM_UART0_BASE+SAM_UART_RHR_OFFSET)
#define SAM_UART0_THR                (SAM_UART0_BASE+SAM_UART_THR_OFFSET)
#define SAM_UART0_BRGR               (SAM_UART0_BASE+SAM_UART_BRGR_OFFSET)

#define SAM_UART1_CR                 (SAM_UART1_BASE+SAM_UART_CR_OFFSET)
#define SAM_UART1_MR                 (SAM_UART1_BASE+SAM_UART_MR_OFFSET)
#define SAM_UART1_IER                (SAM_UART1_BASE+SAM_UART_IER_OFFSET)
#define SAM_UART1_IDR                (SAM_UART1_BASE+SAM_UART_IDR_OFFSET)
#define SAM_UART1_IMR                (SAM_UART1_BASE+SAM_UART_IMR_OFFSET)
#define SAM_UART1_SR                 (SAM_UART1_BASE+SAM_UART_SR_OFFSET)
#define SAM_UART1_RHR                (SAM_UART1_BASE+SAM_UART_RHR_OFFSET)
#define SAM_UART1_THR                (SAM_UART1_BASE+SAM_UART_THR_OFFSET)
#define SAM_UART1_BRGR               (SAM_UART1_BASE+SAM_UART_BRGR_OFFSET)

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

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_USART_LINMR(n)         (SAM_USARTN_BASE(n)+SAM_UART_LINMR_OFFSET)
#  define SAM_USART_LINIR(n)         (SAM_USARTN_BASE(n)+SAM_UART_LINIR_OFFSET)
#endif

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

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_USART0_LINMR           (SAM_USART0_BASE+SAM_UART_LINMR_OFFSET)
#  define SAM_USART0_LINIR           (SAM_USART0_BASE+SAM_UART_LINIR_OFFSET)
#endif

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

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_USART1_LINMR           (SAM_USART1_BASE+SAM_UART_LINMR_OFFSET)
#  define SAM_USART1_LINIR           (SAM_USART1_BASE+SAM_UART_LINIR_OFFSET)
#endif

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

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_USART2_LINMR           (SAM_USART2_BASE+SAM_UART_LINMR_OFFSET)
#  define SAM_USART2_LINIR           (SAM_USART2_BASE+SAM_UART_LINIR_OFFSET)
#endif

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

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_USART3_LINMR           (SAM_USART3_BASE+SAM_UART_LINMR_OFFSET)
#  define SAM_USART3_LINIR           (SAM_USART3_BASE+SAM_UART_LINIR_OFFSET)
#endif

#define SAM_USART3_WPMR              (SAM_USART3_BASE+SAM_UART_WPMR_OFFSET)
#define SAM_USART3_WPSR              (SAM_USART3_BASE+SAM_UART_WPSR_OFFSET)
#define SAM_USART3_VERSION           (SAM_USART3_BASE+SAM_UART_VERSION_OFFSET)

/* UART register bit definitions ********************************************/

/* UART Control Register */

#define UART_CR_RSTRX                (1 << 2)  /* Bit 2:  Reset Receiver (Common) */
#define UART_CR_RSTTX                (1 << 3)  /* Bit 3:  Reset Transmitter (Common) */
#define UART_CR_RXEN                 (1 << 4)  /* Bit 4:  Receiver Enable (Common) */
#define UART_CR_RXDIS                (1 << 5)  /* Bit 5:  Receiver Disable (Common) */
#define UART_CR_TXEN                 (1 << 6)  /* Bit 6:  Transmitter Enable (Common) */
#define UART_CR_TXDIS                (1 << 7)  /* Bit 7:  Transmitter Disable (Common) */
#define UART_CR_RSTSTA               (1 << 8)  /* Bit 8:  Reset Status Bits (Common) */
#define UART_CR_STTBRK               (1 << 9)  /* Bit 9:  Start Break (USART only) */
#define UART_CR_STPBRK               (1 << 10) /* Bit 10: Stop Break (USART only) */
#define UART_CR_STTTO                (1 << 11) /* Bit 11: Start Time-out (USART only) */
#define UART_CR_SENDA                (1 << 12) /* Bit 12: Send Address (USART only) */
#define UART_CR_RSTIT                (1 << 13) /* Bit 13: Reset Iterations (USART only) */
#define UART_CR_RSTNACK              (1 << 14) /* Bit 14: Reset Non Acknowledge (USART only) */
#define UART_CR_RETTO                (1 << 15) /* Bit 15: Rearm Time-out (USART only) */
#define UART_CR_RTSEN                (1 << 18) /* Bit 18: Request to Send Enable (USART only) */
#define UART_CR_FCS                  (1 << 18) /* Bit 18: Force SPI Chip Select (USART SPI mode only) */
#define UART_CR_RTSDIS               (1 << 19) /* Bit 19: Request to Send Disable (USART only) */
#define UART_CR_RCS                  (1 << 19) /* Bit 19: Release SPI Chip Select (USART SPI mode only) */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define UART_CR_LINABT             (1 << 20) /* Bit 20: Abort LIN Transmission */
#  define UART_CR_LINWKUP            (1 << 21) /* Bit 21: Send LIN Wakeup Signal */
#endif

/* UART Mode Register and USART Mode Register (UART MODE) */

#ifdef CONFIG_ARCH_CHIP_SAM4CM
#  define UART_MR_OPT_EN             (1 << 0)  /* Bit 0: UART Optical Interface Enable (UART only) */
#  define UART_MR_OPT_RXINV          (1 << 1)  /* Bit 1: UART Receive Data Inverted (UART only) */
#  define UART_MR_OPT_MDINV          (1 << 2)  /* Bit 2: UART Modulated Data Inverted (UART only) */
#endif

#define UART_MR_MODE_SHIFT           (0)       /* Bits 0-3: (USART only) */
#define UART_MR_MODE_MASK            (15 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_NORMAL        (0  << UART_MR_MODE_SHIFT) /* Normal */
#  define UART_MR_MODE_RS485         (1  << UART_MR_MODE_SHIFT) /* RS485 */
#  define UART_MR_MODE_HWHS          (2  << UART_MR_MODE_SHIFT) /* Hardware Handshaking */
#  define UART_MR_MODE_ISO7816_0     (4  << UART_MR_MODE_SHIFT) /* IS07816 Protocol: T = 0 */
#  define UART_MR_MODE_ISO7816_1     (6  << UART_MR_MODE_SHIFT) /* IS07816 Protocol: T = 1 */
#  define UART_MR_MODE_IRDA          (8  << UART_MR_MODE_SHIFT) /* IrDA */
#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define UART_MR_MODE_LINMSTR       (10 << UART_MR_MODE_SHIFT) /* LIN Master */
#  define UART_MR_MODE_LINSLV        (11 << UART_MR_MODE_SHIFT) /* LIN Slave */
#endif
#  define UART_MR_MODE_SPIMSTR       (14 << UART_MR_MODE_SHIFT) /* SPI Master (SPI mode only) */
#  define UART_MR_MODE_SPISLV        (15 << UART_MR_MODE_SHIFT) /* SPI Slave (SPI mode only) */

#define UART_MR_USCLKS_SHIFT         (4)       /* Bits 4-5: Clock Selection (USART only) */
#define UART_MR_USCLKS_MASK          (3 << UART_MR_USCLKS_SHIFT)
#  define UART_MR_USCLKS_MCK         (0 << UART_MR_USCLKS_SHIFT) /* MCK */
#  define UART_MR_USCLKS_MCKDIV      (1 << UART_MR_USCLKS_SHIFT) /* MCK/DIV (DIV = 8) */
#  define UART_MR_USCLKS_SCK         (3 << UART_MR_USCLKS_SHIFT) /* SCK */

#define UART_MR_CHRL_SHIFT           (6)       /* Bits 6-7: Character Length (USART only) */
#define UART_MR_CHRL_MASK            (3 << UART_MR_CHRL_SHIFT)
#  define UART_MR_CHRL_5BITS         (0 << UART_MR_CHRL_SHIFT) /* 5 bits */
#  define UART_MR_CHRL_6BITS         (1 << UART_MR_CHRL_SHIFT) /* 6 bits */
#  define UART_MR_CHRL_7BITS         (2 << UART_MR_CHRL_SHIFT) /* 7 bits */
#  define UART_MR_CHRL_8BITS         (3 << UART_MR_CHRL_SHIFT) /* 8 bits */

#define UART_MR_SYNC                 (1 << 8)  /* Bit 8: Synchronous Mode Select (USART only) */
#define UART_MR_CPHA                 (1 << 8)  /* Bit 8: SPI Clock Phase (USART SPI mode only) */
#define UART_MR_PAR_SHIFT            (9)       /* Bits 9-11: Parity Type (Common) */
#define UART_MR_PAR_MASK             (7 << UART_MR_PAR_SHIFT)
#  define UART_MR_PAR_EVEN           (0 << UART_MR_PAR_SHIFT) /* Even parity (Common) */
#  define UART_MR_PAR_ODD            (1 << UART_MR_PAR_SHIFT) /* Odd parity (Common) */
#  define UART_MR_PAR_SPACE          (2 << UART_MR_PAR_SHIFT) /* Space: parity forced to 0 (Common) */
#  define UART_MR_PAR_MARK           (3 << UART_MR_PAR_SHIFT) /* Mark: parity forced to 1 (Common) */
#  define UART_MR_PAR_NONE           (4 << UART_MR_PAR_SHIFT) /* No parity (Common) */
#  define UART_MR_PAR_MULTIDROP      (6 << UART_MR_PAR_SHIFT) /* Multidrop mode (USART only) */

#define UART_MR_NBSTOP_SHIFT         (12)      /* Bits 12-13: Number of Stop Bits (USART only) */
#define UART_MR_NBSTOP_MASK          (3 << UART_MR_NBSTOP_SHIFT)
#  define UART_MR_NBSTOP_1           (0 << UART_MR_NBSTOP_SHIFT) /* 1 stop bit 1 stop bit */
#  define UART_MR_NBSTOP_1p5         (1 << UART_MR_NBSTOP_SHIFT) /* 1.5 stop bits */
#  define UART_MR_NBSTOP_2           (2 << UART_MR_NBSTOP_SHIFT) /* 2 stop bits 2 stop bits */

#define UART_MR_CHMODE_SHIFT         (14)      /* Bits 14-15: Channel Mode (Common) */
#define UART_MR_CHMODE_MASK          (3 << UART_MR_CHMODE_SHIFT)
#  define UART_MR_CHMODE_NORMAL      (0 << UART_MR_CHMODE_SHIFT) /* Normal Mode */
#  define UART_MR_CHMODE_ECHO        (1 << UART_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define UART_MR_CHMODE_LLPBK       (2 << UART_MR_CHMODE_SHIFT) /* Local Loopback */
#  define UART_MR_CHMODE_RLPBK       (3 << UART_MR_CHMODE_SHIFT) /* Remote Loopback */
#ifdef CONFIG_ARCH_CHIP_SAM4CM
#  define UART_MR_OPT_CLKDIV_SHIFT   (16)      /* Bits 16-20: Optical Link Clock Divider (UART only) */
#  define UART_MR_OPT_CLKDIV_MASK    (31 << UART_MR_OPT_CLKDIV_SHIFT)
#endif
#define UART_MR_MSBF                 (1 << 16) /* Bit 16: Most Significant Bit first (USART only) */
#define UART_MR_CPOL                 (1 << 16) /* Bit 16: SPI Clock Polarity (USART SPI mode only) */
#define UART_MR_MODE9                (1 << 17) /* Bit 17: 9-bit Character Length (USART only) */
#define UART_MR_CLKO                 (1 << 18) /* Bit 18: Clock Output Select (USART only) */
#define UART_MR_OVER                 (1 << 19) /* Bit 19: Oversampling Mode (USART only) */
#define UART_MR_INACK                (1 << 20) /* Bit 20: Inhibit Non Acknowledge (USART only) */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define UART_MR_WRDBT              (1 << 20) /* Bit 20: Wait Read Data Before Transfer (SPI mode only) */
#endif

#define UART_MR_DSNACK               (1 << 21) /* Bit 21: Disable Successive NACK (USART only) */
#define UART_MR_VARSYNC              (1 << 22) /* Bit 22: Variable Synchronization of Command/Data Sync Start Frame Delimiter (USART only) */
#define UART_MR_INVDATA              (1 << 23) /* Bit 23: INverted Data (USART only) */

#ifdef CONFIG_ARCH_CHIP_SAM4CM
#  define UART_MR_OPT_OPT_DUTY_SHIFT   (24)      /* Bits 24-26: Optical Link Modulation Clock Duty Cycle (UART only) */
#  define UART_MR_OPT_OPT_DUTY_MASK    (7 << UART_MR_OPT_OPT_DUTY_SHIFT)
#    define UART_MR_OPT_OPT_DUTY_50    (0 << UART_MR_OPT_OPT_DUTY_SHIFT)
#    define UART_MR_OPT_OPT_DUTY_43P75 (1 << UART_MR_OPT_OPT_DUTY_SHIFT)
#    define UART_MR_OPT_OPT_DUTY_37P5  (2 << UART_MR_OPT_OPT_DUTY_SHIFT)
#    define UART_MR_OPT_OPT_DUTY_31P25 (3 << UART_MR_OPT_OPT_DUTY_SHIFT)
#    define UART_MR_OPT_OPT_DUTY_25    (4 << UART_MR_OPT_OPT_DUTY_SHIFT)
#    define UART_MR_OPT_OPT_DUTY_18P75 (5 << UART_MR_OPT_OPT_DUTY_SHIFT)
#    define UART_MR_OPT_OPT_DUTY_12P5  (6 << UART_MR_OPT_OPT_DUTY_SHIFT)
#    define UART_MR_OPT_OPT_DUTY_6P25  (7 << UART_MR_OPT_OPT_DUTY_SHIFT)
#  define UART_MR_OPT_CMPTH_SHIFT          (28)
#  define UART_MR_OPT_CMPTH_MASK           (7 << UART_MR_OPT_CMPTH_SHIFT)
#    define UART_MR_OPT_CMPTH_VDDIO_DIV2   (0 << UART_MR_OPT_CMPTH_SHIFT)
#    define UART_MR_OPT_CMPTH_VDDIO_DIV2P5 (1 << UART_MR_OPT_CMPTH_SHIFT)
#    define UART_MR_OPT_CMPTH_VDDIO_DIV3P3 (2 << UART_MR_OPT_CMPTH_SHIFT)
#    define UART_MR_OPT_CMPTH_VDDIO_DIV5   (3 << UART_MR_OPT_CMPTH_SHIFT)
#    define UART_MR_OPT_CMPTH_VDDIO_DIV10  (4 << UART_MR_OPT_CMPTH_SHIFT)
#endif

#define UART_MR_MAXITER_SHIFT        (24)      /* Bits 24-26: Max iterations (ISO7816 T=0 (USART only) */
#define UART_MR_MAXITER_MASK         (7 << UART_MR_MAXITER_SHIFT)
#  define UART_MR_MAXITER(n)         ((uint32_t)(n) << UART_MR_MAXITER_SHIFT)
#define UART_MR_FILTER               (1 << 28) /* Bit 28: Infrared Receive Line Filter (USART only) */
#define UART_MR_MAN                  (1 << 29) /* Bit 29: Manchester Encoder/Decoder Enable (USART only) */
#define UART_MR_MODSYNC              (1 << 30) /* Bit 30: Manchester Synchronization Mode (USART only) */
#define UART_MR_ONEBIT               (1 << 31) /* Bit 31: Start Frame Delimiter Selector (USART only) */

/* UART Interrupt Enable Register,
 * UART Interrupt Disable Register, UART Interrupt Mask
 * Register, and UART Status Register common bit field definitions
 */

#define UART_INT_RXRDY               (1 << 0)  /* Bit 0:  RXRDY Interrupt (Common) */
#define UART_INT_TXRDY               (1 << 1)  /* Bit 1:  TXRDY Interrupt (Common) */
#define UART_INT_RXBRK               (1 << 2)  /* Bit 2:  Break Received/End of Break */
#define UART_INT_ENDRX               (1 << 3)  /* Bit 3:  End of Receive Transfer Interrupt (Common) */
#define UART_INT_ENDTX               (1 << 4)  /* Bit 4:  End of Transmit Interrupt (Common) */
#define UART_INT_OVRE                (1 << 5)  /* Bit 5:  Overrun Error Interrupt (Common) */
#define UART_INT_FRAME               (1 << 6)  /* Bit 6:  Framing Error Interrupt (Common) */
#define UART_INT_PARE                (1 << 7)  /* Bit 7:  Parity Error Interrupt (Common) */
#define UART_INT_TIMEOUT             (1 << 8)  /* Bit 8:  Time-out Interrupt (USART only) */
#define UART_INT_TXEMPTY             (1 << 9)  /* Bit 9:  TXEMPTY Interrupt (Common) */
#define UART_INT_ITER                (1 << 10) /* Bit 10: Iteration Interrupt (USART only) */
#define UART_INT_UNRE                (1 << 10) /* Bit 10: SPI Underrun Error Interrupt (USART SPI mode only) */
#define UART_INT_TXBUFE              (1 << 11) /* Bit 11: Buffer Empty Interrupt (Common) */
#define UART_INT_RXBUFF              (1 << 12) /* Bit 12: Buffer Full Interrupt (Common) */
#define UART_INT_NACK                (1 << 13) /* Bit 13: Non Acknowledge Interrupt (USART only) */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define UART_INT_LINBK             (1 << 13) /* Bit 13: LIN Break Sent or Break Received Interrupt */
#  define UART_INT_LINID             (1 << 14) /* Bit 14: LIN Identifier Sent or Identifier Received Interrupt */
#  define UART_INT_LINTC             (1 << 15) /* Bit 15: LIN Transfer Completed Interrupt */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define UART_INT_RIIC              (1 << 16) /* Bit 16: Ring Indicator Input Change */
#  define UART_INT_DSRIC             (1 << 17) /* Bit 17: Data Set Ready Input Change */
#  define UART_INT_DCDIC             (1 << 18) /* Bit 18: Data Carrier Detect Input Change Interrupt */
#endif

#define UART_INT_CTSIC               (1 << 19) /* Bit 19: Clear to Send Input Change Interrupt (USART only) */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define UART_SR_RI                 (1 << 20) /* Bit 20: Image of RI Input */
#  define UART_SR_DSR                (1 << 21) /* Bit 21: Image of DSR Input */
#  define UART_SR_DCD                (1 << 22) /* Bit 22: Image of DCD Input */
#  define UART_SR_CTS                (1 << 23) /* Bit 23: Image of CTS Input */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define UART_SR_CTS                (1 << 23) /* Bit 23: Image of CTS Input */
#  define UART_SR_LINBLS             (1 << 23) /* Bit 23: LIN Bus Line Status */
#endif

#define UART_INT_MANE                (1 << 24) /* Bit 24: Manchester Error Interrupt (USART only) */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define UART_INT_LINBE             (1 << 25) /* Bit 25: LIN Bus Error Interrupt */
#  define UART_INT_LINISFE           (1 << 26) /* Bit 26: LIN Inconsistent Synch Field Error Interrupt */
#  define UART_INT_LINIPE            (1 << 27) /* Bit 27: LIN Identifier Parity Interrupt */
#  define UART_INT_LINCE             (1 << 28) /* Bit 28: LIN Checksum Error Interrupt */
#  define UART_INT_LINSNRE           (1 << 29) /* Bit 29: LIN Slave Not Responding Error Interrupt */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define UART_INT_ALLINTS           0x3f08ffff
#elif defined(CONFIG_ARCH_CHIP_SAM4S)
#  define UART_INT_ALLINTS           0x010f3fff /* USART - UART only has 0x001AFB? does it matter? */
#else
#  define UART_INT_ALLINTS           0x01083fff
#endif

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

#define UART_BRGR_CD_SHIFT           (0)      /* Bits 0-15: Clock Divisor (Common) */
#define UART_BRGR_CD_MASK            (0xffff << UART_BRGR_CD_SHIFT)
#define UART_BRGR_FP_SHIFT           (16)      /* Bits 16-18: Fractional Part (USART only) */
#define UART_BRGR_FP_MASK            (7 << UART_BRGR_FP_SHIFT)

/* USART Receiver Time-out Register (USART only) */

#define UART_RTOR_TO_SHIFT           (0)       /* Bits 0-15: Time-out Value (USART only) */
#define UART_RTOR_TO_MASK            (0xffff << UART_RTOR_TO_SHIFT)

/* USART Transmitter Timeguard Register (USART only) */

#define UART_TTGR_TG_SHIFT           (0)       /* Bits 0-7: Timeguard Value (USART only) */
#define UART_TTGR_TG_MASK            (0xff << UART_TTGR_TG_SHIFT)

/* USART FI DI RATIO Register (USART only) */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define UART_FIDI_RATIO_SHIFT      (0)       /* Bits 0-15: FI Over DI Ratio Value (USART only) */
#  define UART_FIDI_RATIO_MASK       (0xffff << UART_FIDI_RATIO_SHIFT)
#else
#  define UART_FIDI_RATIO_SHIFT      (0)       /* Bits 0-10: FI Over DI Ratio Value (USART only) */
#  define UART_FIDI_RATIO_MASK       (0x7ff << UART_FIDI_RATIO_SHIFT)
#endif

/* USART Number of Errors Register (USART only) */

#define UART_NER_NBERRORS_SHIFT      (0)       /* Bits 0-7: Number of Errors (USART only) */
#define UART_NER_NBERRORS_MASK       (0xff << UART_NER_NBERRORS_SHIFT)

/* USART IrDA FILTER Register (USART only) */

#define UART_IFR_IRDAFILTER_SHIFT    (0)       /* Bits 0-7: IrDA Filter (USART only) */
#define UART_IFR_IRDAFILTER_MASK     (0xff << UART_IFR_IRDAFILTER_SHIFT)

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

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define UART_MAN_ONE               (1 << 29) /* Bit 29: Must Be Set to 1 */
#endif

#define UART_MAN_DRIFT               (1 << 30) /* Bit 30: Drift compensation (USART only) */

/* LIN Mode Register (USART only) */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define UART_LINMR_NACT_SHIFT       (0)       /* Bits 0-1: LIN Node Action */
#  define UART_LINMR_NACT_MASK        (3 << UART_LINMR_NACT_SHIFT)
#    define UART_LINMR_NACT_PUBLISH   (0 << UART_LINMR_NACT_SHIFT) /* USART transmits response */
#    define UART_LINMR_NACT_SUBSCRIBE (1 << UART_LINMR_NACT_SHIFT) /* USART receives response */
#    define UART_LINMR_NACT_IGNORE    (2 << UART_LINMR_NACT_SHIFT) /* USART does not transmit or receive response */

#  define UART_LINMR_PARDIS          (1 << 2)  /* Bit 2:  Parity Disable */
#  define UART_LINMR_CHKDIS          (1 << 3)  /* Bit 3:  Checksum Disable */
#  define UART_LINMR_CHKTYP          (1 << 4)  /* Bit 4:  Checksum Type */
#  define UART_LINMR_DLM             (1 << 5)  /* Bit 5:  Data Length Mode */
#  define UART_LINMR_FSDIS           (1 << 6)  /* Bit 6:  Frame Slot Mode Disable */
#  define UART_LINMR_WKUPTYP         (1 << 7)  /* Bit 7:  Wakeup Signal Type */
#  define UART_LINMR_DLC_SHIFT       (8)       /* Bits 8-15: Data Length Control */
#  define UART_LINMR_DLC_MASK        (0xff << UART_LINMR_DLC_SHIFT)
#    define UART_LINMR_DLC(n)        ((uint32_t)(n) << UART_LINMR_DLC_SHIFT)
#  define UART_LINMR_PDCM            (1 << 16) /* Bit 16: PDC Mode */
#endif

/* LIN Identifier Register (USART only) */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define UART_LINIR_MASK            0xff      /* Bits 0-7: Identifier Character */
#endif

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

/* USART Version Register */

#define UART_VERSION_VERSION_SHIFT   (0)       /* Bits 0-11: Macrocell version number (USART only) */
#define UART_VERSION_VERSION_MASK    (0xfff << UART_VERSION_VERSION_SHIFT)
#define UART_VERSION_MFN_SHIFT       (16)      /* Bits 16-18: Reserved (USART only) */
#define UART_VERSION_MFN_MASK        (7 << UART_VERSION_MFN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_UART_H */
