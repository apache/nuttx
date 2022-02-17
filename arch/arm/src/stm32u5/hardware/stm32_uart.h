/****************************************************************************
 * arch/arm/src/stm32u5/hardware/stm32_uart.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_UART_H
#define __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_USART_CR1_OFFSET    0x0000  /* Control register 1 */
#define STM32_USART_CR2_OFFSET    0x0004  /* Control register 2 */
#define STM32_USART_CR3_OFFSET    0x0008  /* Control register 3 */
#define STM32_USART_BRR_OFFSET    0x000c  /* Baud Rate register */
#define STM32_USART_GTPR_OFFSET   0x0010  /* Guard time and prescaler register */
#define STM32_USART_RTOR_OFFSET   0x0014  /* Receiver timeout register */
#define STM32_USART_RQR_OFFSET    0x0018  /* Request register */
#define STM32_USART_ISR_OFFSET    0x001c  /* Interrupt and status register */
#define STM32_USART_ICR_OFFSET    0x0020  /* Interrupt flag clear register */
#define STM32_USART_RDR_OFFSET    0x0024  /* Receive Data register */
#define STM32_USART_TDR_OFFSET    0x0028  /* Transmit Data register */
#define STM32_USART_PRESC_OFFSET  0x002c  /* Prescaler register */
#define STM32_USART_AUTOCR_OFFSET 0x0030  /* Autonomous mode control register */

/* Register Addresses *******************************************************/

#if STM32_NUSART > 0
#  define STM32_USART1_CR1        (STM32_USART1_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART1_CR2        (STM32_USART1_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART1_CR3        (STM32_USART1_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART1_BRR        (STM32_USART1_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART1_GTPR       (STM32_USART1_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART1_RTOR       (STM32_USART1_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART1_RQR        (STM32_USART1_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART1_ISR        (STM32_USART1_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART1_ICR        (STM32_USART1_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART1_RDR        (STM32_USART1_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART1_TDR        (STM32_USART1_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART1_PRESC      (STM32_USART1_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if STM32_NUSART > 1
#  define STM32_USART2_CR1        (STM32_USART2_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART2_CR2        (STM32_USART2_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART2_CR3        (STM32_USART2_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART2_BRR        (STM32_USART2_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART2_GTPR       (STM32_USART2_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART2_RTOR       (STM32_USART2_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART2_RQR        (STM32_USART2_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART2_ISR        (STM32_USART2_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART2_ICR        (STM32_USART2_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART2_RDR        (STM32_USART2_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART2_TDR        (STM32_USART2_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART2_PRESC      (STM32_USART2_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if STM32_NUSART > 2
#  define STM32_USART3_CR1        (STM32_USART3_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART3_CR2        (STM32_USART3_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART3_CR3        (STM32_USART3_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART3_BRR        (STM32_USART3_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART3_GTPR       (STM32_USART3_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART3_RTOR       (STM32_USART3_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART3_RQR        (STM32_USART3_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART3_ISR        (STM32_USART3_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART3_ICR        (STM32_USART3_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART3_RDR        (STM32_USART3_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART3_TDR        (STM32_USART3_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART3_PRESC      (STM32_USART3_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if STM32_NUSART > 3
#  define STM32_UART4_CR1         (STM32_UART4_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_UART4_CR2         (STM32_UART4_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_UART4_CR3         (STM32_UART4_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_UART4_BRR         (STM32_UART4_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_UART4_GTPR        (STM32_UART4_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_UART4_RTOR        (STM32_UART4_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_UART4_RQR         (STM32_UART4_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_UART4_ISR         (STM32_UART4_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_UART4_ICR         (STM32_UART4_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_UART4_RDR         (STM32_UART4_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_UART4_TDR         (STM32_UART4_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_UART4_PRESC       (STM32_UART4_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if STM32_NUSART > 4
#  define STM32_UART5_CR1         (STM32_UART5_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_UART5_CR2         (STM32_UART5_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_UART5_CR3         (STM32_UART5_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_UART5_BRR         (STM32_UART5_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_UART5_GTPR        (STM32_UART5_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_UART5_RTOR        (STM32_UART5_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_UART5_RQR         (STM32_UART5_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_UART5_ISR         (STM32_UART5_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_UART5_ICR         (STM32_UART5_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_UART5_RDR         (STM32_UART5_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_UART5_TDR         (STM32_UART5_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_UART5_PRESC       (STM32_UART5_BASE + STM32_USART_PRESC_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Control register 1 */

#define USART_CR1_UE              (1 << 0)  /* Bit 0: USART Enable */
#define USART_CR1_UESM            (1 << 1)  /* Bit 1: USART Enable in Stop mode */
#define USART_CR1_RE              (1 << 2)  /* Bit 2: Receiver Enable */
#define USART_CR1_TE              (1 << 3)  /* Bit 3: Transmitter Enable */
#define USART_CR1_IDLEIE          (1 << 4)  /* Bit 4: IDLE Interrupt Enable */
#define USART_CR1_RXNEIE          (1 << 5)  /* Bit 5: RXNE Interrupt Enable */
#define USART_CR1_TCIE            (1 << 6)  /* Bit 6: Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE           (1 << 7)  /* Bit 7: TXE Interrupt Enable */
#define USART_CR1_PEIE            (1 << 8)  /* Bit 8: PE Interrupt Enable */
#define USART_CR1_PS              (1 << 9)  /* Bit 9: Parity Selection */
#define USART_CR1_PCE             (1 << 10) /* Bit 10: Parity Control Enable */
#define USART_CR1_WAKE            (1 << 11) /* Bit 11: Wakeup method */
#define USART_CR1_M0              (1 << 12) /* Bit 12: Word length */
#define USART_CR1_MME             (1 << 13) /* Bit 13: Mute mode enable */
#define USART_CR1_CMIE            (1 << 14) /* Bit 14: Character match interrupt enable */
#define USART_CR1_OVER8           (1 << 15) /* Bit 15: Oversampling mode */

#define USART_CR1_DEDT_SHIFT      (16)      /* Bits 16..20 DE deactivation delay */
#define USART_CR1_DEDT_MASK       (0x1f << USART_CR1_DEDT_SHIFT)

#define USART_CR1_DEAT_SHIFT      (21)      /* Bits 21..25 DE activation delay */
#define USART_CR1_DEAT_MASK       (0x1f << USART_CR1_DEAT_SHIFT)

#define USART_CR1_RTOIE           (1 << 26) /* Bit 26: Receiver timeout interrupt enable */
#define USART_CR1_EOBIE           (1 << 27) /* Bit 27: End of block interrupt enable */
#define USART_CR1_M1              (1 << 28) /* Bit 28: Word length */

#define USART_CR1_ALLINTS         (USART_CR1_IDLEIE|USART_CR1_RXNEIE| \
                                   USART_CR1_TCIE|USART_CR1_TXEIE| \
                                   USART_CR1_PEIE|USART_CR1_CMIE| \
                                   USART_CR1_RTOIE|USART_CR1_EOBIE)

/* Control register 2 */

#define USART_CR2_ADDM7           (1 << 4)  /* Bit 4: 7-bit/4-bit Address Detection */
#define USART_CR2_LBDL            (1 << 5)  /* Bit 5: LIN Break Detection Length */
#define USART_CR2_LBDIE           (1 << 6)  /* Bit 6: LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL            (1 << 8)  /* Bit 8: Last Bit Clock pulse */
#define USART_CR2_CPHA            (1 << 9)  /* Bit 9: Clock Phase */
#define USART_CR2_CPOL            (1 << 10) /* Bit 10: Clock Polarity */
#define USART_CR2_CLKEN           (1 << 11) /* Bit 11: Clock Enable */

#define USART_CR2_STOP_SHIFT      (12)      /* Bits 13-12: STOP bits */
#define USART_CR2_STOP_MASK       (3 << USART_CR2_STOP_SHIFT)
#  define USART_CR2_STOP1         (0 << USART_CR2_STOP_SHIFT) /* 00: 1 Stop bit */
#  define USART_CR2_STOP0p5       (1 << USART_CR2_STOP_SHIFT) /* 01: 0.5 Stop bit */
#  define USART_CR2_STOP2         (2 << USART_CR2_STOP_SHIFT) /* 10: 2 Stop bits */
#  define USART_CR2_STOP1p5       (3 << USART_CR2_STOP_SHIFT) /* 11: 1.5 Stop bit */

#define USART_CR2_LINEN           (1 << 14) /* Bit 14: LIN mode enable */
#define USART_CR2_SWAP            (1 << 15) /* Bit 15: Swap TX/RX pins */
#define USART_CR2_RXINV           (1 << 16) /* Bit 16: RX pin active level inversion */
#define USART_CR2_TXINV           (1 << 17) /* Bit 17: TX pin active level inversion */
#define USART_CR2_DATAINV         (1 << 18) /* Bit 18: Binary data inversion */
#define USART_CR2_MSBFIRST        (1 << 19) /* Bit 19: Most significant bit first */
#define USART_CR2_ABREN           (1 << 20) /* Bit 20: Auto Baud rate enable */

#define USART_CR2_ABRMOD_SHIFT    (21)      /* Bits 21-22: Autobaud rate mode*/
#define USART_CR2_ABRMOD_MASK     (3 << USART_CR2_ABRMOD_SHIFT)
#define USART_CR2_ABRMOD_START    (0 << USART_CR2_ABRMOD_SHIFT) /* 00: Start bit */
#define USART_CR2_ABRMOD_EDGES    (1 << USART_CR2_ABRMOD_SHIFT) /* 01: Falling-to-falling edge -> frame must start with 10xxxxxx */
#define USART_CR2_ABRMOD_7F       (2 << USART_CR2_ABRMOD_SHIFT) /* 10: 0x7F */
#define USART_CR2_ABRMOD_55       (3 << USART_CR2_ABRMOD_SHIFT) /* 11: 0x55 */

#define USART_CR2_RTOEN           (1 << 23) /* Bit 23: Receiver timeout enable */

#define USART_CR2_ADD_SHIFT       (24)      /* Bits 24-31: Address of the USART node */
#define USART_CR2_ADD_MASK        (0xff << USART_CR2_ADD_SHIFT)

/* Control register 3 */

#define USART_CR3_EIE             (1 << 0)                   /* Bit 0: Error Interrupt Enable */
#define USART_CR3_IREN            (1 << 1)                   /* Bit 1: IrDA mode Enable */
#define USART_CR3_IRLP            (1 << 2)                   /* Bit 2: IrDA Low-Power */
#define USART_CR3_HDSEL           (1 << 3)                   /* Bit 3: Half-Duplex Selection */
#define USART_CR3_NACK            (1 << 4)                   /* Bit 4: Smartcard NACK enable */
#define USART_CR3_SCEN            (1 << 5)                   /* Bit 5: Smartcard mode enable */
#define USART_CR3_DMAR            (1 << 6)                   /* Bit 6: DMA Enable Receiver */
#define USART_CR3_DMAT            (1 << 7)                   /* Bit 7: DMA Enable Transmitter */
#define USART_CR3_RTSE            (1 << 8)                   /* Bit 8: RTS Enable */
#define USART_CR3_CTSE            (1 << 9)                   /* Bit 9: CTS Enable */
#define USART_CR3_CTSIE           (1 << 10)                  /* Bit 10: CTS Interrupt Enable */
#define USART_CR3_ONEBIT          (1 << 11)                  /* Bit 11: One sample bit method Enable */
#define USART_CR3_OVRDIS          (1 << 12)                  /* Bit 12: Overrun Disable */
#define USART_CR3_DDRE            (1 << 13)                  /* Bit 13: DMA disable on Reception error */
#define USART_CR3_DEM             (1 << 14)                  /* Bit 14: Driver Enable mode */
#define USART_CR3_DEP             (1 << 15)                  /* Bit 15: Driver Enable polarity selection */
#define USART_CR3_SCARCNT2_SHIFT  (17)                       /* Bits 17-19: Smart card auto retry count */
#define USART_CR3_SCARCNT2_MASK   (7 << USART_CR3_SCARCNT2_SHIFT)
#define USART_CR3_WUS_SHIFT       (20)                       /* Bits 20-21: Wakeup from Stop mode interrupt flag selection */
#define USART_CR3_WUS_MASK        (3 << USART_CR3_WUS_SHIFT)
#define USART_CR3_WUS_ADDRESS     (0 << USART_CR3_WUS_SHIFT) /* 00: WUF active on address match */
#define USART_CR3_WUS_START       (2 << USART_CR3_WUS_SHIFT) /* 10: WUF active on Start bit detection */
#define USART_CR3_WUS_RXNE        (3 << USART_CR3_WUS_SHIFT) /* 11: WUF active on RXNE */
#define USART_CR3_WUFIE           (1 << 22)                  /* Bit 22: Wakeup from Stop mode interrupt enable */

/* Baud Rate Register */

#define USART_BRR_FRAC_SHIFT      (0)       /* Bits 3-0: fraction of USARTDIV */
#define USART_BRR_FRAC_MASK       (0x0f << USART_BRR_FRAC_SHIFT)
#define USART_BRR_MANT_SHIFT      (4)       /* Bits 15-4: mantissa of USARTDIV */
#define USART_BRR_MANT_MASK       (0x0fff << USART_BRR_MANT_SHIFT)

/* Guard time and prescaler register */

#define USART_GTPR_PSC_SHIFT      (0) /* Bits 0-7: Prescaler value */
#define USART_GTPR_PSC_MASK       (0xff << USART_GTPR_PSC_SHIFT)
#define USART_GTPR_GT_SHIFT       (8) /* Bits 8-15: Guard time value */
#define USART_GTPR_GT_MASK        (0xff <<  USART_GTPR_GT_SHIFT)

/* Receiver timeout register */

/* Request Register */

#define USART_CR1_SBRKQ           (1 << 1)  /* Bit 1: Send Break */

/* Interrupt and Status register */

#define USART_ISR_PE              (1 << 0)  /* Bit 0:  Parity Error */
#define USART_ISR_FE              (1 << 1)  /* Bit 1:  Framing Error */
#define USART_ISR_NF              (1 << 2)  /* Bit 2:  Noise Error Flag */
#define USART_ISR_ORE             (1 << 3)  /* Bit 3:  OverRun Error */
#define USART_ISR_IDLE            (1 << 4)  /* Bit 4:  IDLE line detected */
#define USART_ISR_RXNE            (1 << 5)  /* Bit 5:  Read Data Register Not Empty */
#define USART_ISR_TC              (1 << 6)  /* Bit 6:  Transmission Complete */
#define USART_ISR_TXE             (1 << 7)  /* Bit 7:  Transmit Data Register Empty */
#define USART_ISR_LBDF            (1 << 8)  /* Bit 8:  LIN Break Detection Flag */
#define USART_ISR_CTSIF           (1 << 9)  /* Bit 9:  CTS Interrupt Flag */
#define USART_ISR_CTS             (1 << 10) /* Bit 10: CTS Flag */
#define USART_ISR_RTOF            (1 << 11) /* Bit 11: Receiver timeout Flag */
#define USART_ISR_EOBF            (1 << 12) /* Bit 12: End of block Flag */
#define USART_ISR_UDR             (1 << 13) /* Bit 13: SPI slave underrun error flag */
#define USART_ISR_ABRE            (1 << 14) /* Bit 14: Auto baud rate Error */
#define USART_ISR_ABRF            (1 << 15) /* Bit 15: Auto baud rate Flag */
#define USART_ISR_BUSY            (1 << 16) /* Bit 16: Busy Flag */
#define USART_ISR_CMF             (1 << 17) /* Bit 17: Character match Flag */
#define USART_ISR_SBKF            (1 << 18) /* Bit 18: Send break Flag */
#define USART_ISR_RWU             (1 << 19) /* Bit 19: Receiver wakeup from Mute mode */
#define USART_ISR_WUF             (1 << 20) /* Bit 20: Wakeup from Stop mode Flag */
#define USART_ISR_TEACK           (1 << 21) /* Bit 21: Transmit enable acknowledge Flag */
#define USART_ISR_REACK           (1 << 22) /* Bit 22: Receive enable acknowledge Flag */

/* ICR */

#define USART_ICR_PECF            (1 << 0)  /* Bit 0:  Parity error clear flag */
#define USART_ICR_FECF            (1 << 1)  /* Bit 1:  Framing error clear flag */
#define USART_ICR_NCF             (1 << 2)  /* Bit 2:  Noise detected clear flag */
#define USART_ICR_ORECF           (1 << 3)  /* Bit 3:  Overrun error clear flag */
#define USART_ICR_IDLECF          (1 << 4)  /* Bit 4:  Idle line detected clear flag */
#define USART_ICR_TCCF            (1 << 6)  /* Bit 6:  Transmission complete clear flag */
#define USART_ICR_LBDCF           (1 << 8)  /* Bit 8:  LIN break detection clear flag */
#define USART_ICR_CTSCF           (1 << 9)  /* Bit 9:  CTS clear flag */
#define USART_ICR_RTOCF           (1 << 11) /* Bit 11: Receiver timeout clear flag */
#define USART_ICR_EOBCF           (1 << 12) /* Bit 12: End of block clear flag */
#define USART_ICR_CMCF            (1 << 17) /* Bit 17: Character match clear flag */
#define USART_ICR_WUCF            (1 << 20) /* Bit 20: Wakeup from Stop mode clear flag */

/* Receive Data register */

#define USART_RDR_SHIFT           (0)       /* Bits 8:0: Data value */
#define USART_RDR_MASK            (0xff << USART_RDR_SHIFT)

/* Transmit Data register */

#define USART_TDR_SHIFT           (0)       /* Bits 8:0: Data value */
#define USART_TDR_MASK            (0xff << USART_TDR_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_UART_H */
