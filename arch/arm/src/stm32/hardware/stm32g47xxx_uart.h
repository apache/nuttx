/****************************************************************************
 *  arch/arm/src/stm32/hardware/stm32g47xxx_uart.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G47XXX_UART_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G47XXX_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_USART_CR1_OFFSET         0x0000         /* Control Register 1 */
#define STM32_USART_CR2_OFFSET         0x0004         /* Control Register 2 */
#define STM32_USART_CR3_OFFSET         0x0008         /* Control Register 3 */
#define STM32_USART_BRR_OFFSET         0x000c         /* BAUD Rate Register */
#define STM32_USART_GTPR_OFFSET        0x0010         /* Guard Time and Prescaler Register */
#define STM32_USART_RTOR_OFFSET        0x0014         /* Receiver Timeout Register */
#define STM32_USART_RQR_OFFSET         0x0018         /* Request Register */
#define STM32_USART_ISR_OFFSET         0x001c         /* Interrupt and Status Register */
#define STM32_USART_ICR_OFFSET         0x0020         /* Interrupt Flag Clear Register */
#define STM32_USART_RDR_OFFSET         0x0024         /* Receive Data Register */
#define STM32_USART_TDR_OFFSET         0x0028         /* Transmit Data Register */
#define STM32_USART_PRESC_OFFSET       0x002c         /* Prescaler Register */

/* Register Addresses *******************************************************/

#if (STM32_NUSART > 0)
#  define STM32_USART1_CR1             (STM32_USART1_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART1_CR2             (STM32_USART1_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART1_CR3             (STM32_USART1_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART1_BRR             (STM32_USART1_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART1_GTPR            (STM32_USART1_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART1_RTOR            (STM32_USART1_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART1_RQR             (STM32_USART1_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART1_ISR             (STM32_USART1_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART1_ICR             (STM32_USART1_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART1_RDR             (STM32_USART1_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART1_TDR             (STM32_USART1_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART1_PRESC           (STM32_USART1_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if (STM32_NUSART > 1)
#  define STM32_USART2_CR1             (STM32_USART2_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART2_CR2             (STM32_USART2_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART2_CR3             (STM32_USART2_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART2_BRR             (STM32_USART2_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART2_GTPR            (STM32_USART2_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART2_RTOR            (STM32_USART2_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART2_RQR             (STM32_USART2_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART2_ISR             (STM32_USART2_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART2_ICR             (STM32_USART2_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART2_RDR             (STM32_USART2_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART2_TDR             (STM32_USART2_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART2_PRESC           (STM32_USART2_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if (STM32_NUSART > 2)
#  define STM32_USART3_CR1             (STM32_USART3_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART3_CR2             (STM32_USART3_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART3_CR3             (STM32_USART3_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART3_BRR             (STM32_USART3_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART3_GTPR            (STM32_USART3_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART3_RTOR            (STM32_USART3_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART3_RQR             (STM32_USART3_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART3_ISR             (STM32_USART3_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART3_ICR             (STM32_USART3_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART3_RDR             (STM32_USART3_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART3_TDR             (STM32_USART3_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART3_PRESC           (STM32_USART3_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if (STM32_NUSART > 3)
#  define STM32_UART4_CR1              (STM32_UART4_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_UART4_CR2              (STM32_UART4_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_UART4_CR3              (STM32_UART4_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_UART4_BRR              (STM32_UART4_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_UART4_GTPR             (STM32_UART4_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_UART4_RTOR             (STM32_UART4_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_UART4_RQR              (STM32_UART4_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_UART4_ISR              (STM32_UART4_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_UART4_ICR              (STM32_UART4_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_UART4_RDR              (STM32_UART4_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_UART4_TDR              (STM32_UART4_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_UART4_PRESC            (STM32_UART4_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if (STM32_NUSART > 4)
#  define STM32_UART5_CR1              (STM32_UART5_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_UART5_CR2              (STM32_UART5_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_UART5_CR3              (STM32_UART5_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_UART5_BRR              (STM32_UART5_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_UART5_GTPR             (STM32_UART5_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_UART5_RTOR             (STM32_UART5_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_UART5_RQR              (STM32_UART5_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_UART5_ISR              (STM32_UART5_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_UART5_ICR              (STM32_UART5_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_UART5_RDR              (STM32_UART5_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_UART5_TDR              (STM32_UART5_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_UART5_PRESC            (STM32_UART5_BASE + STM32_USART_PRESC_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Control Register 1 */

#define USART_CR1_UE                   (1 << 0)                           /* Bit 0 - USART Enable */
#define USART_CR1_UESM                 (1 << 1)                           /* Bit 1 - USART Enable in low power Mode */
#define USART_CR1_RE                   (1 << 2)                           /* Bit 2 - Receiver Enable */
#define USART_CR1_TE                   (1 << 3)                           /* Bit 3 - Transmitter Enable */
#define USART_CR1_IDLEIE               (1 << 4)                           /* Bit 4 - IDLE Interrupt Enable */
#define USART_CR1_RXFNEIE              (1 << 5)                           /* Bit 5 in FIFO mode - Rx FIFO Not Empty Interrupt Enable */
#define USART_CR1_RXNEIE               (1 << 5)                           /* Bit 5 in Non-FIFO mode - Rx Data Register Not Empty Interrupt Enable */
#define USART_CR1_TCIE                 (1 << 6)                           /* Bit 6 - Transmission Complete Interrupt Enable */
#define USART_CR1_TXFNFIE              (1 << 7)                           /* Bit 7 in FIFO mode - Tx FIFO Not Full Interrupt Enable */
#define USART_CR1_TXEIE                (1 << 7)                           /* Bit 7 in Non-FIFO mode - Tx Data Register Empty Interrupt Enable */
#define USART_CR1_PEIE                 (1 << 8)                           /* Bit 8 - PE Interrupt Enable */
#define USART_CR1_PS                   (1 << 9)                           /* Bit 9 - Parity Selection */
#define USART_CR1_PCE                  (1 << 10)                          /* Bit 10 - Parity Control Enable */
#define USART_CR1_WAKE                 (1 << 11)                          /* Bit 11 - Receiver Wakeup method */
#define USART_CR1_M0                   (1 << 12)                          /* Bit 12 - Word length - Bit 0 */
#define USART_CR1_MME                  (1 << 13)                          /* Bit 13 - Mute Mode Enable */
#define USART_CR1_CMIE                 (1 << 14)                          /* Bit 14 - Character match interrupt enable */
#define USART_CR1_OVER8                (1 << 15)                          /* Bit 15 - Oversampling by 8-bit or 16-bit mode */
#define USART_CR1_DEDT_SHIFT           (16)                               /* Bits 20:16 - Driver Enable Deassertion Time, in 1/16ths or 1/8ths bit time */
#define USART_CR1_DEDT_MASK            (0x1f << USART_CR1_DEDT_SHIFT)
#  define USART_CR1_DEDT(n)            (((n) << USART_CR1_DEDT_SHIFT) & USART_CR1_DEDT_MASK)
#define USART_CR1_DEAT_SHIFT           (21)                               /* Bits 25:21 - Driver Enable Assertion Time, in 1/16ths or 1/8ths bit time */
#define USART_CR1_DEAT_MASK            (0x1f << USART_CR1_DEAT_SHIFT)
#  define USART_CR1_DEAT(n)            (((n) << USART_CR1_DEAT_SHIFT) & USART_CR1_DEAT_MASK)
#define USART_CR1_RTOIE                (1 << 26)                          /* Bit 26 - Receive Time Out interrupt enable */
#define USART_CR1_EOBIE                (1 << 27)                          /* Bit 27 - End of Block interrupt enable */
#define USART_CR1_M1                   (1 << 28)                          /* Bit 28 - Word length - Bit 1 */
#define USART_CR1_FIFOEN               (1 << 29)                          /* Bit 29 - FIFO mode enable */
#define USART_CR1_TXFEIE               (1 << 30)                          /* Bit 30 - TXFIFO empty interrupt enable */
#define USART_CR1_RXFFIE               (1 << 31)                          /* Bit 31 - RXFIFO Full interrupt enable */

#define USART_CR1_M_MASK               (USART_CR1_M0 | USART_CR1_M1)

#define USART_CR1_ALLINTS \
  (USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_TCIE | \
   USART_CR1_TXEIE | USART_CR1_PEIE | USART_CR1_CMIE | USART_CR1_RTOIE | \
   USART_CR1_EOBIE | USART_CR1_TXFEIE | USART_CR1_RXFFIE)

/* Control Register 2 */

#define USART_CR2_SLVEN                (1 << 0)                           /* Synchronous Slave Mode Enable */
#define USART_CR2_DIS_NSS              (1 << 3)                           /* Slave Select (NSS) Pin Ignore For SPI */
#define USART_CR2_ADDM7                (1 << 4)                           /* 7-Bit / 4-Bit Address Detection */
#define USART_CR2_LBDL                 (1 << 5)                           /* LIN Break Detection Length */
#define USART_CR2_LBDIE                (1 << 6)                           /* LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL                 (1 << 8)                           /* Last Bit Clock pulse */
#define USART_CR2_CPHA                 (1 << 9)                           /* Clock Phase */
#define USART_CR2_CPOL                 (1 << 10)                          /* Clock Polarity */
#define USART_CR2_CLKEN                (1 << 11)                          /* Clock Enable */
#define USART_CR2_STOP_SHIFT           (12)                               /* Stop Bit Mode */
#define USART_CR2_STOP_MASK            (0x3 << USART_CR2_STOP_SHIFT)
#  define USART_CR2_STOP1              (0x0 << USART_CR2_STOP_SHIFT)      /* 1 Stop Bit */
#  define USART_CR2_STOP0p5            (0x1 << USART_CR2_STOP_SHIFT)      /* 0.5 Stop Bit */
#  define USART_CR2_STOP2              (0x2 << USART_CR2_STOP_SHIFT)      /* 2 Stop Bits */
#  define USART_CR2_STOP1p5            (0x3 << USART_CR2_STOP_SHIFT)      /* 1.5 Stop Bits */
#define USART_CR2_LINEN                (1 << 14)                          /* LIN Mode Enable */
#define USART_CR2_SWAP                 (1 << 15)                          /* Swap TX/RX Pins */
#define USART_CR2_RXINV                (1 << 16)                          /* RX Pin Active Level Inversion */
#define USART_CR2_TXINV                (1 << 17)                          /* TX Pin Active Level Inversion */
#define USART_CR2_DATAINV              (1 << 18)                          /* Binary Data Inversion */
#define USART_CR2_MSBFIRST             (1 << 19)                          /* MSB First */
#define USART_CR2_ABREN                (1 << 20)                          /* Auto BAUD-Rate Enable */
#define USART_CR2_ABRMOD_SHIFT         (21)                               /* Auto BAUD-Rate Detection Mode */
#define USART_CR2_ABRMOD_MASK          (0x3 << USART_CR2_ABRMOD_SHIFT)
#  define USART_CR2_ABRMOD_STARTBIT    (0x0 << USART_CR2_ABRMOD_SHIFT)    /* Measurement of Start Bit */
#  define USART_CR2_ABRMOD_FALLEDGE    (0x1 << USART_CR2_ABRMOD_SHIFT)    /* Falling Edge To Falling Edge */
#  define USART_CR2_ABRMOD_7F_FRAME    (0x2 << USART_CR2_ABRMOD_SHIFT)    /* 0X7F Frame Detection */
#  define USART_CR2_ABRMOD_55_FRAME    (0x3 << USART_CR2_ABRMOD_SHIFT)    /* 0X55 Frame Detection */
#define USART_CR2_RTOEN                (1 << 23)                          /* Receiver Time-Out Enable */
#define USART_CR2_ADD8_SHIFT           (24)                               /* Address of the USART Node */
#define USART_CR2_ADD8_MASK            (0xff << USART_CR2_ADD8_SHIFT)

/* Control Register 3 */

#define USART_CR3_EIE                  (1 << 0)                           /* Error Interrupt Enable */
#define USART_CR3_IREN                 (1 << 1)                           /* IrDA Mode Enable */
#define USART_CR3_IRLP                 (1 << 2)                           /* IrDA Low-Power */
#define USART_CR3_HDSEL                (1 << 3)                           /* Half-Duplex Selection */
#define USART_CR3_NACK                 (1 << 4)                           /* SmartCard NACK Enable */
#define USART_CR3_SCEN                 (1 << 5)                           /* SmartCard Mode Enable */
#define USART_CR3_DMAR                 (1 << 6)                           /* DMA Enable Receiver */
#define USART_CR3_DMAT                 (1 << 7)                           /* DMA Enable Transmitter */
#define USART_CR3_RTSE                 (1 << 8)                           /* RTS Enable */
#define USART_CR3_CTSE                 (1 << 9)                           /* CTS Enable */
#define USART_CR3_CTSIE                (1 << 10)                          /* CTS Interrupt Enable */
#define USART_CR3_ONEBIT               (1 << 11)                          /* One Sample Bit Method Enable */
#define USART_CR3_OVRDIS               (1 << 12)                          /* Overrun Disable */
#define USART_CR3_DDRE                 (1 << 13)                          /* DMA Disable on Reception Error */
#define USART_CR3_DEM                  (1 << 14)                          /* Driver Enable Mode */
#define USART_CR3_DEP                  (1 << 15)
#define USART_CR3_SCARCNT_SHIFT        (17)                               /* SmartCard Auto-Retry Count */
#define USART_CR3_SCARCNT_MASK         (0x7 << USART_CR3_SCARCNT_SHIFT)
#  define USART_CR3_SCARCNT(n)         (((n) << USART_CR3_SCARCNT_SHIFT) & USART_CR3_SCARCNT_MASK)
#define USART_CR3_WUS_SHIFT            (20)                               /* Wake Up From Low Power Mode Interrupt Flag Selection) */
#define USART_CR3_WUS_MASK             (0x3 << USART_CR3_WUS_SHIFT)
#  define USART_CR3_WUS_ADDR           (0x0 << USART_CR3_WUS_SHIFT)       /* On Address Match */
#  define USART_CR3_WUS_STARTBIT       (0x2 << USART_CR3_WUS_SHIFT)       /* On Start Bit Detection */
#  define USART_CR3_WUS_RXFNE          (0x3 << USART_CR3_WUS_SHIFT)       /* On RXNE/RXFNE */
#define USART_CR3_WUFIE                (1 << 22)                          /* Wake Up From Low Power Mode Interrupt Enable */
#define USART_CR3_TXFTIE               (1 << 23)                          /* Transmit FIFO Threshold Interrupt Enable */
#define USART_CR3_TCBGTIE              (1 << 24)                          /* Transmit Complete Before Guard Time Interrupt Enable */
#define USART_CR3_RXFTCFG_SHIFT        (25)                               /* Receive FIFO Threshold Configuration */
#define USART_CR3_RXFTCFG_MASK         (0x7 << USART_CR3_RXFTCFG_SHIFT)
#  define USART_CR3_RXFTCFG_1_8        (0x0 << USART_CR3_RXFTCFG_SHIFT)   /* When Rx FIFO Reaches 1/8Th Depth */
#  define USART_CR3_RXFTCFG_1_4        (0x1 << USART_CR3_RXFTCFG_SHIFT)   /* When Rx FIFO Reaches 1/4Th Depth */
#  define USART_CR3_RXFTCFG_1_2        (0x2 << USART_CR3_RXFTCFG_SHIFT)   /* When Rx FIFO Reaches 1/2 Depth */
#  define USART_CR3_RXFTCFG_3_4        (0x3 << USART_CR3_RXFTCFG_SHIFT)   /* When Rx FIFO Reaches 3/4Ths Depth */
#  define USART_CR3_RXFTCFG_7_8        (0x4 << USART_CR3_RXFTCFG_SHIFT)   /* When Rx FIFO Reaches 7/8Ths Depth */
#  define USART_CR3_RXFTCFG_FULL       (0x5 << USART_CR3_RXFTCFG_SHIFT)   /* When Rx FIFO Is Full */
#define USART_CR3_RXFTIE               (1 << 28)                          /* Receive FIFO Threshold Interrupt Enable */
#define USART_CR3_TXFTCFG_SHIFT        (29)                               /* Transmit FIFO Threshold Configuration */
#define USART_CR3_TXFTCFG_MASK         (0x7 << USART_CR3_TXFTCFG_SHIFT)
#  define USART_CR3_TXFTCFG_1_8        (0x0 << USART_CR3_TXFTCFG_SHIFT)   /* When Tx FIFO Reaches 1/8Th Depth */
#  define USART_CR3_TXFTCFG_1_4        (0x1 << USART_CR3_TXFTCFG_SHIFT)   /* When Tx FIFO Reaches 1/4Th Depth */
#  define USART_CR3_TXFTCFG_1_2        (0x2 << USART_CR3_TXFTCFG_SHIFT)   /* When Tx FIFO Reaches 1/2 Depth */
#  define USART_CR3_TXFTCFG_3_4        (0x3 << USART_CR3_TXFTCFG_SHIFT)   /* When Tx FIFO Reaches 3/4Ths Depth */
#  define USART_CR3_TXFTCFG_7_8        (0x4 << USART_CR3_TXFTCFG_SHIFT)   /* When Tx FIFO Reaches 7/8Ths Depth */
#  define USART_CR3_TXFTCFG_FULL       (0x5 << USART_CR3_TXFTCFG_SHIFT)   /* When Tx FIFO Is Full */

/* BAUD Rate Register */

/* Full BRR field */

#define USART_BRR_SHIFT                (0)
#define USART_BRR_MASK                 (0xffff << USART_BRR_BRR_SHIFT)
#  define USART_BRR(n)                 (((n) << USART_BRR_BRR_SHIFT) & USART_BRR_BRR_MASK)

/* Partial BRR field BRR[3:0]:
 *
 * When OVER8 = 0: BRR[3:0] = USARTDIV[3:0]
 *
 * When OVER8 = 1: BRR[2:0] = (USARTDIV[3:0] >> 1) and
 *                 BRR[3] must be kept cleared.
 */

#define USART_BRR_0_3_SHIFT            (0)
#define USART_BRR_0_3_MASK             (0xf << USART_BRR_0_3_SHIFT)
#  define USART_BRR_0_3(n)             (((n) << USART_BRR_0_3_SHIFT) & USART_BRR_0_3_MASK)

/* Partial BRR field BRR[15:4]:
 * BRR[15:4] = USARTDIV[15:4]
 */

#define USART_BRR_4_15_SHIFT           (4)
#define USART_BRR_4_15_MASK            (0xfff << USART_BRR_4_15_SHIFT)
#  define USART_BRR_4_15(n)            (((n) << USART_BRR_4_15_SHIFT) & USART_BRR_4_15_MASK)

/* Guard Time and Prescaler Register */

#define USART_GTPR_PSC_SHIFT           (0)            /* Prescaler Value */
#define USART_GTPR_PSC_MASK            (0xff << USART_GTPR_PSC_SHIFT)
#  define USART_GTPR_PSC(n)            (((n) << USART_GTPR_PSC_SHIFT) & USART_GTPR_PSC_MASK)
#define USART_GTPR_GT_SHIFT            (8)            /* Guard Time Value */
#define USART_GTPR_GT_MASK             (0xff << USART_GTPR_GT_SHIFT)
#  define USART_GTPR_GT(n)             (((n) << USART_GTPR_GT_SHIFT) & USART_GTPR_GT_MASK)

/* Receiver Timeout Register */

#define USART_RTOR_RTO_SHIFT           (0)           /* Receiver Time Out Value */
#define USART_RTOR_RTO_MASK            (0xffffff << USART_RTOR_RTO_SHIFT)
#  define USART_RTOR_RTO(n)            (((n) << USART_RTOR_RTO_SHIFT) & USART_RTOR_RTO_MASK)
#define USART_RTOR_BLEN_SHIFT          (24)          /* Block Length */
#define USART_RTOR_BLEN_MASK           (0xff << USART_RTOR_BLEN_SHIFT)
#  define USART_RTOR_BLEN(n)           (((n) << USART_RTOR_BLEN_SHIFT) & USART_RTOR_BLEN_MASK)

/* Request Register */

#define USART_RQR_ABRRQ                (1 << 0)       /* Bit 0 - Auto-Baud Rate Request */
#define USART_RQR_SBKRQ                (1 << 1)       /* Bit 1 - Send Break Request */
#define USART_RQR_MMRQ                 (1 << 2)       /* Bit 2 - Mute Mode Request */
#define USART_RQR_RXFRQ                (1 << 3)       /* Bit 3 - Receive Data Flush Request */
#define USART_RQR_TXFRQ                (1 << 4)       /* Bit 4 - Transmit Data Flush Request */

/* Interrupt and Status Register */

#define USART_ISR_PE                   (1 << 0)       /* Bit 0 - Parity Error */
#define USART_ISR_FE                   (1 << 1)       /* Bit 1 - Framing Error */
#define USART_ISR_NE                   (1 << 2)       /* Bit 2 - Noise Detected Flag */
#define USART_ISR_ORE                  (1 << 3)       /* Bit 3 - Overrun Error */
#define USART_ISR_IDLE                 (1 << 4)       /* Bit 4 - Idle Line Detected */
#define USART_ISR_RXFNE                (1 << 5)       /* Bit 5 (When FIFO in use) - Rx FIFO Not Empty */
#define USART_ISR_RXNE                 (1 << 5)       /* Bit 5 (When FIFO not in use) - Rx Data Register Not Empty */
#define USART_ISR_TC                   (1 << 6)       /* Bit 6 - Transmission Complete */
#define USART_ISR_TXFNF                (1 << 7)       /* Bit 7 (When FIFO in use) - Tx FIFO Not Full */
#define USART_ISR_TXE                  (1 << 7)       /* Bit 7 (When FIFO not in use) - Tx Data Register Empty */
#define USART_ISR_LBDF                 (1 << 8)       /* Bit 8 - LIN Break Detection Flag */
#define USART_ISR_CTSIF                (1 << 9)       /* Bit 9 - CTS Interrupt Flag */
#define USART_ISR_CTS                  (1 << 10)      /* Bit 10 - CTS Flag */
#define USART_ISR_RTOF                 (1 << 11)      /* Bit 11 - Receiver Time Out */
#define USART_ISR_EOBF                 (1 << 12)      /* Bit 12 - End of Block Flag */
#define USART_ISR_UDR                  (1 << 13)      /* Bit 13 - SPI Slave Underrun Error Flag */
#define USART_ISR_ABRE                 (1 << 14)      /* Bit 14 - Auto BAUD Rate Error */
#define USART_ISR_ABRF                 (1 << 15)      /* Bit 15 - Auto BAUD Rate Flag */
#define USART_ISR_BUSY                 (1 << 16)      /* Bit 16 - Busy Flag */
#define USART_ISR_CMF                  (1 << 17)      /* Bit 17 - Character Match Flag */
#define USART_ISR_SBKF                 (1 << 18)      /* Bit 18 - Send Break Flag */
#define USART_ISR_RWU                  (1 << 19)      /* Bit 19 - Receive Wake Up From Mute Mode Flag */
#define USART_ISR_WUF                  (1 << 20)      /* Bit 20 - Wake Up From Stop Mode Flag */
#define USART_ISR_TEACK                (1 << 21)      /* Bit 21 - Transmit Enable Acknowledge Flag */
#define USART_ISR_REACK                (1 << 22)      /* Bit 22 - Receive Enable Acknowledge Flag */
#define USART_ISR_TXFE                 (1 << 23)      /* Bit 23 (When FIFO in use) - Tx FIFO Empty */
#define USART_ISR_RXFF                 (1 << 24)      /* Bit 24 (When FIFO in use) - Rx FIFO Full */
#define USART_ISR_TCBGT                (1 << 25)      /* Bit 25 - Transmission Complete Before Guard Time Completion */
#define USART_ISR_RXFT                 (1 << 26)      /* Bit 26 (When FIFO in use) - Rx FIFO Threshold Flag */
#define USART_ISR_TXFT                 (1 << 27)      /* Bit 27 (When FIFO in use) - Tx FIFO Threshold Flag */

#define USART_ISR_ALLBITS              (0x0fffffff)

/* Interrupt Flag Clear Register */

#define USART_ICR_PECF                 (1 << 0)       /* Bit 0 - Parity Error Clear Flag */
#define USART_ICR_FECF                 (1 << 1)       /* Bit 1 - Framing Error Clear Flag */
#define USART_ICR_NCF                  (1 << 2)       /* Bit 2 - Noise detected Clear Flag */
#define USART_ICR_ORECF                (1 << 3)       /* Bit 3 - OverRun Error Clear Flag */
#define USART_ICR_IDLECF               (1 << 4)       /* Bit 4 - Idle Line Detected Clear Flag */
#define USART_ICR_TXFECF               (1 << 5)       /* Bit 5 - Tx FIFO Empty Clear Flag */
#define USART_ICR_TCCF                 (1 << 6)       /* Bit 6 - Transmission Complete Clear Flag */
#define USART_ICR_TCBGTCF              (1 << 7)       /* Bit 7 - Transmission Complete Before Guard Time Clear Flag */
#define USART_ICR_LBDCF                (1 << 8)       /* Bit 8 - LIN Break Detection Clear Flag */
#define USART_ICR_CTSCF                (1 << 9)       /* Bit 9 - CTS Interrupt Clear Flag */
#define USART_ICR_RTOCF                (1 << 11)      /* Bit 11 - Receiver Timeout Clear Flag */
#define USART_ICR_EOBCF                (1 << 12)      /* Bit 12 - End of Block Clear Flag */
#define USART_ICR_UDRCF                (1 << 13)      /* Bit 13 - SPI Slave Underrun Clear Flag */
#define USART_ICR_CMCF                 (1 << 17)      /* Bit 17 - Character Match Clear Flag */
#define USART_ICR_WUCF                 (1 << 20)      /* Bit 20 - Wake Up From Stop Mode Clear Flag */

/* Receive Data Register */

#define USART_RDR_SHIFT                (0)
#define USART_RDR_MASK                 (0x1ff << USART_RDR_SHIFT)
#  define USART_RDR(n)                 (((n) << USART_RDR_SHIFT) & USART_RDR_MASK)

/* Transmit Data Register */

#define USART_TDR_SHIFT                (0)
#define USART_TDR_MASK                 (0x1ff << USART_TDR_SHIFT)
#  define USART_TDR(n)                 (((n) << USART_TDR_SHIFT) & USART_TDR_MASK)

/* Prescaler Register */

#define USART_PRESC_PRESCALER_SHIFT    (0)
#define USART_PRESC_PRESCALER_MASK     (0xf << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_1        (0x0 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_2        (0x1 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_4        (0x2 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_6        (0x3 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_8        (0x4 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_10       (0x5 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_12       (0x6 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_16       (0x7 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_32       (0x8 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_64       (0x9 << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_128      (0xa << USART_PRESC_PRESCALER_SHIFT)
#define USART_PRESC_PRESCALER_256      (0xb << USART_PRESC_PRESCALER_SHIFT)

/* Compatibility definitions ************************************************/

/* Compatibility with F1/F2/F4 Status Register names */

#define STM32_USART_SR_OFFSET          STM32_USART_ISR_OFFSET

#define USART_SR_PE                    USART_ISR_PE   /* Parity Error */
#define USART_SR_FE                    USART_ISR_FE   /* Framing error */
#define USART_SR_NE                    USART_ISR_NE   /* Noise detected flag */
#define USART_SR_ORE                   USART_ISR_ORE  /* Overrun error */
#define USART_SR_IDLE                  USART_ISR_IDLE /* IDLE line detected */
#define USART_SR_RXNE                  USART_ISR_RXNE /* Read Data Register Not Empty */
#define USART_SR_TC                    USART_ISR_TC   /* Transmission Complete */
#define USART_SR_TXE                   USART_ISR_TXE  /* Transmit Data Register Empty */
#define USART_SR_LBD                   USART_ISR_LBDF /* LIN Break Detection Flag */
#define USART_SR_CTS                   USART_ISR_CTS  /* CTS Flag */

#define USART_SR_ALLBITS               USART_ISR_ALLBITS

#define USART_CR1_M                    USART_CR1_M0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G47XXX_UART_H */
