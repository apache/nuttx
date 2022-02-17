/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_uart_v2.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_UART_V2_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_UART_V2_H

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
#define STM32_USART_BRR_OFFSET    0x000c  /* Baud Rate Register (32-bits) */
#define STM32_USART_GTPR_OFFSET   0x0010  /* Guard time and prescaler register  */
#define STM32_USART_RTOR_OFFSET   0x0014  /* Receiver timeout register */
#define STM32_USART_RQR_OFFSET    0x0018  /* Request register */
#define STM32_USART_ISR_OFFSET    0x001c  /* Interrupt & status register */
#define STM32_USART_ICR_OFFSET    0x0020  /* Interrupt flag clear register */
#define STM32_USART_RDR_OFFSET    0x0024  /* Receive data register */
#define STM32_USART_TDR_OFFSET    0x0028  /* Transmit data register */
#define STM32_USART_PRESC_OFFSET  0x002c  /* Prescaler register */

/* Register Addresses *******************************************************/

#if STM32_NUSART > 0
#  define STM32_USART1_CR1        (STM32_USART1_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART1_CR2        (STM32_USART1_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART1_CR3        (STM32_USART1_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART1_BRR        (STM32_USART1_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART1_GTPR       (STM32_USART1_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART1_RTOR       (STM32_USART1_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART1_RQR        (STM32_USART1_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART1_GTPR       (STM32_USART1_BASE + STM32_USART_GTPR_OFFSET)
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
#  define STM32_USART2_GTPR       (STM32_USART2_BASE + STM32_USART_GTPR_OFFSET)
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
#  define STM32_USART3_GTPR       (STM32_USART3_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART3_ISR        (STM32_USART3_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART3_ICR        (STM32_USART3_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART3_RDR        (STM32_USART3_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART3_TDR        (STM32_USART3_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART3_PRESC      (STM32_USART3_BASE + STM32_USART_PRESC_OFFSET)
#endif

#if STM32_NUSART > 3
#  define STM32_USART4_CR1        (STM32_USART4_BASE + STM32_USART_CR1_OFFSET)
#  define STM32_USART4_CR2        (STM32_USART4_BASE + STM32_USART_CR2_OFFSET)
#  define STM32_USART4_CR3        (STM32_USART4_BASE + STM32_USART_CR3_OFFSET)
#  define STM32_USART4_BRR        (STM32_USART4_BASE + STM32_USART_BRR_OFFSET)
#  define STM32_USART4_GTPR       (STM32_USART4_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART4_RTOR       (STM32_USART4_BASE + STM32_USART_RTOR_OFFSET)
#  define STM32_USART4_RQR        (STM32_USART4_BASE + STM32_USART_RQR_OFFSET)
#  define STM32_USART4_GTPR       (STM32_USART4_BASE + STM32_USART_GTPR_OFFSET)
#  define STM32_USART4_ISR        (STM32_USART4_BASE + STM32_USART_ISR_OFFSET)
#  define STM32_USART4_ICR        (STM32_USART4_BASE + STM32_USART_ICR_OFFSET)
#  define STM32_USART4_RDR        (STM32_USART4_BASE + STM32_USART_RDR_OFFSET)
#  define STM32_USART4_TDR        (STM32_USART4_BASE + STM32_USART_TDR_OFFSET)
#  define STM32_USART4_PRESC      (STM32_USART4_BASE + STM32_USART_PRESC_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Control register 1 */

#define USART_CR1_UE              (1 << 0)  /* Bit 0:  USART enable */
#define USART_CR1_UESM            (1 << 1)  /* Bit 1:  USART enable in low-power mode */
#define USART_CR1_RE              (1 << 2)  /* Bit 2:  Receiver Enable */
#define USART_CR1_TE              (1 << 3)  /* Bit 3:  Transmitter Enable */
#define USART_CR1_IDLEIE          (1 << 4)  /* Bit 4:  IDLE Interrupt Enable */
#define USART_CR1_RXNEIE          (1 << 5)  /* Bit 5:  RXNE Interrupt Enable */
#define USART_CR1_TCIE            (1 << 6)  /* Bit 6:  Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE           (1 << 7)  /* Bit 7:  TXE Interrupt Enable */
#define USART_CR1_PEIE            (1 << 8)  /* Bit 8:  PE Interrupt Enable */
#define USART_CR1_PS              (1 << 9)  /* Bit 9:  Parity Selection */
#define USART_CR1_PCE             (1 << 10) /* Bit 10: Parity Control Enable */
#define USART_CR1_WAKE            (1 << 11) /* Bit 11: Receiver wakeup method */
#define USART_CR1_M0              (1 << 12) /* Bit 12: Word length, bit 0 */
#define USART_CR1_MME             (1 << 13) /* Bit 13: Mute mode enable */
#define USART_CR1_CMIE            (1 << 14) /* Bit 14: Character match interrupt enable */
#define USART_CR1_OVER8           (1 << 15) /* Bit 15: Oversampling mode */
#define USART_CR1_DEDT_SHIFT      (16)      /* Bits 16-20: Driver Enable deassertion time */
#define USART_CR1_DEDT_MASK       (31 << USART_CR1_DEDT_SHIFT)
#  define USART_CR1_DEDT(n)       ((uint32_t)(n) << USART_CR1_DEDT_SHIFT)
#define USART_CR1_DEAT_SHIFT      (21)      /* Bits 21-25: Driver Enable assertion time */
#define USART_CR1_DEAT_MASK       (31 << USART_CR1_DEAT_SHIFT)
#  define USART_CR1_DEAT(n)       ((uint32_t)(n) << USART_CR1_DEAT_SHIFT)
#define USART_CR1_RTOIE           (1 << 26) /* Bit 26: Receiver timeout interrupt enable */
#define USART_CR1_EOBIE           (1 << 27) /* Bit 27: End of Block interrupt enable */
#define USART_CR1_M1              (1 << 28) /* Bit 28: Word length, bit 1 */
#define USART_CR1_FIFOEN          (1 << 29) /* Bit 29: FIFO mode enable */
#define USART_CR1_TXFEIE          (1 << 30) /* Bit 30: TXFIFO empty interrupt enable */
#define USART_CR1_RXFFIE          (1 << 31) /* Bit 31: RXFIFO Full interrupt enable */

#define USART_CR1_ALLINTS \
  (USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_TCIE | USART_CR1_TXEIE |\
   USART_CR1_PEIE | USART_CR1_CMIE |USART_CR1_RTOIE | USART_CR1_EOBIE |\
   USART_CR1_TXFEIE | USART_CR1_RXFFIE)

/* Control register 2 */

#define USART_CR2_SLVEN           (1 << 0)  /* Bit 0:  Synchronous Slave mode enable */
#define USART_CR2_DISNSS          (1 << 3)  /* Bit 3:  Ignore NSS pin input */
#define USART_CR2_ADDM7           (1 << 4)  /* Bit 4:  7-/4-bit Address Detection */
#define USART_CR2_LBDL            (1 << 5)  /* Bit 5:  LIN Break Detection Length */
#define USART_CR2_LBDIE           (1 << 6)  /* Bit 6:  LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL            (1 << 8)  /* Bit 8:  Last Bit Clock pulse */
#define USART_CR2_CPHA            (1 << 9)  /* Bit 9:  Clock Phase */
#define USART_CR2_CPOL            (1 << 10) /* Bit 10: Clock Polarity */
#define USART_CR2_CLKEN           (1 << 11) /* Bit 11: Clock Enable */
#define USART_CR2_STOP_SHIFT      (12)      /* Bits 13-12: STOP bits */
#define USART_CR2_STOP_MASK       (3 << USART_CR2_STOP_SHIFT)
#  define USART_CR2_STOP1         (0 << USART_CR2_STOP_SHIFT) /* 00: 1 Stop bit */
#  define USART_CR2_STOP2         (2 << USART_CR2_STOP_SHIFT) /* 10: 2 Stop bits */
#  define USART_CR2_STOP1p5       (3 << USART_CR2_STOP_SHIFT) /* 11: 1.5 Stop bit */

#define USART_CR2_LINEN           (1 << 14) /* Bit 14: LIN mode enable */
#define USART_CR2_SWAP            (1 << 15) /* Bit 15: Swap TX/RX pins */
#define USART_CR2_RXINV           (1 << 16) /* Bit 16: RX pin active level inversion */
#define USART_CR2_TXINV           (1 << 17) /* Bit 17: TX pin active level inversion */
#define USART_CR2_DATAINV         (1 << 18) /* Bit 18: Binary data inversion */
#define USART_CR2_MSBFIRST        (1 << 19) /* Bit 19: Most significant bit first */
#define USART_CR2_ABREN           (1 << 20) /* Bit 20: Auto baud rate enable */
#define USART_CR2_ABRMOD_SHIFT    (21)      /* Bits 21-22: Auto baud rate mode */
#define USART_CR2_ABRMOD_MASK     (3 << USART_CR2_ABRMOD_SHIFT)
#  define USART_CR2_ABRMOD_START  (0 << USART_CR2_ABRMOD_SHIFT) /* Start bit */
#  define USART_CR2_ABRMOD_FALL   (1 << USART_CR2_ABRMOD_SHIFT) /* Falling edge measurement */
#  define USART_CR2_ABRMOD_7F     (2 << USART_CR2_ABRMOD_SHIFT) /* 0x7F frame detection */
#  define USART_CR2_ABRMOD_55     (3 << USART_CR2_ABRMOD_SHIFT) /* 0x55 frame detection */

#define USART_CR2_RTOEN           (1 << 23) /* Bit 23: Receiver timeout enable */
#define USART_CR2_ADD4L_SHIFT     (24)      /* Bits 24-27: Address[3:0]:of the USART node */
#define USART_CR2_ADD4L_MASK      (15 << USART_CR2_ADD4L_SHIFT)
#  define USART_CR2_ADD4L(n)      ((uint32_t)(n) << USART_CR2_ADD4L_SHIFT)
#define USART_CR2_ADD4H_SHIFT     (28)      /* Bits 28-31: Address[4:0] of the USART node */
#define USART_CR2_ADD4H_MASK      (15 << USART_CR2_ADD4H_SHIFT)
#  define USART_CR2_ADD4H(n)      ((uint32_t)(n) << USART_CR2_ADD4H_SHIFT)
#define USART_CR2_ADD8_SHIFT      (24)      /* Bits 24-31: Address[7:0] of the USART node */
#define USART_CR2_ADD8_MASK       (255 << USART_CR2_ADD8_SHIFT)
#  define USART_CR2_ADD8(n)       ((uint32_t)(n) << USART_CR2_ADD8_SHIFT)

/* Control register 3 */

#define USART_CR3_EIE             (1 << 0)  /* Bit 0:  Error Interrupt Enable */
#define USART_CR3_IREN            (1 << 1)  /* Bit 1:  IrDA mode Enable */
#define USART_CR3_IRLP            (1 << 2)  /* Bit 2:  IrDA Low-Power */
#define USART_CR3_HDSEL           (1 << 3)  /* Bit 3:  Half-Duplex Selection */
#define USART_CR3_NACK            (1 << 4)  /* Bit 4:  Smartcard NACK enable */
#define USART_CR3_SCEN            (1 << 5)  /* Bit 5:  Smartcard mode enable */
#define USART_CR3_DMAR            (1 << 6)  /* Bit 6:  DMA Enable Receiver */
#define USART_CR3_DMAT            (1 << 7)  /* Bit 7:  DMA Enable Transmitter */
#define USART_CR3_RTSE            (1 << 8)  /* Bit 8:  RTS Enable */
#define USART_CR3_CTSE            (1 << 9)  /* Bit 9:  CTS Enable */
#define USART_CR3_CTSIE           (1 << 10) /* Bit 10: CTS Interrupt Enable */
#define USART_CR3_ONEBIT          (1 << 11) /* Bit 11: One sample bit method enable */
#define USART_CR3_OVRDIS          (1 << 12) /* Bit 12: Overrun Disable */
#define USART_CR3_DDRE            (1 << 13) /* Bit 13: DMA Disable on Reception Error */
#define USART_CR3_DEM             (1 << 14) /* Bit 14: Driver enable mode */
#define USART_CR3_DEP             (1 << 15) /* Bit 15: Driver enable polarity selection */
#define USART_CR3_SCARCNT_SHIFT   (17)      /* Bit 17-19: Smartcard auto-retry count */
#define USART_CR3_SCARCNT_MASK    (7 << USART_CR3_SCARCNT_SHIFT)
#  define USART_CR3_SCARCNT(n)    ((uint32_t)(n) << USART_CR3_SCARCNT_SHIFT)
#define USART_CR3_RXFTCFG_SHIFT   (25)      /* Bit 25-27: Receive FIFO threshold configuration */
#define USART_CR3_RXFTCFG_MASK    (7 << USART_CR3_RXFTCFG_SHIFT)
#  define USART_CR3_RXFTCFG(n)    ((uint32_t)(n) << USART_CR3_RXFTCFG_SHIFT)
#  define USART_CR3_RXFTCFG_12PCT (0 << USART_CR3_RXFTCFG_SHIFT) /* RXFIFO 1/8 full */
#  define USART_CR3_RXFTCFG_25PCT (1 << USART_CR3_RXFTCFG_SHIFT) /* RXFIFO 1/4 full */
#  define USART_CR3_RXFTCFG_50PCT (2 << USART_CR3_RXFTCFG_SHIFT) /* RXFIFO 1/2 full */
#  define USART_CR3_RXFTCFG_75PCT (3 << USART_CR3_RXFTCFG_SHIFT) /* RXFIFO 3/4 full */
#  define USART_CR3_RXFTCFG_88PCT (4 << USART_CR3_RXFTCFG_SHIFT) /* RXFIFO 7/8 full */
#  define USART_CR3_RXFTCFG_FULL  (5 << USART_CR3_RXFTCFG_SHIFT) /* RXIFO full */

#define USART_CR3_RXFTIE          (1 << 28) /* Bit 28: RXFIFO threshold interrupt enable */
#define USART_CR3_TXFTCFG_SHIFT   (29)      /* Bits 29-31: TXFIFO threshold configuration */
#define USART_CR3_TXFTCFG_MASK    (7 << USART_CR3_TXFTCFG_SHIFT)
#  define USART_CR3_TXFTCFG(n)    ((uint32_t)(n) << USART_CR3_TXFTCFG_SHIFT)
#  define USART_CR3_TXFTCFG_12PCT (0 << USART_CR3_TXFTCFG_SHIFT) /* TXFIFO 1/8 full */
#  define USART_CR3_TXFTCFG_24PCT (1 << USART_CR3_TXFTCFG_SHIFT) /* TXFIFO 1/4 full */
#  define USART_CR3_TXFTCFG_50PCT (2 << USART_CR3_TXFTCFG_SHIFT) /* TXFIFO 1/2 full */
#  define USART_CR3_TXFTCFG_75PCT (3 << USART_CR3_TXFTCFG_SHIFT) /* TXFIFO 3/4 full */
#  define USART_CR3_TXFTCFG_88PCT (4 << USART_CR3_TXFTCFG_SHIFT) /* TXFIFO 7/8 full */
#  define USART_CR3_TXFTCFG_EMPY  (5 << USART_CR3_TXFTCFG_SHIFT) /* TXFIFO empty */

/* Baud Rate Register */

#define USART_BRR_SHIFT           (0)       /* Bits 0-15: USARTDIV[15:0] OVER8=0*/
#define USART_BRR_MASK            (0xffff << USART_BRR_SHIFT)
#  define USART_BRR(n)            ((uint32_t)(n) << USART_BRR_SHIFT)

/* Guard time and prescaler register */

#define USART_GTPR_PSC_SHIFT      (0)       /* Bits 0-7: Prescaler value */
#define USART_GTPR_PSC_MASK       (0xff << USART_GTPR_PSC_SHIFT)
#  define USART_GTPR_PSC(n)       ((uint32_t)(n) << USART_GTPR_PSC_SHIFT)
#define USART_GTPR_GT_SHIFT       (8)       /* Bits 8-15: Guard time value */
#define USART_GTPR_GT_MASK        (0xff <<  USART_GTPR_GT_SHIFT)
#  define USART_GTPR_GT(n)        ((uint32_t)(n) <<  USART_GTPR_GT_SHIFT)

/* Receiver timeout register */

#define USART_RTOR_RTO_SHIFT      (0)       /* Bits 0-23: Receiver timeout value */
#define USART_RTOR_RTO_MASK       (0xffffff << USART_RTOR_RTO_SHIFT)
#  define USART_RTOR_RTO(n)       ((uint32_t)(n) << USART_RTOR_RTO_SHIFT)
#define USART_RTOR_BLEN_SHIFT     (24)      /* Bits 24-31: Block Length */
#define USART_RTOR_BLEN_MASK      (0xff << USART_RTOR_BLEN_SHIFT)
#  define USART_RTOR_BLEN(n)      ((uint32_t)(n) << USART_RTOR_BLEN_SHIFT)

/* Request register */

#define USART_RQR_ABRRQ           (1 << 0)  /* Bit 0: Auto baud rate request */
#define USART_RQR_SBKRQ           (1 << 1)  /* Bit 1: Send break request */
#define USART_RQR_MMRQ            (1 << 2)  /* Bit 2: Mute mode request */
#define USART_RQR_RXFRQ           (1 << 3)  /* Bit 3: Receive data flush request */
#define USART_RQR_TXFRQ           (1 << 4)  /* Bit 4: Transmit data flush request */

/* Interrupt & status register */

#define USART_ISR_PE              (1 << 0)  /* Bit 0:  Parity error */
#define USART_ISR_FE              (1 << 1)  /* Bit 1:  Framing error */
#define USART_ISR_NE              (1 << 2)  /* Bit 2:  Noise detected flag */
#define USART_ISR_ORE             (1 << 3)  /* Bit 3:  Overrun error */
#define USART_ISR_IDLE            (1 << 4)  /* Bit 4:  Idle line detected */
#define USART_ISR_RXNE            (1 << 5)  /* Bit 5:  Read data register not empty */
#define USART_ISR_TC              (1 << 6)  /* Bit 6:  Transmission complete */
#define USART_ISR_TXE             (1 << 7)  /* Bit 7:  Transmit data register empty */
#define USART_ISR_LBDF            (1 << 8)  /* Bit 8:  LIN break detection flag */
#define USART_ISR_CTSIF           (1 << 9)  /* Bit 9:  CTS interrupt flag */
#define USART_ISR_CTS             (1 << 10) /* Bit 10: CTS flag */
#define USART_ISR_RTOF            (1 << 11) /* Bit 11: Receiver timeout */
#define USART_ISR_EOBF            (1 << 12) /* Bit 12: End of block flag */
#define USART_ISR_UDR             (1 << 13) /* Bit 13: SPI slave underrun error flag */
#define USART_ISR_ABRE            (1 << 14) /* Bit 14: Auto baud rate error */
#define USART_ISR_ABRF            (1 << 15) /* Bit 15: Auto baud rate flag */
#define USART_ISR_BUSY            (1 << 16) /* Bit 16: Busy flag */
#define USART_ISR_CMF             (1 << 17) /* Bit 17: Character match flag */
#define USART_ISR_SBKF            (1 << 18) /* Bit 18: Send break flag */
#define USART_ISR_RWU             (1 << 19) /* Bit 19: Receiver wakeup from Mute mode */
#define USART_ISR_WUF             (1 << 20) /* Bit 20: Wakeup from low-power mode flag */
#define USART_ISR_TEACK           (1 << 21) /* Bit 21: Transmit enable acknowledge flag */
#define USART_ISR_REACK           (1 << 22) /* Bit 22: Receive enable acknowledge flag */
#define USART_ISR_TXFE            (1 << 23) /* Bit 23: TXFIFO Empty */
#define USART_ISR_RXFF            (1 << 24) /* Bit 24: RXFIFO Full */
#define USART_ISR_TCBGT           (1 << 25) /* Bit 25: Transmission complete before guard time flag */
#define USART_ISR_RXFT            (1 << 26) /* Bit 26: RXFIFO threshold flag */
#define USART_ISR_TXFT            (1 << 27) /* Bit 27: TXFIFO threshold flag */

#define USART_ISR_ALLBITS         (0x0fffffff)

/* Interrupt flag clear register */

#define USART_ICR_PECF            (1 << 0)  /* Bit 0:  Parity error clear flag */
#define USART_ICR_FECF            (1 << 1)  /* Bit 1:  Framing error clear flag */
#define USART_ICR_NCF             (1 << 2)  /* Bit 2:  Noise detected flag *clear flag */
#define USART_ICR_ORECF           (1 << 3)  /* Bit 3:  Overrun error clear flag */
#define USART_ICR_IDLECF          (1 << 4)  /* Bit 4:  Idle line detected clear flag */
#define USART_ICR_TXFECF          (1 << 5)  /* Bit 5:  TXFIFO empty clear flag */
#define USART_ICR_TCCF            (1 << 6)  /* Bit 6:  Transmission complete */
#define USART_ICR_TCBGTCF         (1 << 7)  /* Bit 7:  Transmission complete before Guard time clear flag */
#define USART_ICR_LBDCF           (1 << 8)  /* Bit 8:  LIN break detection clear flag */
#define USART_ICR_CTSCF           (1 << 9)  /* Bit 9:  CTS interrupt clear flag */
#define USART_ICR_RTOCF           (1 << 11) /* Bit 11: Receiver timeout clear flag */
#define USART_ICR_EOBCF           (1 << 12) /* Bit 12: End of block clear flag */
#define USART_ICR_UDRCF           (1 << 13) /* Bit 13:SPI slave underrun clear flag */
#define USART_ICR_CMCF            (1 << 17) /* Bit 17: Character match clear flag */
#define USART_ICR_WUCF            (1 << 20) /* Bit 20: Wakeup from low-power mode clear flag */

#define USART_ICR_ALLBITS         (0x00123b7f)

/* Receive data register */

#define USART_RDR_SHIFT           (0)       /* Bits 0-8: Receive data value */
#define USART_RDR_MASK            (0x1ff << USART_RDR_SHIFT)

/* Transmit data register */

#define USART_TDR_SHIFT           (0)       /* Bits 0-8: Transmit data value */
#define USART_TDR_MASK            (0x1ff << USART_TDR_SHIFT)

/* Prescaler register */

#define USART_PRESC_SHIFT         (0)       /* Bits 0-3: Clock prescaler */
#define USART_PRESC_MASK          (15 << USART_PRESC_SHIFT)
# define USART_PRESC_NODIV        (0 << USART_PRESC_SHIFT)  /* Input clock not divided */
# define USART_PRESC_DIV1         (1 << USART_PRESC_SHIFT)  /* Input clock divided by 2 */
# define USART_PRESC_DIV4         (2 << USART_PRESC_SHIFT)  /* Input clock divided by 4 */
# define USART_PRESC_DIV6         (3 << USART_PRESC_SHIFT)  /* Input clock divided by 6 */
# define USART_PRESC_DIV8         (4 << USART_PRESC_SHIFT)  /* Input clock divided by 8 */
# define USART_PRESC_DIV10        (5 << USART_PRESC_SHIFT)  /* Input clock divided by 10 */
# define USART_PRESC_DIV12        (6 << USART_PRESC_SHIFT)  /* Input clock divided by 12 */
# define USART_PRESC_DIV16        (7 << USART_PRESC_SHIFT)  /* Input clock divided by 16 */
# define USART_PRESC_DIV32        (8 << USART_PRESC_SHIFT)  /* Input clock divided by 32 */
# define USART_PRESC_DIV64        (9 << USART_PRESC_SHIFT)  /* Input clock divided by 64 */
# define USART_PRESC_DIV128       (10 << USART_PRESC_SHIFT) /* Input clock divided by 128 */
# define USART_PRESC_DIV256       (11 << USART_PRESC_SHIFT) /* Input clock divided by 256 */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_UART_V2_H */
