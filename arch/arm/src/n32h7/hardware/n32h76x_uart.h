/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h76x_uart.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_UART_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/n32h7_memorymap.h"

#if defined(CONFIG_N32H7_N32H76X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define N32_USART_CTRL1_OFFSET    0x0000  /* Control register 1 */
#define N32_USART_CTRL2_OFFSET    0x0004  /* Control register 2 */
#define N32_USART_CTRL3_OFFSET    0x0008  /* Control register 3 */
#define N32_USART_STS_OFFSET      0x000C  /* Status register */
#define N32_USART_DAT_OFFSET      0x0010  /* Data register */
#define N32_USART_BRCF_OFFSET     0x0014  /* Baud Rate Configuration register */
#define N32_USART_GTP_OFFSET      0x0018  /* Guard Time and Prescaler register */
#define N32_USART_FIFO_OFFSET     0x001C  /* FIFO Configuration register */
#define N32_USART_IFW_OFFSET      0x0020  /* Idle Frame Width register */
#define N32_USART_RTO_OFFSET      0x0024  /* Receiver Timeout register */

/* Register Addresses *******************************************************/

#if N32H7_NUSART > 0
#  define N32_USART1_CTRL1        (N32_USART1_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_USART1_CTRL2        (N32_USART1_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_USART1_CTRL3        (N32_USART1_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_USART1_STS          (N32_USART1_BASE + N32_USART_STS_OFFSET)
#  define N32_USART1_DAT          (N32_USART1_BASE + N32_USART_DAT_OFFSET)
#  define N32_USART1_BRCF         (N32_USART1_BASE + N32_USART_BRCF_OFFSET)
#  define N32_USART1_GTP          (N32_USART1_BASE + N32_USART_GTP_OFFSET)
#  define N32_USART1_FIFO         (N32_USART1_BASE + N32_USART_FIFO_OFFSET)
#  define N32_USART1_IFW          (N32_USART1_BASE + N32_USART_IFW_OFFSET)
#  define N32_USART1_RTO          (N32_USART1_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUSART > 1
#  define N32_USART2_CTRL1        (N32_USART2_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_USART2_CTRL2        (N32_USART2_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_USART2_CTRL3        (N32_USART2_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_USART2_STS          (N32_USART2_BASE + N32_USART_STS_OFFSET)
#  define N32_USART2_DAT          (N32_USART2_BASE + N32_USART_DAT_OFFSET)
#  define N32_USART2_BRCF         (N32_USART2_BASE + N32_USART_BRCF_OFFSET)
#  define N32_USART2_GTP          (N32_USART2_BASE + N32_USART_GTP_OFFSET)
#  define N32_USART2_FIFO         (N32_USART2_BASE + N32_USART_FIFO_OFFSET)
#  define N32_USART2_IFW          (N32_USART2_BASE + N32_USART_IFW_OFFSET)
#  define N32_USART2_RTO          (N32_USART2_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUSART > 2
#  define N32_USART3_CTRL1        (N32_USART3_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_USART3_CTRL2        (N32_USART3_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_USART3_CTRL3        (N32_USART3_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_USART3_STS          (N32_USART3_BASE + N32_USART_STS_OFFSET)
#  define N32_USART3_DAT          (N32_USART3_BASE + N32_USART_DAT_OFFSET)
#  define N32_USART3_BRCF         (N32_USART3_BASE + N32_USART_BRCF_OFFSET)
#  define N32_USART3_GTP          (N32_USART3_BASE + N32_USART_GTP_OFFSET)
#  define N32_USART3_FIFO         (N32_USART3_BASE + N32_USART_FIFO_OFFSET)
#  define N32_USART3_IFW          (N32_USART3_BASE + N32_USART_IFW_OFFSET)
#  define N32_USART3_RTO          (N32_USART3_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUSART > 3
#  define N32_USART4_CTRL1        (N32_USART4_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_USART4_CTRL2        (N32_USART4_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_USART4_CTRL3        (N32_USART4_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_USART4_STS          (N32_USART4_BASE + N32_USART_STS_OFFSET)
#  define N32_USART4_DAT          (N32_USART4_BASE + N32_USART_DAT_OFFSET)
#  define N32_USART4_BRCF         (N32_USART4_BASE + N32_USART_BRCF_OFFSET)
#  define N32_USART4_GTP          (N32_USART4_BASE + N32_USART_GTP_OFFSET)
#  define N32_USART4_FIFO         (N32_USART4_BASE + N32_USART_FIFO_OFFSET)
#  define N32_USART4_IFW          (N32_USART4_BASE + N32_USART_IFW_OFFSET)
#  define N32_USART4_RTO          (N32_USART4_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUSART > 4
#  define N32_USART5_CTRL1        (N32_USART5_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_USART5_CTRL2        (N32_USART5_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_USART5_CTRL3        (N32_USART5_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_USART5_STS          (N32_USART5_BASE + N32_USART_STS_OFFSET)
#  define N32_USART5_DAT          (N32_USART5_BASE + N32_USART_DAT_OFFSET)
#  define N32_USART5_BRCF         (N32_USART5_BASE + N32_USART_BRCF_OFFSET)
#  define N32_USART5_GTP          (N32_USART5_BASE + N32_USART_GTP_OFFSET)
#  define N32_USART5_FIFO         (N32_USART5_BASE + N32_USART_FIFO_OFFSET)
#  define N32_USART5_IFW          (N32_USART5_BASE + N32_USART_IFW_OFFSET)
#  define N32_USART5_RTO          (N32_USART5_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUSART > 5
#  define N32_USART6_CTRL1        (N32_USART6_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_USART6_CTRL2        (N32_USART6_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_USART6_CTRL3        (N32_USART6_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_USART6_STS          (N32_USART6_BASE + N32_USART_STS_OFFSET)
#  define N32_USART6_DAT          (N32_USART6_BASE + N32_USART_DAT_OFFSET)
#  define N32_USART6_BRCF         (N32_USART6_BASE + N32_USART_BRCF_OFFSET)
#  define N32_USART6_GTP          (N32_USART6_BASE + N32_USART_GTP_OFFSET)
#  define N32_USART6_FIFO         (N32_USART6_BASE + N32_USART_FIFO_OFFSET)
#  define N32_USART6_IFW          (N32_USART6_BASE + N32_USART_IFW_OFFSET)
#  define N32_USART6_RTO          (N32_USART6_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUSART > 6
#  define N32_USART7_CTRL1        (N32_USART7_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_USART7_CTRL2        (N32_USART7_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_USART7_CTRL3        (N32_USART7_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_USART7_STS          (N32_USART7_BASE + N32_USART_STS_OFFSET)
#  define N32_USART7_DAT          (N32_USART7_BASE + N32_USART_DAT_OFFSET)
#  define N32_USART7_BRCF         (N32_USART7_BASE + N32_USART_BRCF_OFFSET)
#  define N32_USART7_GTP          (N32_USART7_BASE + N32_USART_GTP_OFFSET)
#  define N32_USART7_FIFO         (N32_USART7_BASE + N32_USART_FIFO_OFFSET)
#  define N32_USART7_IFW          (N32_USART7_BASE + N32_USART_IFW_OFFSET)
#  define N32_USART7_RTO          (N32_USART7_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUSART > 7
#  define N32_USART8_CTRL1        (N32_USART8_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_USART8_CTRL2        (N32_USART8_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_USART8_CTRL3        (N32_USART8_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_USART8_STS          (N32_USART8_BASE + N32_USART_STS_OFFSET)
#  define N32_USART8_DAT          (N32_USART8_BASE + N32_USART_DAT_OFFSET)
#  define N32_USART8_BRCF         (N32_USART8_BASE + N32_USART_BRCF_OFFSET)
#  define N32_USART8_GTP          (N32_USART8_BASE + N32_USART_GTP_OFFSET)
#  define N32_USART8_FIFO         (N32_USART8_BASE + N32_USART_FIFO_OFFSET)
#  define N32_USART8_IFW          (N32_USART8_BASE + N32_USART_IFW_OFFSET)
#  define N32_USART8_RTO          (N32_USART8_BASE + N32_USART_RTO_OFFSET)
#endif

/* UART Modules (e.g., UART9, UART10) */

#if N32H7_NUART > 0
#  define N32_UART9_CTRL1         (N32_UART9_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_UART9_CTRL2         (N32_UART9_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_UART9_CTRL3         (N32_UART9_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_UART9_STS           (N32_UART9_BASE + N32_USART_STS_OFFSET)
#  define N32_UART9_DAT           (N32_UART9_BASE + N32_USART_DAT_OFFSET)
#  define N32_UART9_BRCF          (N32_UART9_BASE + N32_USART_BRCF_OFFSET)
#  define N32_UART9_GTP           (N32_UART9_BASE + N32_USART_GTP_OFFSET)
#  define N32_UART9_FIFO          (N32_UART9_BASE + N32_USART_FIFO_OFFSET)
#  define N32_UART9_IFW           (N32_UART9_BASE + N32_USART_IFW_OFFSET)
#  define N32_UART9_RTO           (N32_UART9_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUART > 1
#  define N32_UART10_CTRL1        (N32_UART10_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_UART10_CTRL2        (N32_UART10_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_UART10_CTRL3        (N32_UART10_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_UART10_STS          (N32_UART10_BASE + N32_USART_STS_OFFSET)
#  define N32_UART10_DAT          (N32_UART10_BASE + N32_USART_DAT_OFFSET)
#  define N32_UART10_BRCF         (N32_UART10_BASE + N32_USART_BRCF_OFFSET)
#  define N32_UART10_GTP          (N32_UART10_BASE + N32_USART_GTP_OFFSET)
#  define N32_UART10_FIFO         (N32_UART10_BASE + N32_USART_FIFO_OFFSET)
#  define N32_UART10_IFW          (N32_UART10_BASE + N32_USART_IFW_OFFSET)
#  define N32_UART10_RTO          (N32_UART10_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUART > 2
#  define N32_UART11_CTRL1        (N32_UART11_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_UART11_CTRL2        (N32_UART11_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_UART11_CTRL3        (N32_UART11_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_UART11_STS          (N32_UART11_BASE + N32_USART_STS_OFFSET)
#  define N32_UART11_DAT          (N32_UART11_BASE + N32_USART_DAT_OFFSET)
#  define N32_UART11_BRCF         (N32_UART11_BASE + N32_USART_BRCF_OFFSET)
#  define N32_UART11_GTP          (N32_UART11_BASE + N32_USART_GTP_OFFSET)
#  define N32_UART11_FIFO         (N32_UART11_BASE + N32_USART_FIFO_OFFSET)
#  define N32_UART11_IFW          (N32_UART11_BASE + N32_USART_IFW_OFFSET)
#  define N32_UART11_RTO          (N32_UART11_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUART > 3
#  define N32_UART12_CTRL1        (N32_UART12_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_UART12_CTRL2        (N32_UART12_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_UART12_CTRL3        (N32_UART12_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_UART12_STS          (N32_UART12_BASE + N32_USART_STS_OFFSET)
#  define N32_UART12_DAT          (N32_UART12_BASE + N32_USART_DAT_OFFSET)
#  define N32_UART12_BRCF         (N32_UART12_BASE + N32_USART_BRCF_OFFSET)
#  define N32_UART12_GTP          (N32_UART12_BASE + N32_USART_GTP_OFFSET)
#  define N32_UART12_FIFO         (N32_UART12_BASE + N32_USART_FIFO_OFFSET)
#  define N32_UART12_IFW          (N32_UART12_BASE + N32_USART_IFW_OFFSET)
#  define N32_UART12_RTO          (N32_UART12_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUART > 4
#  define N32_UART13_CTRL1        (N32_UART13_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_UART13_CTRL2        (N32_UART13_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_UART13_CTRL3        (N32_UART13_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_UART13_STS          (N32_UART13_BASE + N32_USART_STS_OFFSET)
#  define N32_UART13_DAT          (N32_UART13_BASE + N32_USART_DAT_OFFSET)
#  define N32_UART13_BRCF         (N32_UART13_BASE + N32_USART_BRCF_OFFSET)
#  define N32_UART13_GTP          (N32_UART13_BASE + N32_USART_GTP_OFFSET)
#  define N32_UART13_FIFO         (N32_UART13_BASE + N32_USART_FIFO_OFFSET)
#  define N32_UART13_IFW          (N32_UART13_BASE + N32_USART_IFW_OFFSET)
#  define N32_UART13_RTO          (N32_UART13_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUART > 5
#  define N32_UART14_CTRL1        (N32_UART14_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_UART14_CTRL2        (N32_UART14_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_UART14_CTRL3        (N32_UART14_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_UART14_STS          (N32_UART14_BASE + N32_USART_STS_OFFSET)
#  define N32_UART14_DAT          (N32_UART14_BASE + N32_USART_DAT_OFFSET)
#  define N32_UART14_BRCF         (N32_UART14_BASE + N32_USART_BRCF_OFFSET)
#  define N32_UART14_GTP          (N32_UART14_BASE + N32_USART_GTP_OFFSET)
#  define N32_UART14_FIFO         (N32_UART14_BASE + N32_USART_FIFO_OFFSET)
#  define N32_UART14_IFW          (N32_UART14_BASE + N32_USART_IFW_OFFSET)
#  define N32_UART14_RTO          (N32_UART14_BASE + N32_USART_RTO_OFFSET)
#endif

#if N32H7_NUART > 6
#  define N32_UART15_CTRL1        (N32_UART15_BASE + N32_USART_CTRL1_OFFSET)
#  define N32_UART15_CTRL2        (N32_UART15_BASE + N32_USART_CTRL2_OFFSET)
#  define N32_UART15_CTRL3        (N32_UART15_BASE + N32_USART_CTRL3_OFFSET)
#  define N32_UART15_STS          (N32_UART15_BASE + N32_USART_STS_OFFSET)
#  define N32_UART15_DAT          (N32_UART15_BASE + N32_USART_DAT_OFFSET)
#  define N32_UART15_BRCF         (N32_UART15_BASE + N32_USART_BRCF_OFFSET)
#  define N32_UART15_GTP          (N32_UART15_BASE + N32_USART_GTP_OFFSET)
#  define N32_UART15_FIFO         (N32_UART15_BASE + N32_USART_FIFO_OFFSET)
#  define N32_UART15_IFW          (N32_UART15_BASE + N32_USART_IFW_OFFSET)
#  define N32_UART15_RTO          (N32_UART15_BASE + N32_USART_RTO_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* USART Control Register 1 (USART_CTRL1) */
#define USART_CTRL1_UEN              (1 << 0)  /* Bit 0: USART enable */
#define USART_CTRL1_RXEN             (1 << 1)  /* Bit 1: Receiver enable */
#define USART_CTRL1_TXEN             (1 << 2)  /* Bit 2: Transmitter enable */
#define USART_CTRL1_PSEL             (1 << 3)  /* Bit 3: Parity selection */
#define USART_CTRL1_PCEN             (1 << 4)  /* Bit 4: Parity control enable */
#define USART_CTRL1_WL               (1 << 5)  /* Bit 5: Word length */
#define USART_CTRL1_RCVWU            (1 << 6)  /* Bit 6: Receiver wakeup control */
#define USART_CTRL1_WUM              (1 << 7)  /* Bit 7: Wakeup mode */
#define USART_CTRL1_IDLEIEN          (1 << 8)  /* Bit 8: IDLE interrupt enable */
#define USART_CTRL1_RXDNEIEN         (1 << 9)  /* Bit 9: RXDNE interrupt enable */
#define USART_CTRL1_TXDEIEN          (1 << 10) /* Bit 10: TXDE interrupt enable */
#define USART_CTRL1_TXCIEN           (1 << 11) /* Bit 11: TXC interrupt enable */
#define USART_CTRL1_PEIEN            (1 << 12) /* Bit 12: PE interrupt enable */
#define USART_CTRL1_SDBRK            (1 << 13) /* Bit 13: Send break character */
#define USART_CTRL1_DEM              (1 << 14) /* Bit 14: Driver enable mode */
#define USART_CTRL1_DEP              (1 << 15) /* Bit 15: Driver enable polarity */
#define USART_CTRL1_DEDT_SHIFT       (16)      /* Bits 16-20: Driver deassertion time */
#define USART_CTRL1_DEDT_MASK        (0x1F << USART_CTRL1_DEDT_SHIFT)
#define USART_CTRL1_DEDT(n)          ((n) << USART_CTRL1_DEDT_SHIFT)

#define USART_CTRL1_ALLINTS          (USART_CTRL1_IDLEIEN|USART_CTRL1_RXDNEIEN| \
                                     USART_CTRL1_TXDEIEN|USART_CTRL1_TXCIEN| \
                                     USART_CTRL1_PEIEN)

#define USART_CTRL1_DEAT_SHIFT       (21)      /* Bits 21-25: Driver assertion time */
#define USART_CTRL1_DEAT_MASK        (0x1F << USART_CTRL1_DEAT_SHIFT)
#define USART_CTRL1_DEAT(n)          ((n) << USART_CTRL1_DEAT_SHIFT)

#define USART_CTRL1_OSPM             (1 << 26) /* Bit 26: Oversampling mode */
#define USART_CTRL1_SWAP             (1 << 27) /* Bit 27: TX/RX pins swap */
#define USART_CTRL1_IFCEN            (1 << 28) /* Bit 28: Idle frame controllable */

/* USART Control Register 2 (USART_CTRL2) */
#define USART_CTRL2_ADDR_SHIFT       (0)       /* Bits 0-3: USART address */
#define USART_CTRL2_ADDR_MASK        (0xF << USART_CTRL2_ADDR_SHIFT)
#define USART_CTRL2_ADDR(n)          ((n) << USART_CTRL2_ADDR_SHIFT)

#define USART_CTRL2_STPB_SHIFT       (5)       /* Bits 5-6: STOP bits */
#define USART_CTRL2_STPB_MASK        (0x3 << USART_CTRL2_STPB_SHIFT)
#define USART_CTRL2_STPB(n)          ((n) << USART_CTRL2_STPB_SHIFT)

#define USART_CTRL2_CLKEN            (1 << 8)  /* Bit 8: Clock enable */
#define USART_CTRL2_CLKPOL           (1 << 9)  /* Bit 9: Clock polarity */
#define USART_CTRL2_CLKPHA           (1 << 10) /* Bit 10: Clock phase */
#define USART_CTRL2_LBCLK            (1 << 11) /* Bit 11: Last bit clock pulse */
#define USART_CTRL2_LINMEN           (1 << 12) /* Bit 12: LIN mode enable */
#define USART_CTRL2_LINBDIEN         (1 << 13) /* Bit 13: LIN break detect intr */
#define USART_CTRL2_LINBDL           (1 << 14) /* Bit 14: LIN break detect len */
#define USART_CTRL2_RTOEN            (1 << 15) /* Bit 15: Receiver timeout en */
#define USART_CTRL2_RTOCF            (1 << 16) /* Bit 16: Receiver timeout clear */
#define USART_CTRL2_RTOIEN           (1 << 17) /* Bit 17: RTO interrupt enable */
#define USART_CTRL2_PEFLOSE          (1 << 18) /* Bit 18: PEF discard enable */
#define USART_CTRL2_NEFLOSE          (1 << 19) /* Bit 19: NEF discard enable */
#define USART_CTRL2_FEFLOSE          (1 << 20) /* Bit 20: FEF discard enable */

#define USART_CTRL2_ALLINTS          (USART_CTRL2_RTOIEN)

/* USART Control Register 3 (USART_CTRL3) */
#define USART_CTRL3_CTSEN            (1 << 0)  /* Bit 0: CTS enable */
#define USART_CTRL3_CTSIEN           (1 << 1)  /* Bit 1: CTS interrupt enable */
#define USART_CTRL3_RTSEN            (1 << 2)  /* Bit 2: RTS enable */
#define USART_CTRL3_HDMEN            (1 << 3)  /* Bit 3: Half-duplex mode en */
#define USART_CTRL3_DMATXEN          (1 << 4)  /* Bit 4: DMA transmitter en */
#define USART_CTRL3_DMARXEN          (1 << 5)  /* Bit 5: DMA receiver enable */
#define USART_CTRL3_ERRIEN           (1 << 6)  /* Bit 6: Error interrupt en */
#define USART_CTRL3_IRDAMEN          (1 << 7)  /* Bit 7: IrDA mode enable */
#define USART_CTRL3_IRDALP           (1 << 8)  /* Bit 8: IrDA low-power mode */
#define USART_CTRL3_SCMEN            (1 << 9)  /* Bit 9: Smartcard mode en */
#define USART_CTRL3_SCNACK           (1 << 10) /* Bit 10: Smartcard NACK en */

#define USART_CTRL3_ALLINTS          (USART_CTRL3_CTSIEN|USART_CTRL3_ERRIEN)

/* USART Status Register (USART_STS) */
#define USART_STS_TXFF               (1 << 0)  /* Bit 0: TX FIFO full */
#define USART_STS_RXFF               (1 << 1)  /* Bit 1: RX FIFO full */
#define USART_STS_TXFE               (1 << 2)  /* Bit 2: TX FIFO empty */
#define USART_STS_RXFE               (1 << 3)  /* Bit 3: RX FIFO empty */
#define USART_STS_RXFT               (1 << 4)  /* Bit 4: RX FIFO threshold */
#define USART_STS_TXFT               (1 << 5)  /* Bit 5: TX FIFO threshold */
#define USART_STS_IDLEF              (1 << 6)  /* Bit 6: IDLE line detected */
#define USART_STS_TXDE               (1 << 7)  /* Bit 7: TX data empty */
#define USART_STS_TXC                (1 << 8)  /* Bit 8: Transmission complete */
#define USART_STS_RXDNE              (1 << 9)  /* Bit 9: RX data not empty */
#define USART_STS_CTSF               (1 << 10) /* Bit 10: CTS flag */
#define USART_STS_LINBDF             (1 << 11) /* Bit 11: LIN break detect */
#define USART_STS_PEF                (1 << 12) /* Bit 12: Parity error */
#define USART_STS_OREF               (1 << 13) /* Bit 13: Overrun error */
#define USART_STS_NEF                (1 << 14) /* Bit 14: Noise error */
#define USART_STS_FEF                (1 << 15) /* Bit 15: Framing error */
#define USART_STS_RTOF               (1 << 16) /* Bit 16: Receiver timeout */
#define USART_STS_PELOSEF            (1 << 17) /* Bit 17: PE discard flag */
#define USART_STS_NEELOSEF           (1 << 18) /* Bit 18: NE discard flag */
#define USART_STS_FEELOSEF           (1 << 19) /* Bit 19: FE discard flag */

/* USART Data Register (USART_DAT) */
#define USART_DAT_DATV_SHIFT         (0)       /* Bits 0-8: Data value */
#define USART_DAT_DATV_MASK          (0x1FF << USART_DAT_DATV_SHIFT)

/* USART Baud Rate Register (USART_BRCF) */
#define USART_BRCF_DIVDEC_SHIFT      (0)       /* Bits 0-3: Fractional divider */
#define USART_BRCF_DIVDEC_MASK       (0xF << USART_BRCF_DIVDEC_SHIFT)
#define USART_BRCF_DIVDEC(n)         ((n) << USART_BRCF_DIVDEC_SHIFT)
#define USART_BRCF_DIVINT_SHIFT      (4)       /* Bits 4-15: Integer divider */
#define USART_BRCF_DIVINT_MASK       (0xFFF << USART_BRCF_DIVINT_SHIFT)
#define USART_BRCF_DIVINT(n)         ((n) << USART_BRCF_DIVINT_SHIFT)

/* USART Guard Time Register (USART_GTP) */
#define USART_GTP_PSCV_SHIFT         (0)       /* Bits 0-7: Prescaler value */
#define USART_GTP_PSCV_MASK          (0xFF << USART_GTP_PSCV_SHIFT)
#define USART_GTP_PSCV(n)            ((n) << USART_GTP_PSCV_SHIFT)
#define USART_GTP_GTV_SHIFT          (8)       /* Bits 8-15: Guard time value */
#define USART_GTP_GTV_MASK           (0xFF << USART_GTP_GTV_SHIFT)
#define USART_GTP_GTV(n)             ((n) << USART_GTP_GTV_SHIFT)

/* USART FIFO Register (USART_FIFO) */
#define USART_FIFO_EN                (1 << 0)  /* Bit 0: FIFO enable */
#define USART_FIFO_CLR               (1 << 1)  /* Bit 1: FIFO clear */

#define USART_FIFO_TXFTCFG_SHIFT     (2)       /* Bits 2-4: TX FIFO threshold */
#define USART_FIFO_TXFTCFG_MASK      (0x7 << USART_FIFO_TXFTCFG_SHIFT)
#define USART_FIFO_TXFT_1_8          (0x0 << USART_FIFO_TXFTCFG_SHIFT) /* 000: 1/8 full */
#define USART_FIFO_TXFT_2_8          (0x1 << USART_FIFO_TXFTCFG_SHIFT) /* 001: 2/8 full */
#define USART_FIFO_TXFT_4_8          (0x2 << USART_FIFO_TXFTCFG_SHIFT) /* 010: 4/8 full */
#define USART_FIFO_TXFT_6_8          (0x3 << USART_FIFO_TXFTCFG_SHIFT) /* 011: 6/8 full */
#define USART_FIFO_TXFT_7_8          (0x4 << USART_FIFO_TXFTCFG_SHIFT) /* 100: 7/8 full */
#define USART_FIFO_TXFT_FULL         (0x5 << USART_FIFO_TXFTCFG_SHIFT) /* 101: Full */

#define USART_FIFO_RXFTCFG_SHIFT     (5)       /* Bits 5-7: RX FIFO threshold */
#define USART_FIFO_RXFTCFG_MASK      (0x7 << USART_FIFO_RXFTCFG_SHIFT)
#define USART_FIFO_RXFT_1_8          (0x0 << USART_FIFO_RXFTCFG_SHIFT) /* 000: 1/8 full */
#define USART_FIFO_RXFT_2_8          (0x1 << USART_FIFO_RXFTCFG_SHIFT) /* 001: 2/8 full */
#define USART_FIFO_RXFT_4_8          (0x2 << USART_FIFO_RXFTCFG_SHIFT) /* 010: 4/8 full */
#define USART_FIFO_RXFT_6_8          (0x3 << USART_FIFO_RXFTCFG_SHIFT) /* 011: 6/8 full */
#define USART_FIFO_RXFT_7_8          (0x4 << USART_FIFO_RXFTCFG_SHIFT) /* 100: 7/8 full */
#define USART_FIFO_RXFT_FULL         (0x5 << USART_FIFO_RXFTCFG_SHIFT) /* 101: Full */

#define USART_FIFO_TXFFIEN           (1 << 8)  /* Bit 8: TX FIFO full intr en */
#define USART_FIFO_RXFFIEN           (1 << 9)  /* Bit 9: RX FIFO full intr en */
#define USART_FIFO_TXFEIEN           (1 << 10) /* Bit 10: TX FIFO empty intr en */
#define USART_FIFO_RXFEIEN           (1 << 11) /* Bit 11: RX FIFO empty intr en */
#define USART_FIFO_RXFTIEN           (1 << 12) /* Bit 12: RX FIFO thres intr en */
#define USART_FIFO_TXFTIEN           (1 << 13) /* Bit 13: TX FIFO thres intr en */

#define USART_FIFO_RXCNT_SHIFT       (14)      /* Bits 14-17: RX FIFO count */
#define USART_FIFO_RXCNT_MASK        (0xF << USART_FIFO_RXCNT_SHIFT)
#define USART_FIFO_GET_RXCNT(n)      ((n) >> USART_FIFO_RXCNT_SHIFT)

#define USART_FIFO_TXCNT_SHIFT       (18)      /* Bits 18-21: TX FIFO count */
#define USART_FIFO_TXCNT_MASK        (0xF << USART_FIFO_TXCNT_SHIFT)
#define USART_FIFO_GET_TXCNT(n)      ((n) >> USART_FIFO_TXCNT_SHIFT)

/* USART Idle Frame Width Register (USART_IFW) */
#define USART_IFW_WIDTH_SHIFT        (0)       /* Bits 0-15: Idle frame width */
#define USART_IFW_WIDTH_MASK         (0xFFFF << USART_IFW_WIDTH_SHIFT)

/* USART Timeout Register (USART_RTO) */
#define USART_RTO_TIME_SHIFT         (0)       /* Bits 0-27: Timeout value */
#define USART_RTO_TIME_MASK          (0xFFFFFFF << USART_RTO_TIME_SHIFT)

#endif /* CONFIG_N32H7_N32H76X */
#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_UART_H */