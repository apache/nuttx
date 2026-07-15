/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_uart.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_UART_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USART0, UART1 and UART2 share the same register layout */

#define GD32VW55X_UART_CTL0_OFFSET    0x0000  /* Control 0 */
#define GD32VW55X_UART_CTL1_OFFSET    0x0004  /* Control 1 */
#define GD32VW55X_UART_CTL2_OFFSET    0x0008  /* Control 2 */
#define GD32VW55X_UART_BAUD_OFFSET    0x000c  /* Baud rate */
#define GD32VW55X_UART_GP_OFFSET      0x0010  /* Guard time / prescaler */
#define GD32VW55X_UART_RT_OFFSET      0x0014  /* Receiver timeout */
#define GD32VW55X_UART_CMD_OFFSET     0x0018  /* Command */
#define GD32VW55X_UART_STAT_OFFSET    0x001c  /* Status */
#define GD32VW55X_UART_INTC_OFFSET    0x0020  /* Interrupt/status clear */
#define GD32VW55X_UART_RDATA_OFFSET   0x0024  /* Receive data */
#define GD32VW55X_UART_TDATA_OFFSET   0x0028  /* Transmit data */
#define GD32VW55X_UART_CHC_OFFSET     0x00c0  /* Coherence control */

/* CTL0 register */

#define UART_CTL0_UEN                 (1 << 0)   /* USART enable */
#define UART_CTL0_REN                 (1 << 2)   /* Receiver enable */
#define UART_CTL0_TEN                 (1 << 3)   /* Transmitter enable */
#define UART_CTL0_IDLEIE              (1 << 4)   /* IDLE interrupt enable */
#define UART_CTL0_RBNEIE              (1 << 5)   /* RX not empty int enable */
#define UART_CTL0_TCIE                (1 << 6)   /* TX complete int enable */
#define UART_CTL0_TBEIE               (1 << 7)   /* TX empty int enable */
#define UART_CTL0_PERRIE              (1 << 8)   /* Parity error int enable */
#define UART_CTL0_PM                  (1 << 9)   /* Parity: 0=even 1=odd */
#define UART_CTL0_PCEN                (1 << 10)  /* Parity control enable */
#define UART_CTL0_WL0                 (1 << 12)  /* Word length bit 0 */
#define UART_CTL0_WL1                 (1 << 28)  /* Word length bit 1 */

/* CTL1 register */

#define UART_CTL1_STB_SHIFT           12         /* Stop bits */
#define UART_CTL1_STB_MASK            (3 << UART_CTL1_STB_SHIFT)
#define UART_CTL1_STB_1               (0 << UART_CTL1_STB_SHIFT)
#define UART_CTL1_STB_2               (2 << UART_CTL1_STB_SHIFT)

/* CTL2 register */

#define UART_CTL2_ERRIE               (1 << 0)   /* Error interrupt enable */
#define UART_CTL2_RTSEN               (1 << 8)   /* RTS enable */
#define UART_CTL2_CTSEN               (1 << 9)   /* CTS enable */
#define UART_CTL2_DENR                (1 << 6)   /* DMA enable receive */
#define UART_CTL2_DENT                (1 << 7)   /* DMA enable transmit */

/* STAT register */

#define UART_STAT_PERR                (1 << 0)   /* Parity error */
#define UART_STAT_FERR                (1 << 1)   /* Frame error */
#define UART_STAT_NERR                (1 << 2)   /* Noise error */
#define UART_STAT_ORERR               (1 << 3)   /* Overrun error */
#define UART_STAT_IDLEF               (1 << 4)   /* IDLE detected */
#define UART_STAT_RBNE                (1 << 5)   /* Read buffer not empty */
#define UART_STAT_TC                  (1 << 6)   /* Transmission complete */
#define UART_STAT_TBE                 (1 << 7)   /* Transmit buffer empty */

/* INTC register */

#define UART_INTC_PEC                 (1 << 0)   /* Clear parity error */
#define UART_INTC_FEC                 (1 << 1)   /* Clear frame error */
#define UART_INTC_NEC                 (1 << 2)   /* Clear noise error */
#define UART_INTC_OREC                (1 << 3)   /* Clear overrun error */
#define UART_INTC_IDLEC               (1 << 4)   /* Clear IDLE flag */
#define UART_INTC_TCC                 (1 << 6)   /* Clear TX complete */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_UART_H */
