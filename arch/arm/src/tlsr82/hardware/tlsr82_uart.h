/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_uart.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_UART_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Uart buffer register definitions */

#define UART_BUF_REG                   REG_ADDR32(0x90)
#define UART_BUF0_REG                  REG_ADDR8(0x90)
#define UART_BUF1_REG                  REG_ADDR8(0x91)
#define UART_BUF2_REG                  REG_ADDR8(0x92)
#define UART_BUF3_REG                  REG_ADDR8(0x93)

#define UART_BUF(n)                    REG_ADDR8(0x90 + (n))

/* Uart clock register definitions */

#define UART_CLK_REG                   REG_ADDR16(0x94)

#define UART_CLK_DIV                   0x7fff
#define UART_CLK_EN_SHIFT              15
#define UART_CLK_EN                    (0x1 << UART_CLK_EN_SHIFT)

/* Uart control register definitions */

#define UART_CTRL_REG                  REG_ADDR32(0x96)
#define UART_CTRL0_REG                 REG_ADDR8(0x96)
#define UART_CTRL1_REG                 REG_ADDR8(0x97)
#define UART_CTRL2_REG                 REG_ADDR8(0x98)
#define UART_CTRL3_REG                 REG_ADDR8(0x99)

#define UART_CTRL0_BWPC_SHIFT          0
#define UART_CTRL0_BWPC                (0xf << UART_CTRL0_BWPC_SHIFT)
#define UART_CTRL0_RX_DMA_EN_SHIFT     4
#define UART_CTRL0_RX_DMA_EN           (0x1 << UART_CTRL0_RX_DMA_EN_SHIFT)
#define UART_CTRL0_TX_DMA_EN_SHIFT     5
#define UART_CTRL0_TX_DMA_EN           (0x1 << UART_CTRL0_TX_DMA_EN_SHIFT)
#define UART_CTRL0_RX_IRQ_EN_SHIFT     6
#define UART_CTRL0_RX_IRQ_EN           (0x1 << UART_CTRL0_RX_IRQ_EN_SHIFT)
#define UART_CTRL0_TX_IRQ_EN_SHIFT     7
#define UART_CTRL0_TX_IRQ_EN           (0x1 << UART_CTRL0_TX_IRQ_EN_SHIFT)

#define UART_CTRL1_CTS_SEL_SHIFT       0
#define UART_CTRL1_CTS_SEL             (0x1 << UART_CTRL1_CTS_SEL_SHIFT)
#define UART_CTRL1_CTS_EN_SHIFT        1
#define UART_CTRL1_CTS_EN              (0x1 << UART_CTRL1_CTS_EN_SHIFT)
#define UART_CTRL1_PARITY_EN_SHIFT     2
#define UART_CTRL1_PARITY_EN           (0x1 << UART_CTRL1_PARITY_EN_SHIFT)
#define UART_CTRL1_PARITY_SEL_SHIFT    3
#define UART_CTRL1_PARITY_SEL          (0x1 << UART_CTRL1_PARITY_SEL_SHIFT)
#define UART_CTRL1_STOPBIT_SHIFT       4
#define UART_CTRL1_STOPBIT             (0x3 << UART_CTRL1_STOPBIT_SHIFT)
#define UART_CTRL1_TTL_SHIFT           6
#define UART_CTRL1_TTL                 (0x1 << UART_CTRL1_TTL_SHIFT)
#define UART_CTRL1_LOOPBACK_SHIFT      7
#define UART_CTRL1_LOOPBACK            (0x1 << UART_CTRL1_LOOPBACK_SHIFT)

#define UART_CTRL2_RTS_TRI_LEVEL_SHIFT 0
#define UART_CTRL2_RTS_TRI_LEVEL       (0xf << UART_CTRL2_RTS_TRI_LEVEL_SHIFT)
#define UART_CTRL2_RTS_PARITY_SHIFT    4
#define UART_CTRL2_RTS_PARITY          (0x1 << UART_CTRL2_RTS_PARITY_SHIFT)
#define UART_CTRL2_RTS_MUL_VAL_SHIFT   5
#define UART_CTRL2_RTS_MUL_VAL         (0x1 << UART_CTRL2_RTS_MUL_VAL_SHIFT)
#define UART_CTRL2_RTS_MUL_EN_SHIFT    6
#define UART_CTRL2_RTS_MUL_EN          (0x1 << UART_CTRL2_RTS_MUL_EN_SHIFT)
#define UART_CTRL2_RTS_EN_SHIFT        7
#define UART_CTRL2_RTS_EN              (0x1 << UART_CTRL2_RTS_EN_SHIFT)

#define UART_CTRL3_RX_TRI_LEVEL_SHIFT  0
#define UART_CTRL3_RX_TRI_LEVEL        (0xf << UART_CTRL3_RX_TRI_LEVEL_SHIFT)
#define UART_CTRL3_TX_TRI_LEVEL_SHIFT  4
#define UART_CTRL3_TX_TRI_LEVEL        (0xf << UART_CTRL3_TX_TRI_LEVEL_SHIFT)

/* Uart rx timeout register definitions */

#define UART_RXTIMEOUT0_REG            REG_ADDR8(0x9a)
#define UART_RXTIMEOUT1_REG            REG_ADDR8(0x9b)

#define UART_RXTIMEOUT1_SEL_SHIFT      0
#define UART_RXTIMEOUT1_SEL            (0x3 << UART_RXTIMEOUT1_SEL_SHIFT)
#define UART_RXTIMEOUT1_P7816_EN_SHIFT 5
#define UART_RXTIMEOUT1_P7816_EN       (0x1 << UART_RXTIMEOUT1_P7816_EN_SHIFT)
#define UART_RXTIMEOUT1_MASK_TXDONE_SHIFT 6
#define UART_RXTIMEOUT1_MASK_TXDONE    (0x1 << UART_RXTIMEOUT1_MASK_TXDONE_SHIFT)
#define UART_RXTIMEOUT1_MASK_ERR_SHIFT 7
#define UART_RXTIMEOUT1_MASK_ERR       (0x1 << UART_RXTIMEOUT1_MASK_ERR_SHIFT)

/* Uart buffer count register definitions */

#define UART_BUF_CNT0_REG              REG_ADDR8(0x9c)
#define UART_BUF_CNT1_REG              REG_ADDR8(0x9d)

#define UART_BUF_CNT0_RX_CNT_SHIFT     0
#define UART_BUF_CNT0_RX_CNT           (0xf << UART_BUF_CNT0_RX_CNT_SHIFT)
#define UART_BUF_CNT0_TX_CNT_SHIFT     4
#define UART_BUF_CNT0_TX_CNT           (0xf << UART_BUF_CNT0_TX_CNT_SHIFT)

#define UART_BUF_CNT1_RB_CNT           0x07
#define UART_BUF_CNT1_IRQ              BIT(3)
#define UART_BUF_CNT1_WB_CNT           0x70
#define UART_BUF_CNT1_RX_CLR           BIT(6)
#define UART_BUF_CNT1_TX_CLR           BIT(7)
#define UART_BUF_CNT1_RX_ERR           BIT(7)

#define UART_GET_RX_BUF_CNT()          ((UART_BUF_CNT0_REG & UART_BUF_CNT0_RX_CNT) >>\
                                        UART_BUF_CNT0_RX_CNT_SHIFT)
#define UART_GET_TX_BUF_CNT()          ((UART_BUF_CNT0_REG & UART_BUF_CNT0_TX_CNT) >>\
                                        UART_BUF_CNT0_TX_CNT_SHIFT)

/* Uart interupt register definitions */

#define UART_IRQ_REG                   REG_ADDR8(0x9e)

#define UART_IRQ_TXDONE                (0x1 << 0)
#define UART_IRQ_TXBUF                 (0x1 << 1)
#define UART_IRQ_RXDONE                (0x1 << 2)
#define UART_IRQ_RXBUF                 (0x1 << 3)
#define UART_IRQ_CLR_RX                (0x1 << 6)
#define UART_IRQ_CLR_TX                (0x1 << 7)

#define UART_STATE_REG                 REG_ADDR8(0x9f)

#define UART_STATE_TX                  0x07
#define UART_STATE_RX                  0xf0

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_UART_H */
