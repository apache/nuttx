/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_usart.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_USART_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_USART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efr32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define EFR32_USART_EN_OFFSET       0x0004  /* Enable */
#define EFR32_USART_CTRL_OFFSET     0x0008  /* Control */
#define EFR32_USART_FRAME_OFFSET    0x000c  /* Frame format */
#define EFR32_USART_CMD_OFFSET      0x0014  /* Command */
#define EFR32_USART_STATUS_OFFSET   0x0018  /* Status */
#define EFR32_USART_CLKDIV_OFFSET   0x001c  /* Clock divider */
#define EFR32_USART_RXDATAX_OFFSET  0x0020  /* RX data with error flags */
#define EFR32_USART_RXDATA_OFFSET   0x0024  /* RX data */
#define EFR32_USART_TXDATA_OFFSET   0x003c  /* TX data */
#define EFR32_USART_IF_OFFSET       0x0048  /* Interrupt flags */
#define EFR32_USART_IEN_OFFSET      0x004c  /* Interrupt enables */
#define EFR32_USART_TIMING_OFFSET   0x0058  /* Timing */

/* Register bit definitions *************************************************/

/* Enable */

#define USART_EN_EN                 (1 << 0)

/* Control */

#define USART_CTRL_SYNC             (1 << 0)
#define USART_CTRL_LOOPBK           (1 << 1)
#define USART_CTRL_OVS_SHIFT        (5)
#define USART_CTRL_OVS_MASK         (0x3 << USART_CTRL_OVS_SHIFT)
#  define USART_CTRL_OVS_X16        (0 << USART_CTRL_OVS_SHIFT)
#  define USART_CTRL_OVS_X8         (1 << USART_CTRL_OVS_SHIFT)
#  define USART_CTRL_OVS_X6         (2 << USART_CTRL_OVS_SHIFT)
#  define USART_CTRL_OVS_X4         (3 << USART_CTRL_OVS_SHIFT)
#define USART_CTRL_MSBF             (1 << 10)
#define USART_CTRL_TXBIL            (1 << 12) /* TXBL when buffer has room */
#define USART_CTRL_RXINV            (1 << 13)
#define USART_CTRL_TXINV            (1 << 14)

/* Frame format */

#define USART_FRAME_DATABITS_SHIFT  (0)
#define USART_FRAME_DATABITS_MASK   (0xf << USART_FRAME_DATABITS_SHIFT)
#  define USART_FRAME_DATABITS(n)   ((uint32_t)((n) - 3) << USART_FRAME_DATABITS_SHIFT)
#  define USART_FRAME_DATABITS_EIGHT (5 << USART_FRAME_DATABITS_SHIFT)
#  define USART_FRAME_DATABITS_NINE (6 << USART_FRAME_DATABITS_SHIFT)
#define USART_FRAME_PARITY_SHIFT    (8)
#define USART_FRAME_PARITY_MASK     (0x3 << USART_FRAME_PARITY_SHIFT)
#  define USART_FRAME_PARITY_NONE   (0 << USART_FRAME_PARITY_SHIFT)
#  define USART_FRAME_PARITY_EVEN   (2 << USART_FRAME_PARITY_SHIFT)
#  define USART_FRAME_PARITY_ODD    (3 << USART_FRAME_PARITY_SHIFT)
#define USART_FRAME_STOPBITS_SHIFT  (12)
#define USART_FRAME_STOPBITS_MASK   (0x3 << USART_FRAME_STOPBITS_SHIFT)
#  define USART_FRAME_STOPBITS_HALF (0 << USART_FRAME_STOPBITS_SHIFT)
#  define USART_FRAME_STOPBITS_ONE  (1 << USART_FRAME_STOPBITS_SHIFT)
#  define USART_FRAME_STOPBITS_ONEANDAHALF (2 << USART_FRAME_STOPBITS_SHIFT)
#  define USART_FRAME_STOPBITS_TWO  (3 << USART_FRAME_STOPBITS_SHIFT)

/* Command */

#define USART_CMD_RXEN              (1 << 0)
#define USART_CMD_RXDIS             (1 << 1)
#define USART_CMD_TXEN              (1 << 2)
#define USART_CMD_TXDIS             (1 << 3)
#define USART_CMD_CLEARTX           (1 << 10)
#define USART_CMD_CLEARRX           (1 << 11)

/* Status */

#define USART_STATUS_RXENS          (1 << 0)
#define USART_STATUS_TXENS          (1 << 1)
#define USART_STATUS_TXC            (1 << 5)  /* TX complete */
#define USART_STATUS_TXBL           (1 << 6)  /* TX buffer level */
#define USART_STATUS_RXDATAV        (1 << 7)  /* RX data valid */
#define USART_STATUS_RXFULL         (1 << 8)
#define USART_STATUS_TXIDLE         (1 << 13)

/* Clock divider: 5 fractional bits in DIV[7:3] */

#define USART_CLKDIV_DIV_SHIFT      (3)
#define USART_CLKDIV_DIV_MASK       (0xfffff << USART_CLKDIV_DIV_SHIFT)

/* RX data with error flags */

#define USART_RXDATAX_RXDATA_MASK   (0x1ff)
#define USART_RXDATAX_PERR          (1 << 14)
#define USART_RXDATAX_FERR          (1 << 15)

/* Interrupt flags and enables */

#define USART_INT_TXC               (1 << 0)
#define USART_INT_TXBL              (1 << 1)
#define USART_INT_RXDATAV           (1 << 2)
#define USART_INT_RXFULL            (1 << 3)
#define USART_INT_RXOF              (1 << 4)
#define USART_INT_RXUF              (1 << 5)
#define USART_INT_TXOF              (1 << 6)
#define USART_INT_TXUF              (1 << 7)
#define USART_INT_PERR              (1 << 8)
#define USART_INT_FERR              (1 << 9)
#define USART_INT_TXIDLE            (1 << 13)
#define USART_INT_ALL               (0x0001ffff)

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_USART_H */
