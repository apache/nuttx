/****************************************************************************
 * arch/arm/src/ht32f491x3/hardware/ht32f491x3_uart.h
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_UART_H
#define __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define HT32_USART_STS_OFFSET            0x00
#define HT32_USART_DT_OFFSET             0x04
#define HT32_USART_BAUDR_OFFSET          0x08
#define HT32_USART_CTRL1_OFFSET          0x0c
#define HT32_USART_CTRL2_OFFSET          0x10
#define HT32_USART_CTRL3_OFFSET          0x14
#define HT32_USART_GDIV_OFFSET           0x18
#define HT32_USART_RTOR_OFFSET           0x1c
#define HT32_USART_IFC_OFFSET            0x20

/* Register Addresses *******************************************************/

#define HT32_USART1_STS                  (HT32_USART1_BASE + HT32_USART_STS_OFFSET)
#define HT32_USART1_DT                   (HT32_USART1_BASE + HT32_USART_DT_OFFSET)
#define HT32_USART1_BAUDR                (HT32_USART1_BASE + HT32_USART_BAUDR_OFFSET)
#define HT32_USART1_CTRL1                (HT32_USART1_BASE + HT32_USART_CTRL1_OFFSET)
#define HT32_USART1_CTRL2                (HT32_USART1_BASE + HT32_USART_CTRL2_OFFSET)
#define HT32_USART1_CTRL3                (HT32_USART1_BASE + HT32_USART_CTRL3_OFFSET)

#define HT32_USART2_STS                  (HT32_USART2_BASE + HT32_USART_STS_OFFSET)
#define HT32_USART2_DT                   (HT32_USART2_BASE + HT32_USART_DT_OFFSET)
#define HT32_USART2_BAUDR                (HT32_USART2_BASE + HT32_USART_BAUDR_OFFSET)
#define HT32_USART2_CTRL1                (HT32_USART2_BASE + HT32_USART_CTRL1_OFFSET)
#define HT32_USART2_CTRL2                (HT32_USART2_BASE + HT32_USART_CTRL2_OFFSET)
#define HT32_USART2_CTRL3                (HT32_USART2_BASE + HT32_USART_CTRL3_OFFSET)

#define HT32_USART3_STS                  (HT32_USART3_BASE + HT32_USART_STS_OFFSET)
#define HT32_USART3_DT                   (HT32_USART3_BASE + HT32_USART_DT_OFFSET)
#define HT32_USART3_BAUDR                (HT32_USART3_BASE + HT32_USART_BAUDR_OFFSET)
#define HT32_USART3_CTRL1                (HT32_USART3_BASE + HT32_USART_CTRL1_OFFSET)
#define HT32_USART3_CTRL2                (HT32_USART3_BASE + HT32_USART_CTRL2_OFFSET)
#define HT32_USART3_CTRL3                (HT32_USART3_BASE + HT32_USART_CTRL3_OFFSET)

/* Status register **********************************************************/

#define HT32_USART_STS_PERR              (1 << 0)
#define HT32_USART_STS_FERR              (1 << 1)
#define HT32_USART_STS_NERR              (1 << 2)
#define HT32_USART_STS_ROERR             (1 << 3)
#define HT32_USART_STS_IDLEF             (1 << 4)
#define HT32_USART_STS_RDBF              (1 << 5)
#define HT32_USART_STS_TDC               (1 << 6)
#define HT32_USART_STS_TDBE              (1 << 7)
#define HT32_USART_STS_BFF               (1 << 8)
#define HT32_USART_STS_CTSCF             (1 << 9)
#define HT32_USART_STS_LPWUF             (1 << 20)
#define HT32_USART_STS_TXON              (1 << 21)
#define HT32_USART_STS_RXON              (1 << 22)

/* Data register ************************************************************/

#define HT32_USART_DT_SHIFT              (0)
#define HT32_USART_DT_MASK               (0x1ff << HT32_USART_DT_SHIFT)

/* Baud rate register *******************************************************/

#define HT32_USART_BAUDR_DIV_SHIFT       (0)
#define HT32_USART_BAUDR_DIV_MASK        (0xffff << HT32_USART_BAUDR_DIV_SHIFT)

/* Control register 1 *******************************************************/

#define HT32_USART_CTRL1_SBF             (1 << 0)
#define HT32_USART_CTRL1_RM              (1 << 1)
#define HT32_USART_CTRL1_REN             (1 << 2)
#define HT32_USART_CTRL1_TEN             (1 << 3)
#define HT32_USART_CTRL1_IDLEIEN         (1 << 4)
#define HT32_USART_CTRL1_RDBFIEN         (1 << 5)
#define HT32_USART_CTRL1_TDCIEN          (1 << 6)
#define HT32_USART_CTRL1_TDBEIEN         (1 << 7)
#define HT32_USART_CTRL1_PERRIEN         (1 << 8)
#define HT32_USART_CTRL1_PSEL            (1 << 9)
#define HT32_USART_CTRL1_PEN             (1 << 10)
#define HT32_USART_CTRL1_WUM             (1 << 11)
#define HT32_USART_CTRL1_DBN0            (1 << 12)
#define HT32_USART_CTRL1_UEN             (1 << 13)
#define HT32_USART_CTRL1_DBN1            (1 << 28)

/* Control register 2 *******************************************************/

#define HT32_USART_CTRL2_BFIEN           (1 << 6)
#define HT32_USART_CTRL2_LBCP            (1 << 8)
#define HT32_USART_CTRL2_CLKPHA          (1 << 9)
#define HT32_USART_CTRL2_CLKPOL          (1 << 10)
#define HT32_USART_CTRL2_CLKEN           (1 << 11)
#define HT32_USART_CTRL2_STOPBN_SHIFT    (12)
#define HT32_USART_CTRL2_STOPBN_MASK     (3 << HT32_USART_CTRL2_STOPBN_SHIFT)
#define HT32_USART_CTRL2_STOPBN_10       (0 << HT32_USART_CTRL2_STOPBN_SHIFT)
#define HT32_USART_CTRL2_STOPBN_05       (1 << HT32_USART_CTRL2_STOPBN_SHIFT)
#define HT32_USART_CTRL2_STOPBN_20       (2 << HT32_USART_CTRL2_STOPBN_SHIFT)
#define HT32_USART_CTRL2_STOPBN_15       (3 << HT32_USART_CTRL2_STOPBN_SHIFT)

/* Control register 3 *******************************************************/

#define HT32_USART_CTRL3_ERRIEN          (1 << 0)
#define HT32_USART_CTRL3_DMAREN          (1 << 6)
#define HT32_USART_CTRL3_DMATEN          (1 << 7)
#define HT32_USART_CTRL3_RTSEN           (1 << 8)
#define HT32_USART_CTRL3_CTSEN           (1 << 9)
#define HT32_USART_CTRL3_CTSCFIEN        (1 << 10)
#define HT32_USART_CTRL3_RS485EN         (1 << 14)
#define HT32_USART_CTRL3_DEP             (1 << 15)

/* Convenience aliases ******************************************************/

#define HT32_USART_RDR_OFFSET            HT32_USART_DT_OFFSET
#define HT32_USART_TDR_OFFSET            HT32_USART_DT_OFFSET

#endif /* __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_UART_H */
