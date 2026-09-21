/****************************************************************************
 * arch/risc-v/src/erbium/hardware/erbium_uart.h
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

#ifndef __ARCH_RISCV_SRC_ERBIUM_HARDWARE_ERBIUM_UART_H
#define __ARCH_RISCV_SRC_ERBIUM_HARDWARE_ERBIUM_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Shakti UART registers on the Erbium peripheral bus. */

#define ERBIUM_UART_BAUD_OFFSET          0x00
#define ERBIUM_UART_TX_OFFSET            0x08
#define ERBIUM_UART_RX_OFFSET            0x10
#define ERBIUM_UART_STATUS_OFFSET        0x18
#define ERBIUM_UART_CONTROL_OFFSET       0x28
#define ERBIUM_UART_IEN_OFFSET           0x30
#define ERBIUM_UART_RX_THRESHOLD_OFFSET  0x40

#define ERBIUM_UART_STATUS_TX_EMPTY      (1u << 0)
#define ERBIUM_UART_STATUS_TX_FULL       (1u << 1)
#define ERBIUM_UART_STATUS_RX_NOT_EMPTY  (1u << 2)

#define ERBIUM_UART_IEN_TX_EMPTY         (1u << 0)
#define ERBIUM_UART_IEN_RX_NOT_EMPTY     (1u << 2)
#define ERBIUM_UART_CONTROL_8N1          (8u << 5)

#endif /* __ARCH_RISCV_SRC_ERBIUM_HARDWARE_ERBIUM_UART_H */
