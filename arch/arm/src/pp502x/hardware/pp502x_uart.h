/****************************************************************************
 * arch/arm/src/pp502x/hardware/pp502x_uart.h
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

#ifndef __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_UART_H
#define __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/pp502x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* 16550 compatible registers, one per 32-bit word */

#define PP502X_UART_THR_OFFSET  0x0000  /* Transmit holding (DLL if DLAB) */
#define PP502X_UART_IER_OFFSET  0x0004  /* Interrupt enable (DLM if DLAB) */
#define PP502X_UART_FCR_OFFSET  0x0008  /* FIFO control, interrupt id */
#define PP502X_UART_LCR_OFFSET  0x000c  /* Line control */
#define PP502X_UART_MCR_OFFSET  0x0010  /* Modem control */
#define PP502X_UART_LSR_OFFSET  0x0014  /* Line status */

/* Register Bitfield Definitions ********************************************/

/* LSR */

#define PP502X_UART_LSR_THRE    (1 << 5)  /* Transmit holding empty */

#endif /* __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_UART_H */
