/****************************************************************************
 * arch/arm/src/rtl8730e/hardware/rtl8730e_loguart.h
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

#ifndef __ARCH_ARM_SRC_RTL8730E_HARDWARE_RTL8730E_LOGUART_H
#define __ARCH_ARM_SRC_RTL8730E_HARDWARE_RTL8730E_LOGUART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AmebaSmart (RTL8730E) LOGUART is a Realtek-specific console UART shared by
 * all three cores (KM0/KM4/CA32).  It is NOT a PL011 nor a 16550.  Register
 * base and layout come from the vendor SDK (hal_platform.h /
 * ameba_loguart.h).
 * The KM4 IMG1 bootloader has already configured the baud rate before it
 * hands control to the CA32, so NuttX only needs to poll the line-status
 * register and push characters into its own transmit path FIFO.
 *
 * Each core owns a dedicated transmit path.  Per the SDK LOG_UART_IDX_FLAG
 * table, the CA32 application core (CPUID 2, labelled "CA7") uses transmit
 * path 4: THR index 3 and the LSR "path-4 FIFO not full" flag (bit 23).
 */

#define RTL8730E_LOGUART_BASE      0x4200c000  /* UARTLOG_REG_BASE, LP_APB */

#define RTL8730E_LOGUART_RBR       (RTL8730E_LOGUART_BASE + 0x024)  /* RX buffer */
#define RTL8730E_LOGUART_IER       (RTL8730E_LOGUART_BASE + 0x004)  /* Interrupt enable */
#define RTL8730E_LOGUART_LSR       (RTL8730E_LOGUART_BASE + 0x014)  /* Line status */
#define RTL8730E_LOGUART_THR3      (RTL8730E_LOGUART_BASE + 0x05c + (3 << 2))

/* IER (0x004) bits */

#define LOGUART_IER_ERBI           (1 << 0)   /* RX data available interrupt enable */

/* LSR (0x014) bits */

#define LOGUART_LSR_DRDY           (1 << 0)   /* RX data ready */
#define LOGUART_LSR_TP4F_NOT_FULL  (1 << 23)  /* CA32 tx path4 FIFO not full */
#define LOGUART_LSR_TP4F_EMPTY     (1 << 19)  /* CA32 tx path4 FIFO empty */
#define LOGUART_LSR_RXFIFO_INT     (1 << 13)  /* RX FIFO threshold reached */

#endif /* __ARCH_ARM_SRC_RTL8730E_HARDWARE_RTL8730E_LOGUART_H */
