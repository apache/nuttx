/****************************************************************************
 * include/nuttx/serial/uart_cmsdk.h
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

#ifndef __INCLUDE_NUTTX_SERIAL_UART_CMSDK_H
#define __INCLUDE_NUTTX_SERIAL_UART_CMSDK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define UART_RBR_OFFSET                   0 /* Receiver Buffer Register */
#define UART_THR_OFFSET                   0 /* Transmit Holding Register */
#define UART_STATE_OFFSET                 1 /* Interrupt State Register */
#define UART_CTRL_OFFSET                  2 /* Interrupt Control Register */
#define UART_INTSTS_OFFSET                3 /* Interrupt Status Clear Register */
#define UART_BAUDDIV_OFFSET               4 /* Baud rate divider Register */

/* Register bit definitions */

#define UART_STATE_TX_BUF_FULL            (1 << 0)
#define UART_STATE_RX_BUF_FULL            (1 << 1)
#define UART_STATE_TX_BUF_OVERRUN         (1 << 2)
#define UART_STATE_RX_BUF_OVERRUN         (1 << 3)

#define UART_CTRL_TX_ENABLE               (1 << 0)
#define UART_CTRL_RX_ENABLE               (1 << 1)
#define UART_CTRL_TX_INT_ENABLE           (1 << 2)
#define UART_CTRL_RX_INT_ENABLE           (1 << 3)
#define UART_CTRL_TX_OVERRUN_INT_ENABLE   (1 << 4)
#define UART_CTRL_RX_OVERRUN_INT_ENABLE   (1 << 5)
#define UART_CTRL_TSTMODE_ENABLE          (1 << 6)
#define UART_CTRL_ALLIE                   (0x3C)

#define UART_INTSTATUS_TX                 (1 << 0)
#define UART_INTSTATUS_RX                 (1 << 1)
#define UART_INTSTATUS_TX_OVERRUN         (1 << 2)
#define UART_INTSTATUS_RX_OVERRUN         (1 << 3)

#define UART_BAUDDIV_MIN                  (16)
#define UART_BAUDDIV_MAX                  (0xfffff)

#if defined(CONFIG_CMSDK_UART0_SERIAL_CONSOLE) && defined(CONFIG_CMSDK_UART0)
#  undef CONFIG_CMSDK_UART1_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART2_SERIAL_CONSOLE
#  define HAVE_CMSDK_CONSOLE               1
#elif defined(CONFIG_CMSDK_UART1_SERIAL_CONSOLE) && defined(CONFIG_CMSDK_UART1)
#  undef CONFIG_CMSDK_UART0_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART2_SERIAL_CONSOLE
#  define HAVE_CMSDK_CONSOLE               1
#elif defined(CONFIG_CMSDK_UART2_SERIAL_CONSOLE) && defined(CONFIG_CMSDK_UART2)
#  undef CONFIG_CMSDK_UART0_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART1_SERIAL_CONSOLE
#  define HAVE_CMSDK_CONSOLE               1
#else
#  undef CONFIG_CMSDK_UART0_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART1_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART2_SERIAL_CONSOLE
#  undef HAVE_CMSDK_CONSOLE
#endif

/****************************************************************************
 * Public functions prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cmsdk_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before uart_serialinit.
 *
 ****************************************************************************/

void cmsdk_earlyserialinit(void);

/****************************************************************************
 * Name: cmsdk_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void cmsdk_serialinit(void);

#endif /* __INCLUDE_NUTTX_SERIAL_UART_CMSDK_H */
