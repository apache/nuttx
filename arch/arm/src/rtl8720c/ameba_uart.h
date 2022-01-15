/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_uart.h
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

#ifndef __ARCH_ARM_SRC_RTL8720C_AMEBA_UART_H
#define __ARCH_ARM_SRC_RTL8720C_AMEBA_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hal_uart.h>
#include <hal_pinmux.h>
#ifdef CONFIG_AMEBA_UART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIGURATION */

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(CONFIG_AMEBA_UART0) || defined(CONFIG_AMEBA_UART1) || \
    defined(CONFIG_AMEBA_UART2) || defined(CONFIG_AMEBA_UART3)
#  define HAVE_UART 1
#endif
/* We need to be told the address increment between registers and
 * the register bit width.
 */
#ifndef CONFIG_AMEBA_REGINCR
#  error "CONFIG_AMEBA_REGINCR not defined"
#endif

#if CONFIG_AMEBA_REGINCR != 1 && CONFIG_AMEBA_REGINCR != 2 && CONFIG_AMEBA_REGINCR != 4
#  error "CONFIG_AMEBA_REGINCR not supported"
#endif
#ifndef CONFIG_AMEBA_REGWIDTH
#  error "CONFIG_AMEBA_REGWIDTH not defined"
#endif

#if CONFIG_AMEBA_REGWIDTH != 8 && CONFIG_AMEBA_REGWIDTH != 16 && CONFIG_AMEBA_REGWIDTH != 32
#  error "CONFIG_AMEBA_REGWIDTH not supported"
#endif
#ifndef CONFIG_AMEBA_ADDRWIDTH
#  error "CONFIG_AMEBA_ADDRWIDTH not defined"
#endif

#if CONFIG_AMEBA_ADDRWIDTH != 8 && CONFIG_AMEBA_ADDRWIDTH != 16 && CONFIG_AMEBA_ADDRWIDTH != 32
#  error "CONFIG_AMEBA_ADDRWIDTH not supported"
#endif

/* If a UART is enabled, then its base address,
 * clock, and IRQ must also be provided
 */

#ifdef CONFIG_AMEBA_UART0
#  ifndef CONFIG_AMEBA_UART0_TX_PIN
#    error "CONFIG_AMEBA_UART0_TX_PIN not provided"
#    undef CONFIG_AMEBA_UART0
#  endif
#  ifndef CONFIG_AMEBA_UART0_RX_PIN
#    error "CONFIG_AMEBA_UART0_RX_PIN not provided"
#    undef CONFIG_AMEBA_UART0
#  endif
#endif
#ifdef CONFIG_AMEBA_UART1
#  ifndef CONFIG_AMEBA_UART1_TX_PIN
#    error "CONFIG_AMEBA_UART1_TX_PIN not provided"
#    undef CONFIG_AMEBA_UART1
#  endif
#  ifndef CONFIG_AMEBA_UART1_RX_PIN
#    error "CONFIG_AMEBA_UART1_RX_PIN not provided"
#    undef CONFIG_AMEBA_UART1
#  endif
#endif
#ifdef CONFIG_AMEBA_UART2
#  ifndef CONFIG_AMEBA_UART2_TX_PIN
#    error "CONFIG_AMEBA_UART2_TX_PIN not provided"
#    undef CONFIG_AMEBA_UART2
#  endif
#  ifndef CONFIG_AMEBA_UART2_RX_PIN
#    error "CONFIG_AMEBA_UART2_RX_PIN not provided"
#    undef CONFIG_AMEBA_UART2
#  endif
#endif
#ifdef CONFIG_AMEBA_UART3
#  ifndef CONFIG_AMEBA_UART3_TX_PIN
#    error "CONFIG_AMEBA_UART3_TX_PIN not provided"
#    undef CONFIG_AMEBA_UART3
#  endif
#  ifndef CONFIG_AMEBA_UART3_RX_PIN
#    error "CONFIG_AMEBA_UART3_RX_PIN not provided"
#    undef CONFIG_AMEBA_UART3
#  endif
#endif
/* Is there a serial console? There should be at most one defined.
 * It could be on any UARTn, n=0,1,2,3
 */

#if defined(CONFIG_AMEBA_UART0_SERIAL_CONSOLE) && defined(CONFIG_AMEBA_UART0)
#  undef CONFIG_AMEBA_UART1_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART2_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART3_SERIAL_CONSOLE
#  define HAVE_AMEBA_CONSOLE 1
#elif defined(CONFIG_AMEBA_UART1_SERIAL_CONSOLE) && defined(CONFIG_AMEBA_UART1)
#  undef CONFIG_AMEBA_UART0_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART2_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART3_SERIAL_CONSOLE
#  define HAVE_AMEBA_CONSOLE 1
#elif defined(CONFIG_AMEBA_UART2_SERIAL_CONSOLE) && defined(CONFIG_AMEBA_UART2)
#  undef CONFIG_AMEBA_UART0_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART1_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART3_SERIAL_CONSOLE
#  define HAVE_AMEBA_CONSOLE 1
#elif defined(CONFIG_AMEBA_UART3_SERIAL_CONSOLE) && defined(CONFIG_AMEBA_UART3)
#  undef CONFIG_AMEBA_UART0_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART1_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART2_SERIAL_CONSOLE
#  define HAVE_AMEBA_CONSOLE 1
#else
#  undef CONFIG_AMEBA_UART0_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART1_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART2_SERIAL_CONSOLE
#  undef CONFIG_AMEBA_UART3_SERIAL_CONSOLE
#  undef HAVE_AMEBA_CONSOLE
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if CONFIG_AMEBA_REGWIDTH == 8
typedef uint8_t uart_datawidth_t;
#elif CONFIG_AMEBA_REGWIDTH == 16
typedef uint16_t uart_datawidth_t;
#elif CONFIG_AMEBA_REGWIDTH == 32
typedef uint32_t uart_datawidth_t;
#endif
#if CONFIG_AMEBA_ADDRWIDTH == 8
typedef uint8_t uart_addrwidth_t;
#elif CONFIG_AMEBA_ADDRWIDTH == 16
typedef uint16_t uart_addrwidth_t;
#elif CONFIG_AMEBA_ADDRWIDTH == 32
typedef uint32_t uart_addrwidth_t;
#endif

/****************************************************************************
 * Name: uart_getreg(), uart_putreg(), uart_ioctl()
 *
 * Description:
 *   These functions must be provided by the processor-specific code in order
 *   to correctly access AMEBA registers
 *   uart_ioctl() is optional to provide custom IOCTLs
 *
 ****************************************************************************/

struct file;  /* Forward reference */
int uart_ioctl(struct file *filep, int cmd, unsigned long arg);
#endif /* CONFIG_AMEBA_UART */
#endif /* __ARCH_ARM_SRC_RTL8720C_AMEBA_UART_H */
