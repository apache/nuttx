/****************************************************************************
 * arch/avr/src/avrdx/avrdx_serial.h
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

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_SERIAL_H
#define __ARCH_AVR_SRC_AVRDX_AVRDX_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "avrdx_iodefs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Base address of USARTn peripheral. USART index corresponds to the location
 * of its I/O registers in memory
 */

#define AVRDX_USART(n) (*(avr_usart_t *) (0x0800 + n * 0x20))

#ifdef CONFIG_MCU_SERIAL

/* Mapping of USART to corresponding I/O port. Uses avrdx_usart_ports
 * referenced below and defined in avr_peripherals.c
 */

#  define AVRDX_USART_PORT(n) (AVRDX_PORT(avrdx_usart_ports[n]))

/* Macro that retrieves priv member of uart_dev_s and casts it
 * to pointer to avrdx_uart_priv_s
 */

#  define AVRDX_USART_DEV_PRIV(dev) ((struct avrdx_uart_priv_s *)dev->priv)

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This struct is used in struct uart_dev_s as its priv member. */

struct avrdx_uart_priv_s
{
  /* uart_dev_s instance holding this struct is connected to USARTn
   * peripheral recorded in usart_n
   */

  uint8_t usart_n;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_MCU_SERIAL

EXTERN const IOBJ uint8_t avrdx_usart_ports[];
EXTERN const IOBJ uint8_t avrdx_usart_tx_pins[];

#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_usart_reset
 *
 * Description:
 *   Reset USARTn.
 *
 * Input Parameters:
 *   struct avrdx_uart_priv_s identifying which peripheral is to be reset
 *
 ****************************************************************************/

#ifdef CONFIG_MCU_SERIAL
void avrdx_usart_reset(struct avrdx_uart_priv_s *priv);
#endif

/****************************************************************************
 * Name: avrdx_usart_configure
 *
 * Description:
 *   Configure USARTn
 *
 * Input Parameters:
 *   struct avrdx_uart_priv_s identifying which peripheral
 *   is to be configured
 *
 ****************************************************************************/

#ifdef CONFIG_MCU_SERIAL
void avrdx_usart_configure(struct avrdx_uart_priv_s *priv);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVRDX_AVRDX_SERIAL_H */
