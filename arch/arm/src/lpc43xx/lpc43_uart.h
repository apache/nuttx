/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_uart.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_UART_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/lpc43_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization of the serial console.
 *
 ****************************************************************************/

void lpc43_lowsetup(void);

/****************************************************************************
 * Name: lpc43_usart0_reset, lpc43_uart1_reset, lpc43_usart2_reset, and
 *       lpc43_usart3_reset
 *
 * Description:
 *   Reset a U[S]ART.  These functions are used by the serial driver when a
 *   U[S]ART is closed.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_USART0
void lpc43_usart0_reset(void);
#endif
#ifdef CONFIG_LPC43_UART1
void lpc43_uart1_reset(void);
#endif
#ifdef CONFIG_LPC43_USART2
void lpc43_usart2_reset(void);
#endif
#ifdef CONFIG_LPC43_USART3
void lpc43_usart3_reset(void);
#endif

/****************************************************************************
 * Name: lpc43_usart0_setup, lpc43_uart1_setup, lpc43_usart2_setup, and
 *       lpc43_usart3_setup
 *
 * Description:
 *   Configure the U[S]ART.  This involves:
 *
 *   1. Connecting the input clock to the U[S]ART as specified in the
 *      board.h file,
 *   2. Configuring the U[S]ART pins
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_USART0
void lpc43_usart0_setup(void);
#endif

#ifdef CONFIG_LPC43_UART1
void lpc43_uart1_setup(void);
#endif

#ifdef CONFIG_LPC43_USART2
void lpc43_usart2_setup(void);
#endif

#ifdef CONFIG_LPC43_USART3
void lpc43_usart3_setup(void);
#endif

/****************************************************************************
 * Name: lpc43_setbaud
 *
 * Description:
 *   Configure the U[S]ART divisors to accomplish the desired BAUD given the
 *   U[S]ART base frequency.
 *
 *   This computationally intensive algorithm is based on the same logic
 *   used in the NXP sample code.
 *
 ****************************************************************************/

void lpc43_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_UART_H */
