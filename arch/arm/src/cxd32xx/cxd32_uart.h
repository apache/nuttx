/****************************************************************************
 * arch/arm/src/cxd32xx/cxd32_uart.h
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

#ifndef __ARCH_ARM_SRC_CXD32XX_CXD32_UART_H
#define __ARCH_ARM_SRC_CXD32XX_CXD32_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/cxd32_uart.h"

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
 * Name: cxd32_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization of the serial console.
 *
 ****************************************************************************/

void cxd32_lowsetup(void);

/****************************************************************************
 * Name: cxd32_uart_reset
 *
 * Description:
 *   Reset a UART.  This function is used by the serial driver when a
 *   UART is closed.
 *
 ****************************************************************************/

void cxd32_uart_reset(int ch);

/****************************************************************************
 * Name: cxd32_uart_setup
 *
 * Description:
 *   Configure the UART.  This involves:
 *
 *   1. Connecting the input clock to the UART as specified in the
 *      board.h file,
 *   2. Configuring the UART pins
 *
 ****************************************************************************/

void cxd32_uart_setup(int);

/****************************************************************************
 * Name: cxd32_setbaud
 *
 * Description:
 *   Configure the UART divisors to accomplish the desired BAUD given the
 *   UART base frequency.
 *
 ****************************************************************************/

void cxd32_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud);

/****************************************************************************
 * Name: cxd32_uart_initialize
 *
 * Description:
 *   Various initial registration
 *
 ****************************************************************************/

void cxd32_uart_initialize(int ch);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD32XX_CXD32_UART_H */
