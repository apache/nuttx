/************************************************************************************
 * arch/z80/src/z180/z180_config.h
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
 ************************************************************************************/

#ifndef __ARCH_Z80_SRC_Z180_Z180_CONFIG_H
#define __ARCH_Z80_SRC_Z180_Z180_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/z180/chip.h>

#include "z80_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Verify that selected features match the capability of the selected CPU */

#ifndef HAVE_Z8X181
#  undef CONFIG_Z180_SCC
#  undef CONFIG_Z180_CTC
#endif

#ifndef HAVE_Z8X182
#  undef CONFIG_Z180_ESCCA
#  undef CONFIG_Z180_ESCCB
#  undef CONFIG_Z180_PORTC
#  undef CONFIG_Z180_MIMIC
#endif

#if !defined(HAVE_Z8X181) && !defined(HAVE_Z8X182)
#  undef CONFIG_Z180_PORTA
#  undef CONFIG_Z180_PORTB
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART
#undef HAVE_SCC
#undef HAVE_SERIAL

#if defined(CONFIG_Z180_UART0) || defined(CONFIG_Z180_UART1)
#  define HAVE_UART   1
#  define HAVE_SERIAL 1
#endif

#if defined(CONFIG_Z180_SCC) || defined(CONFIG_Z180_ESCCA) || \
    defined(CONFIG_Z180_ESCCB)
#  define HAVE_SCC    1
#  define HAVE_SERIAL 1
#endif

/* Make sure all features are disabled for disabled UARTs/[E]SCC channels.  This
 * simplifies checking later.
 */

#ifndef CONFIG_Z180_UART0
#  undef CONFIG_Z180_UART0_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z180_UART1
#  undef CONFIG_Z180_UART1_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z180_SCC
#  undef CONFIG_SCC_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z180_ESCCA
#  undef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z180_ESCCB
#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE
#endif

/* Is there a serial console? There should be at most one defined. */

#undef HAVE_UART_CONSOLE
#undef HAVE_SCC_CONSOLE
#undef HAVE_SERIAL_CONSOLE

#if defined(CONFIG_Z180_UART0_SERIAL_CONSOLE)
#  define HAVE_UART_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

  /* Disable other console selections */

#  undef CONFIG_Z180_UART1_SERIAL_CONSOLE
#  undef CONFIG_Z180_SCC_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE

/* If we are not using the serial driver, then the serial console is all
 * that we will support.
 */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART1
#    undef CONFIG_Z180_SCC
#    undef CONFIG_Z180_ESCCA
#    undef CONFIG_Z180_ESCCB
#  endif

#elif defined(CONFIG_Z180_UART1_SERIAL_CONSOLE)
#  define HAVE_UART_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

/* Disable other console selections */

#  undef CONFIG_Z180_SCC_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE

/* If we are not using the serial driver, then the serial console is all
 * that we will support.
 */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART0
#    undef CONFIG_Z180_SCC
#    undef CONFIG_Z180_ESCCA
#    undef CONFIG_Z180_ESCCB
#  endif

#elif defined(CONFIG_Z180_SCC_SERIAL_CONSOLE)
#  define HAVE_SCC_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

/* Disable other console selections */

#  undef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE

/* If we are not using the serial driver, then the serial console is all
 * that we will support.
 */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART0
#    undef CONFIG_Z180_UART1
#    undef CONFIG_Z180_ESCCA
#    undef CONFIG_Z180_ESCCB
#  endif

#elif defined(CONFIG_Z180_ESCCA_SERIAL_CONSOLE)
#  define HAVE_SCC_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

/* Disable other console selections */

#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE

/* If we are not using the serial driver, then the serial console is all
 * that we will support.
 */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART0
#    undef CONFIG_Z180_UART1
#    undef CONFIG_Z180_SCC
#    undef CONFIG_Z180_ESCCB
#  endif

/* If we are not using the serial driver, then the serial console is all
 * that we will support.
 */

#elif defined(CONFIG_Z180_ESCCB_SERIAL_CONSOLE)
#  define HAVE_SCC_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

/* If we are not using the serial driver, then the serial console is all
 * that we will support.
 */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART0
#    undef CONFIG_Z180_UART1
#    undef CONFIG_Z180_SCC
#    undef CONFIG_Z180_ESCCA
#  endif

#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions Prototypes
 ************************************************************************************/

#endif /* __ARCH_Z80_SRC_Z180_Z180_CONFIG_H */
