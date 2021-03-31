/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_config.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_CONFIG_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USART can be configured as a number of different devices (Only UART is
 * supported here now, that will be extended).
 * Check for consistency between USART enable options.
 */

#if AVR32_NUSART < 1
#  undef CONFIG_AVR32_USART0
#  undef CONFIG_AVR32_USART1
#  undef CONFIG_AVR32_USART2
#endif
#if AVR32_NUSART < 2
#  undef CONFIG_AVR32_USART1
#  undef CONFIG_AVR32_USART2
#endif
#if AVR32_NUSART < 3
#  undef CONFIG_AVR32_USART2
#endif

/* Not all USART features are supported on all chips or all USARTS */

#ifdef CONFIG_ARCH_CHIP_AT32UC3B
# undef CONFIG_AVR32_USART0_RS485
# undef CONFIG_AVR32_USART0_MAN
# undef CONFIG_AVR32_USART0_MODEM
# undef CONFIG_AVR32_USART0_IRDA
# undef CONFIG_AVR32_USART0_ISO786
# undef CONFIG_AVR32_USART1_RS485
# undef CONFIG_AVR32_USART2_RS485
# undef CONFIG_AVR32_USART2_MAN
# undef CONFIG_AVR32_USART2_MODEM
# undef CONFIG_AVR32_USART2_IRDA
# undef CONFIG_AVR32_USART2_ISO786
#endif

/* Disable configurations if USART not selected in configuration file */

#ifndef CONFIG_AVR32_USART0
#  undef CONFIG_AVR32_USART0_RS232
#  undef CONFIG_AVR32_USART0_SPI
#  undef CONFIG_AVR32_USART0_RS485
#  undef CONFIG_AVR32_USART0_MAN
#  undef CONFIG_AVR32_USART0_MODEM
#  undef CONFIG_AVR32_USART0_IRDA
#  undef CONFIG_AVR32_USART0_ISO786
#endif

#ifndef CONFIG_AVR32_USART1
#  undef CONFIG_AVR32_USART1_RS232
#  undef CONFIG_AVR32_USART1_SPI
#  undef CONFIG_AVR32_USART1_RS485
#  undef CONFIG_AVR32_USART1_MAN
#  undef CONFIG_AVR32_USART1_MODEM
#  undef CONFIG_AVR32_USART1_IRDA
#  undef CONFIG_AVR32_USART1_ISO786
#endif

#ifndef CONFIG_AVR32_USART2
#  undef CONFIG_AVR32_USART2_RS232
#  undef CONFIG_AVR32_USART2_SPI
#  undef CONFIG_AVR32_USART2_RS485
#  undef CONFIG_AVR32_USART2_MAN
#  undef CONFIG_AVR32_USART2_MODEM
#  undef CONFIG_AVR32_USART2_IRDA
#  undef CONFIG_AVR32_USART2_ISO786
#endif

/* Is any UART configured? */

#if defined(CONFIG_AVR32_USART0_RS232) || \
    defined(CONFIG_AVR32_USART1_RS232) || \
    defined(CONFIG_AVR32_USART2_RS232)
#  define HAVE_RS232_DEVICE
#else
#  undef  HAVE_RS232_DEVICE
#endif

/* Is there a serial console? */

#if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_AVR32_USART0_RS232)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_AVR32_USART1_RS232)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_AVR32_USART2_RS232)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#ifndef CONFIG_DEV_CONSOLE
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#else
#  if defined(CONFIG_CONSOLE_SYSLOG)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  elif defined(HAVE_RS232_DEVICE)
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
#  else
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  endif
#endif

/* If some other device is used as the console, then the serial driver may
 * still be needed.  Let's assume that if the upper half serial driver is
 * built, then the lower half will also be needed.  There is no need for
 * the early serial initialization in this case.
 */

#if !defined(USE_SERIALDRIVER) && defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/* If GPIO IRQ support is defined, then a set of GPIOs must all be included */

#if CONFIG_AVR32_GPIOIRQSETA == 0 && CONFIG_AVR32_GPIOIRQSETB == 0
#  undef CONFIG_AVR32_GPIOIRQ
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_CONFIG_H */
