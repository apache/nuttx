/****************************************************************************
 * arch/avr/src/at90usb/at90usb_config.h
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

#ifndef __ARCH_AVR_SRC_AT90USB_AT90USB_CONFIG_H
#define __ARCH_AVR_SRC_AT90USB_AT90USB_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USARTs *******************************************************************/

#undef HAVE_USART_DEVICE
#if defined(CONFIG_AVR_USART1)
#  define HAVE_USART_DEVICE 1
#endif

/* Is there a serial console?  There should be at most one defined.  It
 * could be on any USARTn (but n=1 only for the AT90USB).
 */

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_AVR_USART1)
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_USART1_SERIAL_CONSOLE
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
#  elif defined(HAVE_USART_DEVICE)
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_AVR_SRC_AT90USB_AT90USB_CONFIG_H */
