/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_config.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CONFIG_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UARTs ********************************************************************/

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#ifdef CONFIG_ESP32S3_UART
#  define HAVE_UART_DEVICE 1
#endif

/* Serial Console ***********************************************************/

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn. n E {0,1}
 */

#undef CONSOLE_UART
#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_ESP32S3_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define CONSOLE_UART 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_ESP32S3_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  define CONSOLE_UART 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#endif

#ifdef CONFIG_ESP32S3_USBSERIAL
#  define HAVE_SERIAL_CONSOLE 1
#  define HAVE_UART_DEVICE 1
#endif

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CONFIG_H */
