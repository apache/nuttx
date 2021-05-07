/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_config.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_CONFIG_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#undef HAVE_LPUART0
#undef HAVE_LPUART1
#undef HAVE_LPUART2

#ifdef CONFIG_S32K1XX_LPUART0
#  define HAVE_LPUART0 1
#endif
#ifdef CONFIG_S32K1XX_LPUART1
#  define HAVE_LPUART1 1
#endif
#ifdef CONFIG_S32K1XX_LPUART2
#  define HAVE_LPUART2 1
#endif

/* Check if we have a LPUART device */

#undef CONFIG_S32K1XX_HAVE_LPUART
#undef HAVE_LPUART_DEVICE

#if defined(HAVE_LPUART0) || defined(HAVE_LPUART1) || defined(HAVE_LPUART2)
#  define HAVE_LPUART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined.
 * It could be on any LPUARTn, n=0,1,2,3
 */

#undef HAVE_LPUART_CONSOLE

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE) && defined(HAVE_LPUART0)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART1_SERIAL_CONSOLE) && defined(HAVE_LPUART1)
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE) && defined(HAVE_LPUART2)
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#else
#  ifdef CONFIG_DEV_CONSOLE
#    warning "No valid CONFIG_[LP]LPUART[n]_SERIAL_CONSOLE Setting"
#  endif
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#endif

/* Check LPUART flow control (Not yet supported) */

# undef CONFIG_LPUART0_FLOWCONTROL
# undef CONFIG_LPUART1_FLOWCONTROL
# undef CONFIG_LPUART2_FLOWCONTROL

/* Ethernet controller configuration */

#ifndef CONFIG_S32K1XX_ENET_NRXBUFFERS
#  define CONFIG_S32K1XX_ENET_NRXBUFFERS 6
#endif

#ifndef CONFIG_S32K1XX_ENET_NTXBUFFERS
#  define CONFIG_S32K1XX_ENET_NTXBUFFERS 2
#endif

#ifndef CONFIG_S32K1XX_ENET_NETHIFS
#  define CONFIG_S32K1XX_ENET_NETHIFS 1
#endif

#define S32K1XX_ENET_HAS_DBSWAP 1

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_CONFIG_H */
