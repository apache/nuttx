/****************************************************************************
 * arch/arm/src/imxrt/imxrt_config.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_CONFIG_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#undef HAVE_LPUART1
#undef HAVE_LPUART2
#undef HAVE_LPUART3
#undef HAVE_LPUART4
#undef HAVE_LPUART5
#undef HAVE_LPUART6
#undef HAVE_LPUART7
#undef HAVE_LPUART8

#ifdef CONFIG_IMXRT_LPUART1
#  define HAVE_LPUART1 1
#endif
#ifdef CONFIG_IMXRT_LPUART2
#  define HAVE_LPUART2 1
#endif
#ifdef CONFIG_IMXRT_LPUART3
#  define HAVE_LPUART3 1
#endif
#ifdef CONFIG_IMXRT_LPUART4
#  define HAVE_LPUART4 1
#endif
#ifdef CONFIG_IMXRT_LPUART5
#  define HAVE_LPUART5 1
#endif
#ifdef CONFIG_IMXRT_LPUART6
#  define HAVE_LPUART6 1
#endif
#ifdef CONFIG_IMXRT_LPUART7
#  define HAVE_LPUART7 1
#endif
#ifdef CONFIG_IMXRT_LPUART8
#  define HAVE_LPUART8 1
#endif

/* Check if we have a LPUART device */

#undef CONFIG_IMXRT_HAVE_LPUART
#undef HAVE_LPUART_DEVICE

#if defined(HAVE_LPUART1) || defined(HAVE_LPUART2) || defined(HAVE_LPUART3) || \
    defined(HAVE_LPUART4) || defined(HAVE_LPUART5) || defined(HAVE_LPUART6) || \
    defined(HAVE_LPUART7) || defined(HAVE_LPUART8)
#  define HAVE_LPUART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined.
 * It could be on any LPUARTn, n=1,2,3,4,5,6,7,8
 */

#undef HAVE_LPUART_CONSOLE

#if defined(CONFIG_LPUART1_SERIAL_CONSOLE) && defined(HAVE_LPUART1)
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef CONFIG_LPUART4_SERIAL_CONSOLE
#  undef CONFIG_LPUART5_SERIAL_CONSOLE
#  undef CONFIG_LPUART6_SERIAL_CONSOLE
#  undef CONFIG_LPUART7_SERIAL_CONSOLE
#  undef CONFIG_LPUART8_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE) && defined(HAVE_LPUART2)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef CONFIG_LPUART4_SERIAL_CONSOLE
#  undef CONFIG_LPUART5_SERIAL_CONSOLE
#  undef CONFIG_LPUART6_SERIAL_CONSOLE
#  undef CONFIG_LPUART7_SERIAL_CONSOLE
#  undef CONFIG_LPUART8_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART3_SERIAL_CONSOLE) && defined(HAVE_LPUART3)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART4_SERIAL_CONSOLE
#  undef CONFIG_LPUART5_SERIAL_CONSOLE
#  undef CONFIG_LPUART6_SERIAL_CONSOLE
#  undef CONFIG_LPUART7_SERIAL_CONSOLE
#  undef CONFIG_LPUART8_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART4_SERIAL_CONSOLE) && defined(HAVE_LPUART4)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef CONFIG_LPUART5_SERIAL_CONSOLE
#  undef CONFIG_LPUART6_SERIAL_CONSOLE
#  undef CONFIG_LPUART7_SERIAL_CONSOLE
#  undef CONFIG_LPUART8_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART5_SERIAL_CONSOLE) && defined(HAVE_LPUART5)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef CONFIG_LPUART4_SERIAL_CONSOLE
#  undef CONFIG_LPUART6_SERIAL_CONSOLE
#  undef CONFIG_LPUART7_SERIAL_CONSOLE
#  undef CONFIG_LPUART8_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART6_SERIAL_CONSOLE) && defined(HAVE_LPUART6)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef CONFIG_LPUART4_SERIAL_CONSOLE
#  undef CONFIG_LPUART5_SERIAL_CONSOLE
#  undef CONFIG_LPUART7_SERIAL_CONSOLE
#  undef CONFIG_LPUART8_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART7_SERIAL_CONSOLE) && defined(HAVE_LPUART7)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef CONFIG_LPUART4_SERIAL_CONSOLE
#  undef CONFIG_LPUART5_SERIAL_CONSOLE
#  undef CONFIG_LPUART6_SERIAL_CONSOLE
#  undef CONFIG_LPUART8_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART8_SERIAL_CONSOLE) && defined(HAVE_LPUART8)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef CONFIG_LPUART4_SERIAL_CONSOLE
#  undef CONFIG_LPUART5_SERIAL_CONSOLE
#  undef CONFIG_LPUART6_SERIAL_CONSOLE
#  undef CONFIG_LPUART7_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#else
#  ifdef CONFIG_DEV_CONSOLE
#    warning "No valid CONFIG_[LP]LPUART[n]_SERIAL_CONSOLE Setting"
#  endif
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef CONFIG_LPUART4_SERIAL_CONSOLE
#  undef CONFIG_LPUART5_SERIAL_CONSOLE
#  undef CONFIG_LPUART6_SERIAL_CONSOLE
#  undef CONFIG_LPUART7_SERIAL_CONSOLE
#  undef CONFIG_LPUART8_SERIAL_CONSOLE
#endif

/* Check LPUART flow control (Not yet supported) */

# undef CONFIG_LPUART1_FLOWCONTROL
# undef CONFIG_LPUART2_FLOWCONTROL
# undef CONFIG_LPUART3_FLOWCONTROL
# undef CONFIG_LPUART4_FLOWCONTROL
# undef CONFIG_LPUART5_FLOWCONTROL
# undef CONFIG_LPUART6_FLOWCONTROL
# undef CONFIG_LPUART7_FLOWCONTROL
# undef CONFIG_LPUART8_FLOWCONTROL

/* Ethernet controller configuration */

#ifndef CONFIG_IMXRT_ENET_NRXBUFFERS
#  define CONFIG_IMXRT_ENET_NRXBUFFERS 6
#endif

#ifndef CONFIG_IMXRT_ENET_NTXBUFFERS
#  define CONFIG_IMXRT_ENET_NTXBUFFERS 2
#endif

#ifndef CONFIG_IMXRT_ENET_NETHIFS
#  define CONFIG_IMXRT_ENET_NETHIFS 1
#endif

#define IMXRT_ENET_HAS_DBSWAP 1

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_CONFIG_H */
