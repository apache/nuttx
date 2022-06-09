/****************************************************************************
 * arch/arm/src/imxrt/imxrt_serial.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_SERIAL_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "imxrt_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_IMXRT_LPUART1) || defined(CONFIG_IMXRT_LPUART2) || \
    defined(CONFIG_IMXRT_LPUART3) || defined(CONFIG_IMXRT_LPUART4) || \
    defined(CONFIG_IMXRT_LPUART5) || defined(CONFIG_IMXRT_LPUART6) || \
    defined(CONFIG_IMXRT_LPUART7) || defined(CONFIG_IMXRT_LPUART8)
#  define HAVE_UART 1
#endif

/* Assume DMA is not used on the console UART */

#undef SERIAL_HAVE_CONSOLE_RXDMA
#undef SERIAL_HAVE_CONSOLE_TXDMA

#if !defined(HAVE_UART) || !defined(CONFIG_ARCH_DMA)
#  undef CONFIG_LPUART1_RXDMA
#  undef CONFIG_LPUART1_TXDMA
#  undef CONFIG_LPUART2_RXDMA
#  undef CONFIG_LPUART2_TXDMA
#  undef CONFIG_LPUART3_RXDMA
#  undef CONFIG_LPUART3_TXDMA
#  undef CONFIG_LPUART4_RXDMA
#  undef CONFIG_LPUART4_TXDMA
#  undef CONFIG_LPUART5_RXDMA
#  undef CONFIG_LPUART5_TXDMA
#  undef CONFIG_LPUART6_RXDMA
#  undef CONFIG_LPUART6_TXDMA
#  undef CONFIG_LPUART7_RXDMA
#  undef CONFIG_LPUART7_TXDMA
#  undef CONFIG_LPUART8_RXDMA
#  undef CONFIG_LPUART8_TXDMA
#endif

/* Disable the DMA configuration on all unused LPUARTs */

#ifndef CONFIG_IMXRT_LPUART1
#  undef CONFIG_LPUART1_RXDMA
#  undef CONFIG_LPUART1_TXDMA
#endif

#ifndef CONFIG_IMXRT_LPUART2
#  undef CONFIG_LPUART2_RXDMA
#  undef CONFIG_LPUART2_TXDMA
#endif

#ifndef CONFIG_IMXRT_LPUART3
#  undef CONFIG_LPUART3_RXDMA
#  undef CONFIG_LPUART3_TXDMA
#endif

#ifndef CONFIG_IMXRT_LPUART4
#  undef CONFIG_LPUART4_RXDMA
#  undef CONFIG_LPUART4_TXDMA
#endif

#ifndef CONFIG_IMXRT_LPUART5
#  undef CONFIG_LPUART5_RXDMA
#  undef CONFIG_LPUART5_TXDMA
#endif

#ifndef CONFIG_IMXRT_LPUART6
#  undef CONFIG_LPUART6_RXDMA
#  undef CONFIG_LPUART6_TXDMA
#endif

#ifndef CONFIG_IMXRT_LPUART7
#  undef CONFIG_LPUART7_RXDMA
#  undef CONFIG_LPUART7_TXDMA
#endif

#ifndef CONFIG_IMXRT_LPUART8
#  undef CONFIG_LPUART8_RXDMA
#  undef CONFIG_LPUART8_TXDMA
#endif

/* Is RX DMA available on any (enabled) LPUART? */

#undef SERIAL_HAVE_RXDMA
#if defined(CONFIG_LPUART1_RXDMA) || defined(CONFIG_LPUART2_RXDMA) || \
    defined(CONFIG_LPUART3_RXDMA) || defined(CONFIG_LPUART4_RXDMA) || \
    defined(CONFIG_LPUART5_RXDMA) || defined(CONFIG_LPUART6_RXDMA) || \
    defined(CONFIG_LPUART7_RXDMA) || defined(CONFIG_LPUART8_RXDMA)
#  define SERIAL_HAVE_RXDMA 1
#endif

/* Is TX DMA available on any (enabled) LPUART? */
#undef SERIAL_HAVE_TXDMA
#if defined(CONFIG_LPUART1_TXDMA) || defined(CONFIG_LPUART2_TXDMA) || \
    defined(CONFIG_LPUART3_TXDMA) || defined(CONFIG_LPUART4_TXDMA) || \
    defined(CONFIG_LPUART5_TXDMA) || defined(CONFIG_LPUART6_TXDMA) || \
    defined(CONFIG_LPUART7_TXDMA) || defined(CONFIG_LPUART8_TXDMA)
#  define SERIAL_HAVE_TXDMA 1
#endif

/* Is RX DMA used on all (enabled) LPUARTs */

#define SERIAL_HAVE_ONLY_RXDMA 1
#if defined(CONFIG_IMXRT_LPUART1) && !defined(CONFIG_LPUART1_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_IMXRT_LPUART2) && !defined(CONFIG_LPUART2_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_IMXRT_LPUART3) && !defined(CONFIG_LPUART3_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_IMXRT_LPUART4) && !defined(CONFIG_LPUART4_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_IMXRT_LPUART5) && !defined(CONFIG_LPUART5_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_IMXRT_LPUART6) && !defined(CONFIG_LPUART6_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_IMXRT_LPUART7) && !defined(CONFIG_LPUART7_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(CONFIG_LPUART8_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#endif

/* Is TX DMA used on all (enabled) LPUARTs */

#define SERIAL_HAVE_ONLY_TXDMA 1
#if defined(CONFIG_IMXRT_LPUART1) && !defined(CONFIG_LPUART1_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_IMXRT_LPUART2) && !defined(CONFIG_LPUART2_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_IMXRT_LPUART3) && !defined(CONFIG_LPUART3_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_IMXRT_LPUART4) && !defined(CONFIG_LPUART4_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_IMXRT_LPUART5) && !defined(CONFIG_LPUART5_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_IMXRT_LPUART6) && !defined(CONFIG_LPUART6_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_IMXRT_LPUART7) && !defined(CONFIG_LPUART7_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_IMXRT_LPUART8) && !defined(CONFIG_LPUART8_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#endif

#undef SERIAL_HAVE_ONLY_DMA
#if defined(SERIAL_HAVE_ONLY_RXDMA) && defined(SERIAL_HAVE_ONLY_TXDMA)
#define SERIAL_HAVE_ONLY_DMA
#endif

/* Verify that DMA has been enabled and the DMA channel has been defined.
 */

#  if defined(SERIAL_HAVE_TXDMA) || defined(SERIAL_HAVE_RXDMA)
#    ifndef CONFIG_IMXRT_EDMA
#      error IMXRT LPUART receive or transmit DMA requires CONFIG_IMXRT_EDMA
#    endif
#  endif

#if defined(SERIAL_HAVE_RXDMA)
/* Currently RS-485 support cannot be enabled when RXDMA is in use due to
 * lack of testing.
 */

#  if (defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_RS485)) || \
      (defined(CONFIG_LPUART2_RXDMA) && defined(CONFIG_LPUART2_RS485)) || \
      (defined(CONFIG_LPUART3_RXDMA) && defined(CONFIG_LPUART3_RS485)) || \
      (defined(CONFIG_LPUART4_RXDMA) && defined(CONFIG_LPUART4_RS485)) || \
      (defined(CONFIG_LPUART5_RXDMA) && defined(CONFIG_LPUART5_RS485)) || \
      (defined(CONFIG_LPUART6_RXDMA) && defined(CONFIG_LPUART6_RS485)) || \
      (defined(CONFIG_LPUART7_RXDMA) && defined(CONFIG_LPUART7_RS485)) || \
      (defined(CONFIG_LPUART8_RXDMA) && defined(CONFIG_LPUART8_RS485))
#    error "RXDMA and RS-485 cannot be enabled at the same time for the same LPUART"
#  endif
#endif /* SERIAL_HAVE_RXDMA */

/* Currently RS-485 support cannot be enabled when TXDMA is in use due to
 * lack of testing.
 */

#if (defined(CONFIG_LPUART1_TXDMA) && defined(CONFIG_LPUART1_RS485)) || \
    (defined(CONFIG_LPUART2_TXDMA) && defined(CONFIG_LPUART2_RS485)) || \
    (defined(CONFIG_LPUART3_TXDMA) && defined(CONFIG_LPUART3_RS485)) || \
    (defined(CONFIG_LPUART4_TXDMA) && defined(CONFIG_LPUART4_RS485)) || \
    (defined(CONFIG_LPUART5_TXDMA) && defined(CONFIG_LPUART5_RS485)) || \
    (defined(CONFIG_LPUART6_TXDMA) && defined(CONFIG_LPUART6_RS485)) || \
    (defined(CONFIG_LPUART7_TXDMA) && defined(CONFIG_LPUART7_RS485)) || \
    (defined(CONFIG_LPUART8_TXDMA) && defined(CONFIG_LPUART8_RS485))
#  error "TXDMA and RS-485 cannot be enabled at the same time for the same LPUART"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void imxrt_earlyserialinit(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_SERIAL_H */
