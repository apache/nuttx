/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_serial.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_S32K3XX_SERIAL_H
#define __ARCH_ARM_SRC_S32K3XX_S32K3XX_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "s32k3xx_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_S32K3XX_LPUART0)  || defined(CONFIG_S32K3XX_LPUART1)  || \
    defined(CONFIG_S32K3XX_LPUART2)  || defined(CONFIG_S32K3XX_LPUART3)  || \
    defined(CONFIG_S32K3XX_LPUART3)  || defined(CONFIG_S32K3XX_LPUART5)  || \
    defined(CONFIG_S32K3XX_LPUART6)  || defined(CONFIG_S32K3XX_LPUART7)  || \
    defined(CONFIG_S32K3XX_LPUART8)  || defined(CONFIG_S32K3XX_LPUART9)  || \
    defined(CONFIG_S32K3XX_LPUART10) || defined(CONFIG_S32K3XX_LPUART11) || \
    defined(CONFIG_S32K3XX_LPUART12) || defined(CONFIG_S32K3XX_LPUART13) || \
    defined(CONFIG_S32K3XX_LPUART14) || defined(CONFIG_S32K3XX_LPUART15)
#  define HAVE_UART 1
#endif

/* Assume DMA is not used on the console UART */

#undef SERIAL_HAVE_CONSOLE_RXDMA
#undef SERIAL_HAVE_CONSOLE_TXDMA

#if !defined(HAVE_UART) || !defined(CONFIG_ARCH_DMA)
#  undef CONFIG_LPUART0_RXDMA
#  undef CONFIG_LPUART0_TXDMA
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
#  undef CONFIG_LPUART9_RXDMA
#  undef CONFIG_LPUART9_TXDMA
#  undef CONFIG_LPUART10_RXDMA
#  undef CONFIG_LPUART10_TXDMA
#  undef CONFIG_LPUART11_RXDMA
#  undef CONFIG_LPUART11_TXDMA
#  undef CONFIG_LPUART12_RXDMA
#  undef CONFIG_LPUART12_TXDMA
#  undef CONFIG_LPUART13_RXDMA
#  undef CONFIG_LPUART13_TXDMA
#  undef CONFIG_LPUART14_RXDMA
#  undef CONFIG_LPUART14_TXDMA
#  undef CONFIG_LPUART15_RXDMA
#  undef CONFIG_LPUART15_TXDMA
#endif

/* Disable the DMA configuration on all unused LPUARTs */

#ifndef CONFIG_S32K3XX_LPUART1
#  undef CONFIG_LPUART1_RXDMA
#  undef CONFIG_LPUART1_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART2
#  undef CONFIG_LPUART2_RXDMA
#  undef CONFIG_LPUART2_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART3
#  undef CONFIG_LPUART3_RXDMA
#  undef CONFIG_LPUART3_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART4
#  undef CONFIG_LPUART4_RXDMA
#  undef CONFIG_LPUART4_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART5
#  undef CONFIG_LPUART5_RXDMA
#  undef CONFIG_LPUART5_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART6
#  undef CONFIG_LPUART6_RXDMA
#  undef CONFIG_LPUART6_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART7
#  undef CONFIG_LPUART7_RXDMA
#  undef CONFIG_LPUART7_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART8
#  undef CONFIG_LPUART8_RXDMA
#  undef CONFIG_LPUART8_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART9
#  undef CONFIG_LPUART9_RXDMA
#  undef CONFIG_LPUART9_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART10
#  undef CONFIG_LPUART10_RXDMA
#  undef CONFIG_LPUART10_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART11
#  undef CONFIG_LPUART11_RXDMA
#  undef CONFIG_LPUART11_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART12
#  undef CONFIG_LPUART12_RXDMA
#  undef CONFIG_LPUART12_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART13
#  undef CONFIG_LPUART13_RXDMA
#  undef CONFIG_LPUART13_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART14
#  undef CONFIG_LPUART14_RXDMA
#  undef CONFIG_LPUART14_TXDMA
#endif

#ifndef CONFIG_S32K3XX_LPUART15
#  undef CONFIG_LPUART15_RXDMA
#  undef CONFIG_LPUART15_TXDMA
#endif

/* Is RX DMA available on any (enabled) LPUART? */

#undef SERIAL_HAVE_RXDMA
#if defined(CONFIG_LPUART0_RXDMA)  || defined(CONFIG_LPUART1_RXDMA)  || \
    defined(CONFIG_LPUART2_RXDMA)  || defined(CONFIG_LPUART3_RXDMA)  || \
    defined(CONFIG_LPUART3_RXDMA)  || defined(CONFIG_LPUART5_RXDMA)  || \
    defined(CONFIG_LPUART6_RXDMA)  || defined(CONFIG_LPUART7_RXDMA)  || \
    defined(CONFIG_LPUART8_RXDMA)  || defined(CONFIG_LPUART9_RXDMA)  || \
    defined(CONFIG_LPUART10_RXDMA) || defined(CONFIG_LPUART11_RXDMA) || \
    defined(CONFIG_LPUART12_RXDMA) || defined(CONFIG_LPUART13_RXDMA) || \
    defined(CONFIG_LPUART14_RXDMA) || defined(CONFIG_LPUART15_RXDMA)
#  define SERIAL_HAVE_RXDMA 1
#endif

/* Is TX DMA available on any (enabled) LPUART? */
#undef SERIAL_HAVE_TXDMA
#if defined(CONFIG_LPUART0_TXDMA)  || defined(CONFIG_LPUART1_TXDMA)  || \
    defined(CONFIG_LPUART2_TXDMA)  || defined(CONFIG_LPUART3_TXDMA)  || \
    defined(CONFIG_LPUART3_TXDMA)  || defined(CONFIG_LPUART5_TXDMA)  || \
    defined(CONFIG_LPUART6_TXDMA)  || defined(CONFIG_LPUART7_TXDMA)  || \
    defined(CONFIG_LPUART8_TXDMA)  || defined(CONFIG_LPUART9_TXDMA)  || \
    defined(CONFIG_LPUART10_TXDMA) || defined(CONFIG_LPUART11_TXDMA) || \
    defined(CONFIG_LPUART12_TXDMA) || defined(CONFIG_LPUART13_TXDMA) || \
    defined(CONFIG_LPUART14_TXDMA) || defined(CONFIG_LPUART15_TXDMA)
#  define SERIAL_HAVE_TXDMA 1
#endif

/* Is RX DMA used on all (enabled) LPUARTs */

#define SERIAL_HAVE_ONLY_RXDMA 1
#if defined(CONFIG_S32K3XX_LPUART0) && !defined(CONFIG_LPUART0_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART1) && !defined(CONFIG_LPUART1_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART2) && !defined(CONFIG_LPUART2_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART3) && !defined(CONFIG_LPUART3_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART4) && !defined(CONFIG_LPUART4_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART5) && !defined(CONFIG_LPUART5_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART6) && !defined(CONFIG_LPUART6_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART7) && !defined(CONFIG_LPUART7_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART8) && !defined(CONFIG_LPUART8_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART9) && !defined(CONFIG_LPUART9_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART10) && !defined(CONFIG_LPUART10_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART11) && !defined(CONFIG_LPUART11_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART12) && !defined(CONFIG_LPUART12_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART13) && !defined(CONFIG_LPUART13_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART14) && !defined(CONFIG_LPUART14_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#elif defined(CONFIG_S32K3XX_LPUART15) && !defined(CONFIG_LPUART15_RXDMA)
#  undef SERIAL_HAVE_ONLY_RXDMA
#endif

/* Is TX DMA used on all (enabled) LPUARTs */

#define SERIAL_HAVE_ONLY_TXDMA 1
#if defined(CONFIG_S32K3XX_LPUART0) && !defined(CONFIG_LPUART0_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART1) && !defined(CONFIG_LPUART1_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART2) && !defined(CONFIG_LPUART2_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART3) && !defined(CONFIG_LPUART3_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART4) && !defined(CONFIG_LPUART4_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART5) && !defined(CONFIG_LPUART5_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART6) && !defined(CONFIG_LPUART6_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART7) && !defined(CONFIG_LPUART7_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART8) && !defined(CONFIG_LPUART8_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART9) && !defined(CONFIG_LPUART9_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART10) && !defined(CONFIG_LPUART10_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART11) && !defined(CONFIG_LPUART11_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART12) && !defined(CONFIG_LPUART12_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART13) && !defined(CONFIG_LPUART13_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART14) && !defined(CONFIG_LPUART14_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#elif defined(CONFIG_S32K3XX_LPUART15) && !defined(CONFIG_LPUART15_TXDMA)
#  undef SERIAL_HAVE_ONLY_TXDMA
#endif

#undef SERIAL_HAVE_ONLY_DMA
#if defined(SERIAL_HAVE_ONLY_RXDMA) && defined(SERIAL_HAVE_ONLY_TXDMA)
#define SERIAL_HAVE_ONLY_DMA
#endif

/* Verify that DMA has been enabled and the DMA channel has been defined.
 */

#  if defined(SERIAL_HAVE_TXDMA) || defined(SERIAL_HAVE_RXDMA)
#    ifndef CONFIG_S32K3XX_EDMA
#      error IMXRT LPUART receive or transmit DMA requires CONFIG_S32K3XX_EDMA
#    endif
#  endif

/* Verify that there are not 2 devices enabled on one DMAMUX input */

#if (defined(CONFIG_LPUART0_RXDMA) && defined(CONFIG_LPUART8_RXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART0_RXDMA and CONFIG_LPUART8_RXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART0_TXDMA) && defined(CONFIG_LPUART8_TXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART0_TXDMA and CONFIG_LPUART8_TXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART9_RXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART1_RXDMA and CONFIG_LPUART9_RXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART1_TXDMA) && defined(CONFIG_LPUART9_TXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART1_TXDMA and CONFIG_LPUART9_TXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART2_RXDMA) && defined(CONFIG_LPUART10_RXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART2_RXDMA and CONFIG_LPUART10_RXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART2_TXDMA) && defined(CONFIG_LPUART10_TXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART2_TXDMA and CONFIG_LPUART10_TXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART3_RXDMA) && defined(CONFIG_LPUART11_RXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART3_RXDMA and CONFIG_LPUART11_RXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART3_TXDMA) && defined(CONFIG_LPUART11_TXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART3_TXDMA and CONFIG_LPUART11_TXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART4_RXDMA) && defined(CONFIG_LPUART12_RXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART4_RXDMA and CONFIG_LPUART12_RXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART4_TXDMA) && defined(CONFIG_LPUART12_TXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART4_TXDMA and CONFIG_LPUART12_TXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART5_RXDMA) && defined(CONFIG_LPUART13_RXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART5_RXDMA and CONFIG_LPUART13_RXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART5_TXDMA) && defined(CONFIG_LPUART13_TXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART5_TXDMA and CONFIG_LPUART13_TXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART6_RXDMA) && defined(CONFIG_LPUART14_RXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART6_RXDMA and CONFIG_LPUART14_RXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART6_TXDMA) && defined(CONFIG_LPUART14_TXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART6_TXDMA and CONFIG_LPUART14_TXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART7_RXDMA) && defined(CONFIG_LPUART15_RXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART7_RXDMA and CONFIG_LPUART15_RXDMA can not be set at the same time"
#endif
#if (defined(CONFIG_LPUART7_TXDMA) && defined(CONFIG_LPUART15_TXDMA))
#  error "DMA MUX conflict:CONFIG_LPUART7_TXDMA and CONFIG_LPUART15_TXDMA can not be set at the same time"
#endif
#if defined(SERIAL_HAVE_RXDMA)
/* Currently RS-485 support cannot be enabled when RXDMA is in use due to
 * lack of testing.
 */

#  if (defined(CONFIG_LPUART0_RXDMA)  && defined(CONFIG_LPUART0_RS485))  || \
      (defined(CONFIG_LPUART1_RXDMA)  && defined(CONFIG_LPUART1_RS485))  || \
      (defined(CONFIG_LPUART2_RXDMA)  && defined(CONFIG_LPUART2_RS485))  || \
      (defined(CONFIG_LPUART3_RXDMA)  && defined(CONFIG_LPUART3_RS485))  || \
      (defined(CONFIG_LPUART4_RXDMA)  && defined(CONFIG_LPUART4_RS485))  || \
      (defined(CONFIG_LPUART5_RXDMA)  && defined(CONFIG_LPUART5_RS485))  || \
      (defined(CONFIG_LPUART6_RXDMA)  && defined(CONFIG_LPUART6_RS485))  || \
      (defined(CONFIG_LPUART7_RXDMA)  && defined(CONFIG_LPUART7_RS485))  || \
      (defined(CONFIG_LPUART8_RXDMA)  && defined(CONFIG_LPUART8_RS485))  || \
      (defined(CONFIG_LPUART9_RXDMA)  && defined(CONFIG_LPUART9_RS485))  || \
      (defined(CONFIG_LPUART10_RXDMA) && defined(CONFIG_LPUART10_RS485)) || \
      (defined(CONFIG_LPUART11_RXDMA) && defined(CONFIG_LPUART11_RS485)) || \
      (defined(CONFIG_LPUART12_RXDMA) && defined(CONFIG_LPUART12_RS485)) || \
      (defined(CONFIG_LPUART13_RXDMA) && defined(CONFIG_LPUART13_RS485)) || \
      (defined(CONFIG_LPUART14_RXDMA) && defined(CONFIG_LPUART14_RS485)) || \
      (defined(CONFIG_LPUART15_RXDMA) && defined(CONFIG_LPUART15_RS485))
#    error "RXDMA and RS-485 cannot be enabled at the same time for the same LPUART"
#  endif
#endif /* SERIAL_HAVE_RXDMA */

/* Currently RS-485 support cannot be enabled when TXDMA is in use due to
 * lack of testing.
 */

#  if (defined(CONFIG_LPUART0_TXDMA)  && defined(CONFIG_LPUART0_RS485))  || \
      (defined(CONFIG_LPUART1_TXDMA)  && defined(CONFIG_LPUART1_RS485))  || \
      (defined(CONFIG_LPUART2_TXDMA)  && defined(CONFIG_LPUART2_RS485))  || \
      (defined(CONFIG_LPUART3_TXDMA)  && defined(CONFIG_LPUART3_RS485))  || \
      (defined(CONFIG_LPUART4_TXDMA)  && defined(CONFIG_LPUART4_RS485))  || \
      (defined(CONFIG_LPUART5_TXDMA)  && defined(CONFIG_LPUART5_RS485))  || \
      (defined(CONFIG_LPUART6_TXDMA)  && defined(CONFIG_LPUART6_RS485))  || \
      (defined(CONFIG_LPUART7_TXDMA)  && defined(CONFIG_LPUART7_RS485))  || \
      (defined(CONFIG_LPUART8_TXDMA)  && defined(CONFIG_LPUART8_RS485))  || \
      (defined(CONFIG_LPUART9_TXDMA)  && defined(CONFIG_LPUART9_RS485))  || \
      (defined(CONFIG_LPUART10_TXDMA) && defined(CONFIG_LPUART10_RS485)) || \
      (defined(CONFIG_LPUART11_TXDMA) && defined(CONFIG_LPUART11_RS485)) || \
      (defined(CONFIG_LPUART12_TXDMA) && defined(CONFIG_LPUART12_RS485)) || \
      (defined(CONFIG_LPUART13_TXDMA) && defined(CONFIG_LPUART13_RS485)) || \
      (defined(CONFIG_LPUART14_TXDMA) && defined(CONFIG_LPUART14_RS485)) || \
      (defined(CONFIG_LPUART15_TXDMA) && defined(CONFIG_LPUART15_RS485))
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
 * Name: s32k3xx_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void s32k3xx_earlyserialinit(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K3XX_S32K3XX_SERIAL_H */
