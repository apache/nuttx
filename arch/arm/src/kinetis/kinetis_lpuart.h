/****************************************************************************
 * arch/arm/src/kinetis/kinetis_lpuart.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_LPUART_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_LPUART_H

#if defined(HAVE_UART_DEVICE) && defined(USE_SERIALDRIVER)

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Is DMA available on any (enabled) LPUART? */

#undef LPSERIAL_HAVE_DMA
#if defined(CONFIG_KINETIS_LPUART0_RXDMA) || defined(CONFIG_KINETIS_LPUART1_RXDMA) || \
    defined(CONFIG_KINETIS_LPUART2_RXDMA) || defined(CONFIG_KINETIS_LPUART3_RXDMA) || \
    defined(CONFIG_KINETIS_LPUART4_RXDMA)
#  define LPSERIAL_HAVE_DMA 1

/* Is DMA available on All (enabled) LPUART? */

#define LPSERIAL_HAVE_ALL_DMA 1
#  if (defined(CONFIG_KINETIS_LPUART0) && !defined(CONFIG_KINETIS_LPUART0_RXDMA)) || \
      (defined(CONFIG_KINETIS_LPUART1) && !defined(CONFIG_KINETIS_LPUART1_RXDMA)) || \
      (defined(CONFIG_KINETIS_LPUART2) && !defined(CONFIG_KINETIS_LPUART2_RXDMA)) || \
      (defined(CONFIG_KINETIS_LPUART3) && !defined(CONFIG_KINETIS_LPUART3_RXDMA)) || \
      (defined(CONFIG_KINETIS_LPUART4) && !defined(CONFIG_KINETIS_LPUART4_RXDMA))
#    undef LPSERIAL_HAVE_ALL_DMA
#  endif
#endif

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
 * Name: kinetis_serial_dma_poll
 *
 * Description:
 *   Must be called periodically if any Kinetis LPUART is configured for DMA.
 *   The DMA callback is triggered for each fifo size/2 bytes, but this can
 *   result in some bytes being transferred but not collected if the incoming
 *   data is not a whole multiple of half the FIFO size.
 *
 *   May be safely called from either interrupt or thread context.
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
void kinetis_lpserial_dma_poll(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER) */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H */
