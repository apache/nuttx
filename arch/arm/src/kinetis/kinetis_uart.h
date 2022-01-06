/****************************************************************************
 * arch/arm/src/kinetis/kinetis_uart.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H

#if defined(HAVE_UART_DEVICE) && defined(USE_SERIALDRIVER)

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Is DMA available on any (enabled) UART? */

#undef SERIAL_HAVE_DMA
#if defined(CONFIG_KINETIS_UART0_RXDMA) || defined(CONFIG_KINETIS_UART1_RXDMA) || \
    defined(CONFIG_KINETIS_UART2_RXDMA) || defined(CONFIG_KINETIS_UART3_RXDMA) || \
    defined(CONFIG_KINETIS_UART4_RXDMA) || defined(CONFIG_KINETIS_UART5_RXDMA)
#  define SERIAL_HAVE_DMA 1

/* Is DMA available on All (enabled) UART? */

#define SERIAL_HAVE_ALL_DMA 1
#  if (defined(CONFIG_KINETIS_UART0) && !defined(CONFIG_KINETIS_UART0_RXDMA)) || \
      (defined(CONFIG_KINETIS_UART1) && !defined(CONFIG_KINETIS_UART1_RXDMA)) || \
      (defined(CONFIG_KINETIS_UART2) && !defined(CONFIG_KINETIS_UART2_RXDMA)) || \
      (defined(CONFIG_KINETIS_UART3) && !defined(CONFIG_KINETIS_UART3_RXDMA)) || \
      (defined(CONFIG_KINETIS_UART4) && !defined(CONFIG_KINETIS_UART4_RXDMA)) || \
      (defined(CONFIG_KINETIS_UART5) && !defined(CONFIG_KINETIS_UART5_RXDMA))
#    undef SERIAL_HAVE_ALL_DMA
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
 *   Must be called periodically if any Kinetis UART is configured for DMA.
 *   The DMA callback is triggered for each fifo size/2 bytes, but this can
 *   result in some bytes being transferred but not collected if the incoming
 *   data is not a whole multiple of half the FIFO size.
 *
 *   May be safely called from either interrupt or thread context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
void kinetis_serial_dma_poll(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER) */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H */
