/************************************************************************************
 * arch/arm/src/kinetis/kinetis_uart.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: David Sidrane <david_s5@nscdg.com>
 *            Jan Okle <jan@leitwert.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H

#if defined(HAVE_UART_DEVICE) && defined(USE_SERIALDRIVER)

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/


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

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: kinetis_serial_dma_poll
 *
 * Description:
 *   Must be called periodically if any Kinetis UART is configured for DMA.  The DMA
 *   callback is triggered for each fifo size/2 bytes, but this can result in some
 *   bytes being transferred but not collected if the incoming data is not a whole
 *   multiple of half the FIFO size.
 *
 *   May be safely called from either interrupt or thread context.
 *
 ************************************************************************************/

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
