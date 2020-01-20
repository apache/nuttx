/****************************************************************************
 * arch/arm/src/tiva/tiva_hciuart.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HCIUART_H
#define __ARCH_ARM_SRC_TIVA_HCIUART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum hciuart_devno_e
{
#ifdef CONFIG_TIVA_UART0_HCIUART
  HCIUART0,   /* HCI UART on TIVA UART0 */
#endif
#ifdef CONFIG_TIVA_UART1_HCIUART
  HCIUART1,   /* HCI UART on TIVA UART1 */
#endif
#ifdef CONFIG_TIVA_UART2_HCIUART
  HCIUART2,   /* HCI UART on TIVA UART2 */
#endif
#ifdef CONFIG_TIVA_UART3_HCIUART
  HCIUART3,   /* HCI UART on TIVA UART3 */
#endif
#ifdef CONFIG_TIVA_UART4_HCIUART
  HCIUART4,   /* HCI UART on TIVA UART4 */
#endif
#ifdef CONFIG_TIVA_UART5_HCIUART
  HCIUART5,   /* HCI UART on TIVA UART5 */
#endif
#ifdef CONFIG_TIVA_UART6_HCIUART
  HCIUART6,   /* HCI UART on TIVA UART6 */
#endif
#ifdef CONFIG_TIVA_UART7_HCIUART
  HCIUART7    /* HCI UART on TIVA UART7 */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hciuart_instantiate
 *
 * Description:
 *   Obtain an instance of the HCI UART interface for the specified HCI UART
 *   This assumes that hciuart_initialize was called previously.
 *
 * Input Parameters:
 *   uart - Identifies the HCI UART to be configured
 *
 * Returned Value:
 *   On success, a reference to the HCI UART lower driver for the associated
 *   UART
 *
 ****************************************************************************/

const struct btuart_lowerhalf_s *
hciuart_instantiate(enum hciuart_devno_e uart);

/****************************************************************************
 * Name: hciuart_initialize
 *
 * Description:
 *   Performs the low-level, one-time USART initialization.  This must be
 *   called before hciuart_instantiate.
 *
 ****************************************************************************/

void hciuart_initialize(void);

/****************************************************************************
 * Name: tiva_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_HCIUART_RXDMA
void tiva_serial_dma_poll(void);
#endif

#endif /* __ARCH_ARM_SRC_TIVA_HCIUART_H */
