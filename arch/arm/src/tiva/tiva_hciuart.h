/****************************************************************************
 * arch/arm/src/tiva/tiva_hciuart.h
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_HCIUART_H
#define __ARCH_ARM_SRC_TIVA_TIVA_HCIUART_H

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

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_HCIUART_H */
