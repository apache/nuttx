/****************************************************************************
 * boards/arm/samv7/common/include/board_uart_rxdma_poll.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_UART_RXDMA_POLL_H
#define __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_UART_RXDMA_POLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

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
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: board_uart_rxdma_poll_stop
 *
 * Description:
 *   This function starts the timer polling UART.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_uart_rxdma_poll_start(void);

/****************************************************************************
 * Name: board_uart_rxdma_poll_stop
 *
 * Description:
 *   This function stops the timer polling UART. No resources are freed,
 *   time is just stopped and can be started again.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_uart_rxdma_poll_stop(void);

/****************************************************************************
 * Name: board_uart_rxdma_poll_init
 *
 * Description:
 *   This function initializes and starts the timer that is polling
 *   UART/USART peripherals with RX DMA enabled.
 *
 * Input Parameters:
 *   channel   - Channel number (0-11, see TC_CHANx)
 *   frequency - Timer frequency
 *
 * Returned Value:
 *   0 on success, negative value (errno) on ERROR.
 *
 ****************************************************************************/

int board_uart_rxdma_poll_init(int channel, uint32_t frequency);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_UART_RXDMA_POLL_H */
