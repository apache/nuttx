/****************************************************************************
 * boards/arm/samv7/common/src/sam_uart_rxdma_poll.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sam_tc.h"
#include "sam_serial.h"

#include "board_uart_rxdma_poll.h"

#ifdef CONFIG_SAMV7_UART_RXDMA_POLL

static TC_HANDLE poll_tc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_handler
 *
 * Description:
 *   Timer interrupt callback.
 *
 * Input Parameters:
 *   tch - The handle that represents the timer state
 *   arg - An opaque argument provided when the interrupt was registered
 *   sr  - The value of the timer interrupt status register at the time
 *         that the interrupt occurred.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void timer_handler(TC_HANDLE tch, void *arg, uint32_t sr)
{
  sam_serial_dma_poll();
}

/****************************************************************************
 * Public Functions
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

void board_uart_rxdma_poll_start(void)
{
  sam_tc_start(poll_tc);
}

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

void board_uart_rxdma_poll_stop(void)
{
  sam_tc_stop(poll_tc);
}

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

int board_uart_rxdma_poll_init(int channel, uint32_t frequency)
{
  int ret;
  uint32_t div;
  uint32_t tcclks;
  uint32_t actual;
  uint32_t mode;
  uint32_t fdiv;
  uint32_t regval;

  if (channel < TC_CHAN0 || channel > TC_CHAN11)
    {
      syslog(LOG_ERR, "Invalid channel number: %d", channel);
      return -1;
    }

  ret = sam_tc_clockselect(frequency, &tcclks, &actual);
  if (ret < 0)
    {
      syslog(LOG_ERR, "sam_tc_divisor failed: %d", ret);
      return ret;
    }

  div = BOARD_MCK_FREQUENCY / actual;

  /* Set the timer/counter waveform mode the clock input selected by
   * sam_tc_clockselect()
   */

  mode = ((tcclks << TC_CMR_TCCLKS_SHIFT) |  /* Use selected TCCLKS value */
          TC_CMR_WAVSEL_UPRC |               /* UP mode w/ trigger on RC Compare */
          TC_CMR_WAVE);                      /* Wave mode */

  /* Now allocate and configure the channel */

  poll_tc = sam_tc_allocate(channel, mode);
  if (!poll_tc)
    {
      syslog(LOG_ERR, "Failed to allocate channel %d mode %08" PRIx32 "\n",
             channel, mode);
      return -EINVAL;
    }

  /* The divider returned by sam_tc_clockselect() is the reload value
   * that will achieve a 1Hz rate.  We need to multiply this to get the
   * desired frequency.  sam_tc_divisor() should have already assure
   * that we can do this without overflowing a 32-bit unsigned integer.
   */

  fdiv = div * frequency;
  DEBUGASSERT(div > 0 && div <= fdiv); /* Will check for integer overflow */

  /* Calculate the actual counter value from this divider and the tc input
   * frequency.
   */

  regval = BOARD_MCK_FREQUENCY / fdiv;
  sam_tc_setregister(poll_tc, TC_REGC, regval);
  sam_tc_attach(poll_tc, timer_handler, NULL, TC_INT_CPCS);

  /* And start the timer */

  sam_tc_start(poll_tc);

  return 0;
}

#endif /* SAMV7_UART_RXDMA_POLL */
