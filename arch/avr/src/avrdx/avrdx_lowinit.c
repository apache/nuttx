/****************************************************************************
 * arch/avr/src/avrdx/avrdx_lowinit.c
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
#include <avr/power.h>

#include <arch/board/board.h>
#include "avrdx_config.h"
#include "avr_internal.h"
#include "avrdx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avr_lowinit
 *
 * Description:
 *   This performs basic initialization of the hardware, notably sets main
 *   clock and configures USART used for the serial console etc. (USART is
 *   configured early to make the console output available as soon
 *   as possible.)
 *
 *
 ****************************************************************************/

void avr_lowinit(void)
{
  /* Set the system clock */

#if (CONFIG_AVRDX_HFO_CLOCK_FREQ == 24000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_24M_GC));
#elif (CONFIG_AVRDX_HFO_CLOCK_FREQ == 20000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_20M_GC));
#elif (CONFIG_AVRDX_HFO_CLOCK_FREQ == 16000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_16M_GC));
#elif (CONFIG_AVRDX_HFO_CLOCK_FREQ == 12000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_12M_GC));
#elif (CONFIG_AVRDX_HFO_CLOCK_FREQ == 8000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_8M_GC));
#elif (CONFIG_AVRDX_HFO_CLOCK_FREQ == 4000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_4M_GC));
#elif (CONFIG_AVRDX_HFO_CLOCK_FREQ == 3000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_3M_GC));
#elif (CONFIG_AVRDX_HFO_CLOCK_FREQ == 2000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_2M_GC));
#elif (CONFIG_AVRDX_HFO_CLOCK_FREQ == 1000000)
  _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
    ((CLKCTRL.OSCHFCTRLA & (~CLKCTRL_FRQSEL_GM)) | CLKCTRL_FRQSEL_1M_GC));
#else
  /* At some point, add support to have the board supply
   * its own clock setting (ie. support external clock)
   */
# error Unsupported frequency
#endif

  /* Initialize a console (only try if any serial port is configured).
   * If any serial peripheral is configured to be a serial console,
   * it is set up early
   */

#ifdef CONFIG_MCU_SERIAL
  up_consoleinit();
#endif

  /* Perform board-level initialization */

  avrdx_boardinitialize();
}
