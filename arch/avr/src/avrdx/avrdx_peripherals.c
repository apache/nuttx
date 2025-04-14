/****************************************************************************
 * arch/avr/src/avrdx/avrdx_peripherals.c
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
#include "avrdx.h"
#include "avrdx_gpio.h"
#include <avr/io.h>
#include <arch/irq.h>

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
 * Private Data
 ****************************************************************************/

/* FRQSEL bits in CLKCTRL.OSCHFCTRLA I/O register correspond
 * to this frequency of internal oscillator in MHz. Index of 4 is reserved
 */

static const IOBJ uint8_t avrdx_frqsel_mhz[] = \
  {
    1, 2, 3, 4, 0, 8, 12, 16, 20, 24
  };

/* PDIV bits in CLKCTRL.MCLKCTRLB I/O register specify ratio of a prescaler
 * between MAIN clock and PER(ipheral) clock. Indices of 6 and 7 are reserved
 */

static const IOBJ uint8_t avrdx_main_pdiv[] = \
  {
    2, 4, 8, 16, 32, 64, 0, 0, \
    6, 10, 12, 24, 48
  };

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_MCU_SERIAL

/* Map of I/O ports connected to specific USARTn peripherals. USARTn
 * peripheral with index n is connected to the I/O port ports[n].
 * Ports are "numbered" with letters, PORTA is indexed 0 and so on
 */

const IOBJ uint8_t avrdx_usart_ports[] =
{
#  ifdef CONFIG_AVR_HAS_USART_2
  AVRDX_GPIO_PORTA_IDX,   /* A, C, F */
  AVRDX_GPIO_PORTC_IDX,
  AVRDX_GPIO_PORTF_IDX
#  endif
#  ifdef CONFIG_AVR_HAS_USART_4
  , AVRDX_GPIO_PORTB_IDX  /* B, E */
  , AVRDX_GPIO_PORTE_IDX
#  endif
#  ifdef CONFIG_AVR_HAS_USART_5
  , AVRDX_GPIO_PORTG_IDX  /* port G */
#  endif
};

/* I/O pin constant for each USARTn peripheral. Always the same
 * for all USART peripherals but this way we can determine
 * correct pin without run-time switch statement on peripheral
 * index (index being n in USARTn)
 */

const IOBJ uint8_t avrdx_usart_tx_pins[] =
{
#  ifdef CONFIG_AVR_HAS_USART_2
#    if !defined(CONFIG_AVR_USART0_ALT)
  PIN0_bm ,
#    else
  PIN4_bm ,
#    endif
#    if !defined(CONFIG_AVR_USART1_ALT)
  PIN0_bm ,
#    else
  PIN4_bm ,
#    endif
#    if !defined(CONFIG_AVR_USART2_ALT)
  PIN0_bm
#    else
  PIN4_bm
#    endif
#  endif

#  ifdef CONFIG_AVR_HAS_USART_4
#    if !defined(CONFIG_AVR_USART3_ALT)
  , PIN0_bm
#    else
  , PIN4_bm
#    endif
#    if !defined(CONFIG_AVR_USART4_ALT)
  , PIN0_bm
#    else
  , PIN4_bm
#    endif
#  endif

#  ifdef CONFIG_AVR_HAS_USART_5
#    if !defined(CONFIG_AVR_USART5_ALT)
  , PIN0_bm
#    else
  , PIN4_bm
#    endif
#  endif
};

#endif

/* Interrupt vector numbers for pin change interrupts
 * indexed by port number (with port A being indexed as 0,
 * port B as 1 etc.) Does not skip vectors that the chip
 * does not have to keep the index identical for all chips.
 */

const IOBJ uint8_t avrdx_gpio_irq_vectors[] =
{
  AVRDX_IRQ_PORTA_PORT,

#ifdef CONFIG_AVR_HAS_PORTB
  AVRDX_IRQ_PORTB_PORT,
#else
  0,
#endif

  AVRDX_IRQ_PORTC_PORT,
  AVRDX_IRQ_PORTD_PORT,
#ifdef CONFIG_AVR_HAS_PORTE
  AVRDX_IRQ_PORTE_PORT,
#else
  0,
#endif

  AVRDX_IRQ_PORTF_PORT

#ifdef CONFIG_AVR_HAS_PORTG
  , AVRDX_IRQ_PORTG_PORT
#else
  , 0
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_current_freq_per
 *
 * Description:
 *   Calculate and return current f_per (peripheral clock frequency)
 *
 * Assumptions:
 *   Main clock source is internal oscillator
 *
 ****************************************************************************/

uint32_t avrdx_current_freq_per()
{
  uint32_t f_per;

  /* Shortcut variables */

  uint8_t frqsel;
  uint8_t pdiv;
  uint8_t mclkctrlb;

  /* Calculate frequency in MHz, then divide it by main prescaler,
   * if set.
   */

  frqsel = (CLKCTRL.OSCHFCTRLA & CLKCTRL_FRQSEL_GM) >> CLKCTRL_FRQSEL_GP;
  f_per = 1000000UL * avrdx_frqsel_mhz[frqsel];

  /* Read this once, no point in re-reading */

  mclkctrlb = CLKCTRL.MCLKCTRLB;
  if (mclkctrlb & CLKCTRL_PEN_bm)
    {
      pdiv = (mclkctrlb & CLKCTRL_PDIV_GM) >> CLKCTRL_PDIV_GP;
      f_per /= avrdx_main_pdiv[pdiv];
    }

  /* Currently, rest of the code only supports internal oscillator
   * and its frequency is pre-set using Kconfig. Nevertheless, that
   * can change at some point and this function accounts for some
   * of that.
   *
   * It doesn't account for the chip being clocked by external source
   * though, that's to be done.
   */

  return f_per;
}
