/****************************************************************************
 * arch/avr/src/avrdx/avrdx_lowconsole.c
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
#include "avrdx_config.h"

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <arch/board/board.h>
#include <avr/io.h>

#include "avr_internal.h"
#include "avrdx.h"
#include "avrdx_serial.h"

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

/* BAUD rates for USART peripherals as configured (or not) in Kconfig.
 * Helps us to avoid switch statement
 */

#ifdef CONFIG_MCU_SERIAL
static const IOBJ uint32_t avrdx_usart_baud[] =
{
#  ifdef CONFIG_AVR_HAS_USART_2

#    if defined(CONFIG_USART0_SERIALDRIVER)
  CONFIG_USART0_BAUD,
#    else
  0,
#    endif

#    if defined(CONFIG_USART1_SERIALDRIVER)
  CONFIG_USART1_BAUD,
#    else
  0,
#    endif

#    if defined(CONFIG_USART2_SERIALDRIVER)
  CONFIG_USART2_BAUD
#    else
  0
#    endif

  #  endif /* ifdef CONFIG_AVR_HAS_USART_2 */

#  ifdef CONFIG_AVR_HAS_USART_4
#    if defined(CONFIG_USART3_SERIALDRIVER)
  , CONFIG_USART3_BAUD
#    else
  , 0
#    endif
#    if defined(CONFIG_USART4_SERIALDRIVER)
  , CONFIG_USART4_BAUD
#    else
  , 0
#    endif
#  endif /* ifdef CONFIG_AVR_HAS_USART_4 */

#  ifdef CONFIG_AVR_HAS_USART_5
#    if defined(CONFIG_USART5_SERIALDRIVER)
  , CONFIG_USART5_BAUD
#    else
  , 0
#    endif
#  endif /* ifdef CONFIG_AVR_HAS_USART_5 */
};

/* Peripheral settings for USARTn.CTRLC. Combined from multiple Kconfig
 * settings (parity, stop bits etc.)
 */

static const IOBJ uint8_t avrdx_usart_ctrlc[] =
{
#  ifdef CONFIG_AVR_HAS_USART_2

  /* USART0 */

#    if defined(CONFIG_USART0_SERIALDRIVER)
  (
#      if (CONFIG_USART0_PARITY == 0)
  USART_PMODE_DISABLED_GC
#      elif (CONFIG_USART0_PARITY == 1)
  USART_PMODE_ODD_GC
#      elif (CONFIG_USART0_PARITY == 2)
  USART_PMODE_EVEN_GC
#      else
#        error CONFIG_USART0_PARITY
#      endif
  |
#      if (CONFIG_USART0_2STOP == 0)
  USART_SBMODE_1BIT_GC
#      elif (CONFIG_USART0_2STOP == 1)
  USART_SBMODE_2BIT_GC
#      else
#        error CONFIG_USART0_2STOP
#      endif
  |
#      if (CONFIG_USART0_BITS == 7)
  USART_CHSIZE_7BIT_GC
#      elif (CONFIG_USART0_BITS == 8)
  USART_CHSIZE_8BIT_GC
#      else
#        error CONFIG_USART0_BITS
#      endif
  ),
#    else /* for if defined(CONFIG_USART0_SERIALDRIVER) */
  0,
#    endif /* if defined(CONFIG_USART0_SERIALDRIVER) */

  /* USART1 */

#    if defined(CONFIG_USART1_SERIALDRIVER)
  (
#      if (CONFIG_USART1_PARITY == 0)
  USART_PMODE_DISABLED_GC
#      elif (CONFIG_USART1_PARITY == 1)
  USART_PMODE_ODD_GC
#      elif (CONFIG_USART1_PARITY == 2)
  USART_PMODE_EVEN_GC
#      else
#        error CONFIG_USART1_PARITY
#      endif
  |
#      if (CONFIG_USART1_2STOP == 0)
  USART_SBMODE_1BIT_GC
#      elif (CONFIG_USART1_2STOP == 1)
  USART_SBMODE_2BIT_GC
#      else
#        error CONFIG_USART1_2STOP
#      endif
  |
#      if (CONFIG_USART1_BITS == 7)
  USART_CHSIZE_7BIT_GC
#      elif (CONFIG_USART1_BITS == 8)
  USART_CHSIZE_8BIT_GC
#      else
#        error CONFIG_USART1_BITS
#      endif
  ),
#    else /* for if defined(CONFIG_USART1_SERIALDRIVER) */
  0,
#    endif /* if defined(CONFIG_USART1_SERIALDRIVER) */

  /* USART2 */

#    if defined(CONFIG_USART2_SERIALDRIVER)
  (
#      if (CONFIG_USART2_PARITY == 0)
  USART_PMODE_DISABLED_GC
#      elif (CONFIG_USART2_PARITY == 1)
  USART_PMODE_ODD_GC
#      elif (CONFIG_USART2_PARITY == 2)
  USART_PMODE_EVEN_GC
#      else
#        error CONFIG_USART2_PARITY
#      endif
  |
#      if (CONFIG_USART2_2STOP == 0)
  USART_SBMODE_1BIT_GC
#      elif (CONFIG_USART2_2STOP == 1)
  USART_SBMODE_2BIT_GC
#      else
#        error CONFIG_USART2_2STOP
#      endif
  |
#      if (CONFIG_USART2_BITS == 7)
  USART_CHSIZE_7BIT_GC
#      elif (CONFIG_USART2_BITS == 8)
  USART_CHSIZE_8BIT_GC
#      else
#        error CONFIG_USART2_BITS
#      endif
  )
#    else /* for if defined(CONFIG_USART2_SERIALDRIVER) */
  0
#    endif /* if defined(CONFIG_USART2_SERIALDRIVER) */

#  endif /* ifdef CONFIG_AVR_HAS_USART_2 */

  /* Definitions for chips that have USART4 */

#  ifdef CONFIG_AVR_HAS_USART_4

  /* USART3 */

#    if defined(CONFIG_USART3_SERIALDRIVER)
  , (
#      if (CONFIG_USART3_PARITY == 0)
  USART_PMODE_DISABLED_GC
#      elif (CONFIG_USART3_PARITY == 1)
  USART_PMODE_ODD_GC
#      elif (CONFIG_USART3_PARITY == 2)
  USART_PMODE_EVEN_GC
#      else
#        error CONFIG_USART3_PARITY
#      endif
  |
#      if (CONFIG_USART3_2STOP == 0)
  USART_SBMODE_1BIT_GC
#      elif (CONFIG_USART3_2STOP == 1)
  USART_SBMODE_2BIT_GC
#      else
#        error CONFIG_USART3_2STOP
#      endif
  |
#      if (CONFIG_USART3_BITS == 7)
  USART_CHSIZE_7BIT_GC
#      elif (CONFIG_USART3_BITS == 8)
  USART_CHSIZE_8BIT_GC
#      else
#        error CONFIG_USART3_BITS
#      endif
  )
#    else /* for if defined(CONFIG_USART3_SERIALDRIVER) */
  , 0
#    endif /* if defined(CONFIG_USART3_SERIALDRIVER) */

  /* USART4 */

#    if defined(CONFIG_USART4_SERIALDRIVER)
  , (
#      if (CONFIG_USART4_PARITY == 0)
  USART_PMODE_DISABLED_GC
#      elif (CONFIG_USART4_PARITY == 1)
  USART_PMODE_ODD_GC
#      elif (CONFIG_USART4_PARITY == 2)
  USART_PMODE_EVEN_GC
#      else
#        error CONFIG_USART4_PARITY
#      endif
  |
#      if (CONFIG_USART4_2STOP == 0)
  USART_SBMODE_1BIT_GC
#      elif (CONFIG_USART4_2STOP == 1)
  USART_SBMODE_2BIT_GC
#      else
#        error CONFIG_USART4_2STOP
#      endif
  |
#      if (CONFIG_USART4_BITS == 7)
  USART_CHSIZE_7BIT_GC
#      elif (CONFIG_USART4_BITS == 8)
  USART_CHSIZE_8BIT_GC
#      else
#        error CONFIG_USART4_BITS
#      endif
  )
#    else /* for if defined(CONFIG_USART4_SERIALDRIVER) */
  , 0
#    endif /* if defined(CONFIG_USART4_SERIALDRIVER) */

#  endif /* ifdef CONFIG_AVR_HAS_USART_4 */

  /* Definitions for chips that have USART5 */

#  ifdef CONFIG_AVR_HAS_USART_5

  /* USART5 */

#    if defined(CONFIG_USART5_SERIALDRIVER)
  , (
#      if (CONFIG_USART5_PARITY == 0)
  USART_PMODE_DISABLED_GC
#      elif (CONFIG_USART5_PARITY == 1)
  USART_PMODE_ODD_GC
#      elif (CONFIG_USART5_PARITY == 2)
  USART_PMODE_EVEN_GC
#      else
#        error CONFIG_USART5_PARITY
#      endif
  |
#      if (CONFIG_USART5_2STOP == 0)
  USART_SBMODE_1BIT_GC
#      elif (CONFIG_USART5_2STOP == 1)
  USART_SBMODE_2BIT_GC
#      else
#        error CONFIG_USART5_2STOP
#      endif
  |
#      if (CONFIG_USART5_BITS == 7)
  USART_CHSIZE_7BIT_GC
#      elif (CONFIG_USART5_BITS == 8)
  USART_CHSIZE_8BIT_GC
#      else
#        error CONFIG_USART5_BITS
#      endif
  )
#    else /* for if defined(CONFIG_USART5_SERIALDRIVER) */
  , 0
#    endif /* if defined(CONFIG_USART5_SERIALDRIVER) */

#  endif /* ifdef CONFIG_AVR_HAS_USART_5 */
};

#endif /* ifdef CONFIG_MCU_SERIAL */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_MCU_SERIAL

/****************************************************************************
 * Name: avrdx_usart_reset
 *
 * Description:
 *   Reset USARTn.
 *
 * Input Parameters:
 *   struct avrdx_uart_priv_s identifying which peripheral is to be reset
 *
 ****************************************************************************/

void avrdx_usart_reset(struct avrdx_uart_priv_s *priv)
{
  /* Shortcut variable, also should prevent the compiler from doing
   * something dumb like re-reading the value from program memory
   * over and over
   */

  avr_usart_t *usart;

  usart = &(AVRDX_USART(priv->usart_n));

  /* Disable the peripheral, switch I/O pins to their default state */

  AVRDX_USART_PORT(priv->usart_n).DIRCLR = \
    avrdx_usart_tx_pins[priv->usart_n];

  usart->CTRLA = 0;
  usart->CTRLB = 0;
  usart->CTRLC = 0;

  /* Flags cleared by writing 1 to them, otherwise zeroes */

  usart->STATUS = USART_RXCIF_bm | \
    USART_TXCIF_bm | \
    USART_RXSIF_bm | \
    USART_ISFIF_bm | \
    USART_BDF_bm;
}

#endif

#ifdef CONFIG_MCU_SERIAL

/****************************************************************************
 * Name: avrdx_usart_configure
 *
 * Description:
 *   Configure USARTn
 *
 * Input Parameters:
 *   struct avrdx_uart_priv_s identifying which peripheral
 *   is to be configured
 *
 ****************************************************************************/

void avrdx_usart_configure(struct avrdx_uart_priv_s *priv)
{
  irqstate_t irqstate;
  uint32_t baud_temp; /* Being calculated, needs 32 bits */
  uint32_t temp32;
  uint8_t double_speed;
  uint8_t temp;

  avr_usart_t *usart;

  usart = &(AVRDX_USART(priv->usart_n));

  /* For interrupt-driven USART operation, global interrupts must
   * be disabled during the initialization (datasheet rev. C, section 25.3.1)
   */

  irqstate = up_irq_save();

  /* Initialization as per section 25.3.1., datasheet rev. C */

  /* 1. set the baud rate */

  baud_temp = avrdx_current_freq_per() * 64;
  baud_temp /= avrdx_usart_baud[priv->usart_n];

  temp32 = baud_temp / 16;
  if (temp32 < 64)
    {
      /* Attempt to go above minimum value at the cost of more strict
       * requirement for transmitter/receiver clock match
       */

      baud_temp /= 8;
      double_speed = USART_RXMODE_CLK2X_GC;
    }
  else
    {
      baud_temp = temp32;
      double_speed = USART_RXMODE_NORMAL_GC;
    }

  if (baud_temp < 64)
    {
      /* If we still ended up below minimum value, correct it
       * and expect things to not work
       */

      usart->BAUD = 64;
    }
  else
    {
      if (baud_temp > 65535)
        {
          /* Above maximum value, won't work either */

          usart->BAUD = 65535;
        }
      else
        {
          usart->BAUD = baud_temp;
        }
    }

  /* 2. set frame format and mode of operation */

  usart->CTRLC = avrdx_usart_ctrlc[priv->usart_n];

  /* 3. configure TXD pin as output */

  AVRDX_USART_PORT(priv->usart_n).DIRSET = \
    avrdx_usart_tx_pins[priv->usart_n];

  /* 4. enable the transmitter and the receiver */

  temp = USART_RXEN_bm | USART_TXEN_bm;
  usart->CTRLB = (temp | double_speed);

  up_irq_restore(irqstate);
}

#endif /* ifdef CONFIG_MCU_SERIAL */

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the initialization sequence to configure the serial console
 *   uart (only).
 *
 ****************************************************************************/

void up_consoleinit(void)
{
#ifdef AVRDX_SERIAL_CONSOLE_USART_N
  struct avrdx_uart_priv_s priv =
  {
    .usart_n = AVRDX_SERIAL_CONSOLE_USART_N
  };

  avrdx_usart_configure(&priv);
#endif
}

/****************************************************************************
 * Name: avr_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void avr_lowputc(char ch)
{
#ifdef AVRDX_SERIAL_CONSOLE_USART_N
  avr_usart_t *usart;

  usart = &(AVRDX_USART(AVRDX_SERIAL_CONSOLE_USART_N));
  while (!(usart->STATUS & USART_DREIF_bm));
  usart->TXDATAL = ch;
#endif
}
