/****************************************************************************
 * arch/arm/src/at32/at32_lowputc.c
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

#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "at32.h"
#include "at32_rcc.h"
#include "at32_gpio.h"
#include "at32_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define AT32_CONSOLE_BASE     AT32_USART1_BASE
#    define AT32_APBCLOCK         AT32_PCLK2_FREQUENCY
#    define AT32_CONSOLE_APBREG   AT32_CRM_APB2EN
#    define AT32_CONSOLE_APBEN    CRM_APB2EN_USART1EN
#    define AT32_CONSOLE_BAUD     CONFIG_USART1_BAUD
#    define AT32_CONSOLE_BITS     CONFIG_USART1_BITS
#    define AT32_CONSOLE_PARITY   CONFIG_USART1_PARITY
#    define AT32_CONSOLE_2STOP    CONFIG_USART1_2STOP
#    define AT32_CONSOLE_TX       GPIO_USART1_TX
#    define AT32_CONSOLE_RX       GPIO_USART1_RX
#    ifdef CONFIG_USART1_RS485
#      define AT32_CONSOLE_RS485_DIR GPIO_USART1_RS485_DIR
#      if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
#        define AT32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define AT32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define AT32_CONSOLE_BASE     AT32_USART2_BASE
#    define AT32_APBCLOCK         AT32_PCLK1_FREQUENCY
#    define AT32_CONSOLE_APBREG   AT32_CRM_APB1EN
#    define AT32_CONSOLE_APBEN    CRM_APB1EN_USART2EN
#    define AT32_CONSOLE_BAUD     CONFIG_USART2_BAUD
#    define AT32_CONSOLE_BITS     CONFIG_USART2_BITS
#    define AT32_CONSOLE_PARITY   CONFIG_USART2_PARITY
#    define AT32_CONSOLE_2STOP    CONFIG_USART2_2STOP
#    define AT32_CONSOLE_TX       GPIO_USART2_TX
#    define AT32_CONSOLE_RX       GPIO_USART2_RX
#    ifdef CONFIG_USART2_RS485
#      define AT32_CONSOLE_RS485_DIR GPIO_USART2_RS485_DIR
#      if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
#        define AT32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define AT32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define AT32_CONSOLE_BASE     AT32_USART3_BASE
#    define AT32_APBCLOCK         AT32_PCLK1_FREQUENCY
#    define AT32_CONSOLE_APBREG   AT32_CRM_APB1EN
#    define AT32_CONSOLE_APBEN    CRM_APB1EN_USART3EN
#    define AT32_CONSOLE_BAUD     CONFIG_USART3_BAUD
#    define AT32_CONSOLE_BITS     CONFIG_USART3_BITS
#    define AT32_CONSOLE_PARITY   CONFIG_USART3_PARITY
#    define AT32_CONSOLE_2STOP    CONFIG_USART3_2STOP
#    define AT32_CONSOLE_TX       GPIO_USART3_TX
#    define AT32_CONSOLE_RX       GPIO_USART3_RX
#    ifdef CONFIG_USART3_RS485
#      define AT32_CONSOLE_RS485_DIR GPIO_USART3_RS485_DIR
#      if (CONFIG_USART3_RS485_DIR_POLARITY == 0)
#        define AT32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define AT32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define AT32_CONSOLE_BASE     AT32_UART4_BASE
#    define AT32_APBCLOCK         AT32_PCLK1_FREQUENCY
#    define AT32_CONSOLE_APBREG   AT32_CRM_APB1EN
#    define AT32_CONSOLE_APBEN    CRM_APB1EN_UART4EN
#    define AT32_CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define AT32_CONSOLE_BITS     CONFIG_UART4_BITS
#    define AT32_CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define AT32_CONSOLE_2STOP    CONFIG_UART4_2STOP
#    define AT32_CONSOLE_TX       GPIO_UART4_TX
#    define AT32_CONSOLE_RX       GPIO_UART4_RX
#    ifdef CONFIG_UART4_RS485
#      define AT32_CONSOLE_RS485_DIR GPIO_UART4_RS485_DIR
#      if (CONFIG_UART4_RS485_DIR_POLARITY == 0)
#        define AT32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define AT32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define AT32_CONSOLE_BASE     AT32_UART5_BASE
#    define AT32_APBCLOCK         AT32_PCLK1_FREQUENCY
#    define AT32_CONSOLE_APBREG   AT32_CRM_APB1EN
#    define AT32_CONSOLE_APBEN    CRM_APB1EN_UART4EN
#    define AT32_CONSOLE_BAUD     CONFIG_UART5_BAUD
#    define AT32_CONSOLE_BITS     CONFIG_UART5_BITS
#    define AT32_CONSOLE_PARITY   CONFIG_UART5_PARITY
#    define AT32_CONSOLE_2STOP    CONFIG_UART5_2STOP
#    define AT32_CONSOLE_TX       GPIO_UART5_TX
#    define AT32_CONSOLE_RX       GPIO_UART5_RX
#    ifdef CONFIG_UART5_RS485
#      define AT32_CONSOLE_RS485_DIR GPIO_UART5_RS485_DIR
#      if (CONFIG_UART5_RS485_DIR_POLARITY == 0)
#        define AT32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define AT32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#    define AT32_CONSOLE_BASE     AT32_USART6_BASE
#    define AT32_APBCLOCK         AT32_PCLK2_FREQUENCY
#    define AT32_CONSOLE_APBREG   AT32_CRM_APB2EN
#    define AT32_CONSOLE_APBEN    CRM_APB2EN_USART6EN
#    define AT32_CONSOLE_BAUD     CONFIG_USART6_BAUD
#    define AT32_CONSOLE_BITS     CONFIG_USART6_BITS
#    define AT32_CONSOLE_PARITY   CONFIG_USART6_PARITY
#    define AT32_CONSOLE_2STOP    CONFIG_USART6_2STOP
#    define AT32_CONSOLE_TX       GPIO_USART6_TX
#    define AT32_CONSOLE_RX       GPIO_USART6_RX
#    ifdef CONFIG_USART6_RS485
#      define AT32_CONSOLE_RS485_DIR GPIO_USART6_RS485_DIR
#      if (CONFIG_USART6_RS485_DIR_POLARITY == 0)
#        define AT32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define AT32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define AT32_CONSOLE_BASE     AT32_UART7_BASE
#    define AT32_APBCLOCK         AT32_PCLK1_FREQUENCY
#    define AT32_CONSOLE_APBREG   AT32_CRM_APB1EN
#    define AT32_CONSOLE_APBEN    CRM_APB1EN_UART7EN
#    define AT32_CONSOLE_BAUD     CONFIG_UART7_BAUD
#    define AT32_CONSOLE_BITS     CONFIG_UART7_BITS
#    define AT32_CONSOLE_PARITY   CONFIG_UART7_PARITY
#    define AT32_CONSOLE_2STOP    CONFIG_UART7_2STOP
#    define AT32_CONSOLE_TX       GPIO_UART7_TX
#    define AT32_CONSOLE_RX       GPIO_UART7_RX
#    ifdef CONFIG_UART7_RS485
#      define AT32_CONSOLE_RS485_DIR GPIO_UART7_RS485_DIR
#      if (CONFIG_UART7_RS485_DIR_POLARITY == 0)
#        define AT32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define AT32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART8_SERIAL_CONSOLE)
#    define AT32_CONSOLE_BASE     AT32_UART8_BASE
#    define AT32_APBCLOCK         AT32_PCLK1_FREQUENCY
#    define AT32_CONSOLE_APBREG   AT32_CRM_APB1EN
#    define AT32_CONSOLE_APBEN    CRM_APB1EN_UART8EN
#    define AT32_CONSOLE_BAUD     CONFIG_UART8_BAUD
#    define AT32_CONSOLE_BITS     CONFIG_UART8_BITS
#    define AT32_CONSOLE_PARITY   CONFIG_UART8_PARITY
#    define AT32_CONSOLE_2STOP    CONFIG_UART8_2STOP
#    define AT32_CONSOLE_TX       GPIO_UART8_TX
#    define AT32_CONSOLE_RX       GPIO_UART8_RX
#    ifdef CONFIG_UART8_RS485
#      define AT32_CONSOLE_RS485_DIR GPIO_UART8_RS485_DIR
#      if (CONFIG_UART8_RS485_DIR_POLARITY == 0)
#        define AT32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define AT32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  endif

/* CR1 settings */

#  if AT32_CONSOLE_BITS == 9
#    define USART_CR1_M_VALUE (USART_CTRL1_DBN0 | (~USART_CTRL1_DBN1))
#  elif AT32_CONSOLE_BITS == 7
#    define USART_CR1_M_VALUE ((~USART_CTRL1_DBN0) | USART_CTRL1_DBN1)
#  else
#    define USART_CR1_M_VALUE 0
#  endif

#  if AT32_CONSOLE_PARITY == 1
#    define USART_CR1_PARITY_VALUE (USART_CTRL1_PEN|USART_CTRL1_PSEL)
#  elif AT32_CONSOLE_PARITY == 2
#    define USART_CR1_PARITY_VALUE USART_CTRL1_PEN
#  else
#    define USART_CR1_PARITY_VALUE 0
#  endif

#    define USART_CR1_CLRBITS\
      (USART_CTRL1_DBN0 | USART_CTRL1_DBN1 | USART_CTRL1_PEN | USART_CTRL1_PSEL | \
       USART_CTRL1_TEN | USART_CTRL1_REN | USART_CTRL1_IDLEIEN | USART_CTRL1_RDBFIEN | \
       USART_CTRL1_TDCIEN |USART_CTRL1_TDBEIEN | USART_CTRL1_PERRIEN)

#  define USART_CR1_SETBITS (USART_CR1_M_VALUE|USART_CR1_PARITY_VALUE)

/* CR2 settings */

#  if AT32_CONSOLE_2STOP != 0
#    define USART_CR2_STOP2_VALUE USART_CTRL2_STOPBN_20
#  else
#    define USART_CR2_STOP2_VALUE 0
#  endif

#    define USART_CR2_CLRBITS \
       (USART_CTRL2_STOPBN_MASK | USART_CTRL2_CLKEN | USART_CTRL2_CLKPOL | USART_CTRL2_CLKPHA | \
        USART_CTRL2_LBCP | USART_CTRL2_BFIEN)

#  define USART_CR2_SETBITS USART_CR2_STOP2_VALUE

/* CR3 settings */

#    define USART_CR3_CLRBITS \
      (USART_CTRL3_CTSCFIEN | USART_CTRL3_CTSEN | USART_CTRL3_RTSEN | USART_CTRL3_ERRIEN)

#  define USART_CR3_SETBITS 0

#endif /* HAVE_CONSOLE */

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
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  /* Wait until the TX data register is empty */

  while ((getreg32(AT32_CONSOLE_BASE + AT32_USART_STS_OFFSET) &
         USART_STS_TDC) == 0);
#ifdef AT32_CONSOLE_RS485_DIR
  at32_gpiowrite(AT32_CONSOLE_RS485_DIR,
                  AT32_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Then send the character */

  putreg32((uint32_t)ch, AT32_CONSOLE_BASE + AT32_USART_DT_OFFSET);

#ifdef AT32_CONSOLE_RS485_DIR
  while ((getreg32(AT32_CONSOLE_BASE + AT32_USART_STS_OFFSET) &
         USART_STS_TDC) == 0);
  at32_gpiowrite(AT32_CONSOLE_RS485_DIR,
                  !AT32_CONSOLE_RS485_DIR_POLARITY);
#endif

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: at32_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

#if defined(CONFIG_AT32_AT32F43XX)

void at32_lowsetup(void)
{
#if defined(HAVE_SERIALDRIVER)
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t cr;
#endif

#if defined(HAVE_CONSOLE)
  /* Enable USART APB1/2 clock */

  modifyreg32(AT32_CONSOLE_APBREG, 0, AT32_CONSOLE_APBEN);
#endif

  /* Enable the console USART and configure GPIO pins needed for rx/tx.
   *
   * NOTE: Clocking for selected U[S]ARTs was already provided in at32_rcc.c
   */

#ifdef AT32_CONSOLE_TX
  at32_configgpio(AT32_CONSOLE_TX);
#endif
#ifdef AT32_CONSOLE_RX
  at32_configgpio(AT32_CONSOLE_RX);
#endif

#ifdef AT32_CONSOLE_RS485_DIR
  at32_configgpio(AT32_CONSOLE_RS485_DIR);
  at32_gpiowrite(AT32_CONSOLE_RS485_DIR,
                  !AT32_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Enable and configure the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Ensure the USART is disabled because some bits of the following
   * registers cannot be modified otherwise.
   *
   * Although the USART is expected to be disabled at power on reset, this
   * might not be the case if we boot from a serial bootloader that does not
   * clean up properly.
   */

  cr  = getreg32(AT32_CONSOLE_BASE + AT32_USART_CTRL1_OFFSET);
  cr &= ~USART_CTRL1_UEN;
  putreg32(cr, AT32_CONSOLE_BASE + AT32_USART_CTRL1_OFFSET);

  /* Configure CR2 */

  cr  = getreg32(AT32_CONSOLE_BASE + AT32_USART_CTRL2_OFFSET);
  cr &= ~USART_CR2_CLRBITS;
  cr |= USART_CR2_SETBITS;
  putreg32(cr, AT32_CONSOLE_BASE + AT32_USART_CTRL2_OFFSET);

  /* Configure CR1 */

  cr  = getreg32(AT32_CONSOLE_BASE + AT32_USART_CTRL1_OFFSET);
  cr &= ~USART_CR1_CLRBITS;
  cr |= USART_CR1_SETBITS;
  putreg32(cr, AT32_CONSOLE_BASE + AT32_USART_CTRL1_OFFSET);

  /* Configure CR3 */

  cr  = getreg32(AT32_CONSOLE_BASE + AT32_USART_CTRL3_OFFSET);
  cr &= ~USART_CR3_CLRBITS;
  cr |= USART_CR3_SETBITS;
  putreg32(cr, AT32_CONSOLE_BASE + AT32_USART_CTRL3_OFFSET);

  /* Configure the USART Baud Rate */

  uint32_t temp_val = (AT32_APBCLOCK * 10 / AT32_CONSOLE_BAUD);
  temp_val = ((temp_val % 10) < 5) ? (temp_val / 10) : (temp_val / 10 + 1);

  putreg32(temp_val, AT32_CONSOLE_BASE + AT32_USART_BAUDR_OFFSET);

  /* Enable Rx, Tx, and the USART */

  cr |= (USART_CTRL1_UEN | USART_CTRL1_TEN | USART_CTRL1_REN);
  putreg32(cr, AT32_CONSOLE_BASE + AT32_USART_CTRL1_OFFSET);

#endif /* HAVE_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_SERIALDRIVER */
}

#else
#  error "Unsupported AT32 chip"
#endif
