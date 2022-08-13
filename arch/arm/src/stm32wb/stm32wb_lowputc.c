/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_lowputc.c
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
#include "stm32wb.h"
#include "stm32wb_rcc.h"
#include "stm32wb_gpio.h"
#include "stm32wb_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#    define STM32WB_CONSOLE_BASE     STM32WB_LPUART1_BASE
#    define STM32WB_APBCLOCK         STM32WB_PCLK1_FREQUENCY
#    define STM32WB_CONSOLE_APBREG   STM32WB_RCC_APB1ENR2
#    define STM32WB_CONSOLE_APBEN    RCC_APB1ENR2_LPUART1EN
#    define STM32WB_CONSOLE_BAUD     CONFIG_LPUART1_BAUD
#    define STM32WB_CONSOLE_BITS     CONFIG_LPUART1_BITS
#    define STM32WB_CONSOLE_PARITY   CONFIG_LPUART1_PARITY
#    define STM32WB_CONSOLE_2STOP    CONFIG_LPUART1_2STOP
#    define STM32WB_CONSOLE_TX       GPIO_LPUART1_TX
#    define STM32WB_CONSOLE_RX       GPIO_LPUART1_RX
#    ifdef CONFIG_LPUART1_RS485
#      define STM32WB_CONSOLE_RS485_DIR GPIO_LPUART1_RS485_DIR
#      if (CONFIG_LPUART1_RS485_DIR_POLARITY == 0)
#        define STM32WB_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32WB_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define STM32WB_CONSOLE_BASE     STM32WB_USART1_BASE
#    define STM32WB_APBCLOCK         STM32WB_PCLK2_FREQUENCY
#    define STM32WB_CONSOLE_APBREG   STM32WB_RCC_APB2ENR
#    define STM32WB_CONSOLE_APBEN    RCC_APB2ENR_USART1EN
#    define STM32WB_CONSOLE_BAUD     CONFIG_USART1_BAUD
#    define STM32WB_CONSOLE_BITS     CONFIG_USART1_BITS
#    define STM32WB_CONSOLE_PARITY   CONFIG_USART1_PARITY
#    define STM32WB_CONSOLE_2STOP    CONFIG_USART1_2STOP
#    define STM32WB_CONSOLE_TX       GPIO_USART1_TX
#    define STM32WB_CONSOLE_RX       GPIO_USART1_RX
#    ifdef CONFIG_USART1_RS485
#      define STM32WB_CONSOLE_RS485_DIR GPIO_USART1_RS485_DIR
#      if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
#        define STM32WB_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32WB_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  endif

/* CR1 settings */

#  if STM32WB_CONSOLE_BITS == 9
#    define USART_CR1_M0_VALUE USART_CR1_M0
#    define USART_CR1_M1_VALUE 0
#  elif STM32WB_CONSOLE_BITS == 7
#    define USART_CR1_M0_VALUE 0
#    define USART_CR1_M1_VALUE USART_CR1_M1
#  else /* 8 bits */
#    define USART_CR1_M0_VALUE 0
#    define USART_CR1_M1_VALUE 0
#  endif

#  if STM32WB_CONSOLE_PARITY == 1 /* odd parity */
#    define USART_CR1_PARITY_VALUE (USART_CR1_PCE|USART_CR1_PS)
#  elif STM32WB_CONSOLE_PARITY == 2 /* even parity */
#    define USART_CR1_PARITY_VALUE USART_CR1_PCE
#  else /* no parity */
#    define USART_CR1_PARITY_VALUE 0
#  endif

#  define USART_CR1_CLRBITS \
    (USART_CR1_UESM | USART_CR1_RE | USART_CR1_TE | USART_CR1_PS | \
     USART_CR1_PCE | USART_CR1_WAKE | USART_CR1_M0 | USART_CR1_M1 | \
     USART_CR1_MME | USART_CR1_OVER8 | USART_CR1_DEDT_MASK | \
     USART_CR1_DEAT_MASK | USART_CR1_ALLINTS)

#  define USART_CR1_SETBITS (USART_CR1_M0_VALUE|USART_CR1_M1_VALUE|USART_CR1_PARITY_VALUE)

/* CR2 settings */

#  if STM32WB_CONSOLE_2STOP != 0
#    define USART_CR2_STOP2_VALUE USART_CR2_STOP2
#  else
#    define USART_CR2_STOP2_VALUE 0
#  endif

#  define USART_CR2_CLRBITS \
    (USART_CR2_ADDM7 | USART_CR2_LBDL | USART_CR2_LBDIE | USART_CR2_LBCL | \
     USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN | USART_CR2_STOP_MASK | \
     USART_CR2_LINEN | USART_CR2_SWAP | USART_CR2_RXINV | USART_CR2_TXINV | \
     USART_CR2_DATAINV | USART_CR2_MSBFIRST | USART_CR2_ABREN | \
     USART_CR2_ABRMOD_MASK | USART_CR2_RTOEN | USART_CR2_ADD_MASK)

#  define USART_CR2_SETBITS USART_CR2_STOP2_VALUE

/* CR3 settings */

#  define USART_CR3_CLRBITS \
    (USART_CR3_EIE | USART_CR3_IREN | USART_CR3_IRLP | USART_CR3_HDSEL | \
     USART_CR3_NACK | USART_CR3_SCEN | USART_CR3_DMAR | USART_CR3_DMAT | \
     USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_CTSIE | USART_CR3_ONEBIT | \
     USART_CR3_OVRDIS | USART_CR3_DDRE | USART_CR3_DEM | USART_CR3_DEP | \
     USART_CR3_SCARCNT_MASK | USART_CR3_WUS_MASK | USART_CR3_WUFIE)

#  define USART_CR3_SETBITS 0

#  undef USE_OVER8

/* Calculate USART BAUD rate divider */

/* Baud rate for standard USART (SPI mode included):
 *
 * In case of oversampling by 16, the equation is:
 *   baud    = fCK / UARTDIV
 *   UARTDIV = fCK / baud
 *
 * In case of oversampling by 8, the equation is:
 *
 *   baud    = 2 * fCK / UARTDIV
 *   UARTDIV = 2 * fCK / baud
 */

#  define STM32WB_USARTDIV8 \
      (((STM32WB_APBCLOCK << 1) + (STM32WB_CONSOLE_BAUD >> 1)) / STM32WB_CONSOLE_BAUD)
#  define STM32WB_USARTDIV16 \
      ((STM32WB_APBCLOCK + (STM32WB_CONSOLE_BAUD >> 1)) / STM32WB_CONSOLE_BAUD)

/* Use oversamply by 8 only if the divisor is small.  But what is small? */

#  if STM32WB_USARTDIV8 > 100
#    define STM32WB_BRR_VALUE STM32WB_USARTDIV16
#  else
#    define USE_OVER8 1
#    define STM32WB_BRR_VALUE \
      ((STM32WB_USARTDIV8 & 0xfff0) | ((STM32WB_USARTDIV8 & 0x000f) >> 1))
#  endif

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
 * Private Variables
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

  while ((getreg32(STM32WB_CONSOLE_BASE + STM32WB_USART_ISR_OFFSET) &
                   USART_ISR_TXE) == 0);
#ifdef STM32WB_CONSOLE_RS485_DIR
  stm32wb_gpiowrite(STM32WB_CONSOLE_RS485_DIR,
                    STM32WB_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Then send the character */

  putreg32((uint32_t)ch, STM32WB_CONSOLE_BASE + STM32WB_USART_TDR_OFFSET);

#ifdef STM32WB_CONSOLE_RS485_DIR
  while ((getreg32(STM32WB_CONSOLE_BASE + STM32WB_USART_ISR_OFFSET) &
                   USART_ISR_TC) == 0);
  stm32wb_gpiowrite(STM32WB_CONSOLE_RS485_DIR,
                    !STM32WB_CONSOLE_RS485_DIR_POLARITY);
#endif

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: stm32wb_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void stm32wb_lowsetup(void)
{
#if defined(HAVE_UART)
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t cr;
#endif

#if defined(HAVE_CONSOLE)
  /* Enable USART APB1/2 clock */

  modifyreg32(STM32WB_CONSOLE_APBREG, 0, STM32WB_CONSOLE_APBEN);
#endif

  /* Enable the console USART and configure GPIO pins needed for rx/tx.
   *
   * NOTE: Clocking for selected U[S]ARTs was already provided in
   * stm32wb_rcc.c
   */

#ifdef STM32WB_CONSOLE_TX
  stm32wb_configgpio(STM32WB_CONSOLE_TX);
#endif
#ifdef STM32WB_CONSOLE_RX
  stm32wb_configgpio(STM32WB_CONSOLE_RX);
#endif

#ifdef STM32WB_CONSOLE_RS485_DIR
  stm32wb_configgpio(STM32WB_CONSOLE_RS485_DIR);
  stm32wb_gpiowrite(STM32WB_CONSOLE_RS485_DIR,
                    !STM32WB_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Enable and configure the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Configure CR2 */

  cr  = getreg32(STM32WB_CONSOLE_BASE + STM32WB_USART_CR2_OFFSET);
  cr &= ~USART_CR2_CLRBITS;
  cr |= USART_CR2_SETBITS;
  putreg32(cr, STM32WB_CONSOLE_BASE + STM32WB_USART_CR2_OFFSET);

  /* Configure CR1 */

  cr  = getreg32(STM32WB_CONSOLE_BASE + STM32WB_USART_CR1_OFFSET);
  cr &= ~USART_CR1_CLRBITS;
  cr |= USART_CR1_SETBITS;
  putreg32(cr, STM32WB_CONSOLE_BASE + STM32WB_USART_CR1_OFFSET);

  /* Configure CR3 */

  cr  = getreg32(STM32WB_CONSOLE_BASE + STM32WB_USART_CR3_OFFSET);
  cr &= ~USART_CR3_CLRBITS;
  cr |= USART_CR3_SETBITS;
  putreg32(cr, STM32WB_CONSOLE_BASE + STM32WB_USART_CR3_OFFSET);

  /* Configure the USART Baud Rate */

  putreg32(STM32WB_BRR_VALUE,
           STM32WB_CONSOLE_BASE + STM32WB_USART_BRR_OFFSET);

  /* Select oversampling by 8 */

  cr  = getreg32(STM32WB_CONSOLE_BASE + STM32WB_USART_CR1_OFFSET);
#ifdef USE_OVER8
  cr |= USART_CR1_OVER8;
  putreg32(cr, STM32WB_CONSOLE_BASE + STM32WB_USART_CR1_OFFSET);
#endif

  /* Enable Rx, Tx, and the USART */

  cr |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
  putreg32(cr, STM32WB_CONSOLE_BASE + STM32WB_USART_CR1_OFFSET);

#endif /* HAVE_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
