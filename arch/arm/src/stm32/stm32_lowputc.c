/**************************************************************************
 * arch/arm/src/stm32/stm32_lowputc.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

#include "stm32.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_uart.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/* Select USART parameters for the selected console */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_USART1_BASE
#    define STM32_APBCLOCK         STM32_PCLK2_FREQUENCY
#    define STM32_CONSOLE_APBREG   STM32_RCC_APB2ENR
#    define STM32_CONSOLE_APBEN    RCC_APB2ENR_USART1EN
#    define STM32_CONSOLE_BAUD     CONFIG_USART1_BAUD
#    define STM32_CONSOLE_BITS     CONFIG_USART1_BITS
#    define STM32_CONSOLE_PARITY   CONFIG_USART1_PARITY
#    define STM32_CONSOLE_2STOP    CONFIG_USART1_2STOP
#    define STM32_CONSOLE_TX       GPIO_USART1_TX
#    define STM32_CONSOLE_RX       GPIO_USART1_RX
#    ifdef CONFIG_USART1_RS485
#      define STM32_CONSOLE_RS485_DIR GPIO_USART1_RS485_DIR
#      if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
#        define STM32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_USART2_BASE
#    define STM32_APBCLOCK         STM32_PCLK1_FREQUENCY
#    define STM32_CONSOLE_APBREG   STM32_RCC_APB1ENR
#    define STM32_CONSOLE_APBEN    RCC_APB1ENR_USART2EN
#    define STM32_CONSOLE_BAUD     CONFIG_USART2_BAUD
#    define STM32_CONSOLE_BITS     CONFIG_USART2_BITS
#    define STM32_CONSOLE_PARITY   CONFIG_USART2_PARITY
#    define STM32_CONSOLE_2STOP    CONFIG_USART2_2STOP
#    define STM32_CONSOLE_TX       GPIO_USART2_TX
#    define STM32_CONSOLE_RX       GPIO_USART2_RX
#    ifdef CONFIG_USART2_RS485
#      define STM32_CONSOLE_RS485_DIR GPIO_USART2_RS485_DIR
#      if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
#        define STM32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_USART3_BASE
#    define STM32_APBCLOCK         STM32_PCLK1_FREQUENCY
#    define STM32_CONSOLE_APBREG   STM32_RCC_APB1ENR
#    define STM32_CONSOLE_APBEN    RCC_APB1ENR_USART3EN
#    define STM32_CONSOLE_BAUD     CONFIG_USART3_BAUD
#    define STM32_CONSOLE_BITS     CONFIG_USART3_BITS
#    define STM32_CONSOLE_PARITY   CONFIG_USART3_PARITY
#    define STM32_CONSOLE_2STOP    CONFIG_USART3_2STOP
#    define STM32_CONSOLE_TX       GPIO_USART3_TX
#    define STM32_CONSOLE_RX       GPIO_USART3_RX
#    ifdef CONFIG_USART3_RS485
#      define STM32_CONSOLE_RS485_DIR GPIO_USART3_RS485_DIR
#      if (CONFIG_USART3_RS485_DIR_POLARITY == 0)
#        define STM32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_UART4_BASE
#    define STM32_APBCLOCK         STM32_PCLK1_FREQUENCY
#    define STM32_CONSOLE_APBREG   STM32_RCC_APB1ENR
#    define STM32_CONSOLE_APBEN    RCC_APB1ENR_UART4EN
#    define STM32_CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define STM32_CONSOLE_BITS     CONFIG_UART4_BITS
#    define STM32_CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define STM32_CONSOLE_2STOP    CONFIG_UART4_2STOP
#    define STM32_CONSOLE_TX       GPIO_UART4_TX
#    define STM32_CONSOLE_RX       GPIO_UART4_RX
#    ifdef CONFIG_UART4_RS485
#      define STM32_CONSOLE_RS485_DIR GPIO_UART4_RS485_DIR
#      if (CONFIG_UART4_RS485_DIR_POLARITY == 0)
#        define STM32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_UART5_BASE
#    define STM32_APBCLOCK         STM32_PCLK1_FREQUENCY
#    define STM32_CONSOLE_APBREG   STM32_RCC_APB1ENR
#    define STM32_CONSOLE_APBEN    RCC_APB1ENR_UART5EN
#    define STM32_CONSOLE_BAUD     CONFIG_UART5_BAUD
#    define STM32_CONSOLE_BITS     CONFIG_UART5_BITS
#    define STM32_CONSOLE_PARITY   CONFIG_UART5_PARITY
#    define STM32_CONSOLE_2STOP    CONFIG_UART5_2STOP
#    define STM32_CONSOLE_TX       GPIO_UART5_TX
#    define STM32_CONSOLE_RX       GPIO_UART5_RX
#    ifdef CONFIG_UART5_RS485
#      define STM32_CONSOLE_RS485_DIR GPIO_UART5_RS485_DIR
#      if (CONFIG_UART5_RS485_DIR_POLARITY == 0)
#        define STM32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_USART6_BASE
#    define STM32_APBCLOCK         STM32_PCLK2_FREQUENCY
#    define STM32_CONSOLE_APBREG   STM32_RCC_APB2ENR
#    define STM32_CONSOLE_APBEN    RCC_APB2ENR_USART6EN
#    define STM32_CONSOLE_BAUD     CONFIG_USART6_BAUD
#    define STM32_CONSOLE_BITS     CONFIG_USART6_BITS
#    define STM32_CONSOLE_PARITY   CONFIG_USART6_PARITY
#    define STM32_CONSOLE_2STOP    CONFIG_USART6_2STOP
#    define STM32_CONSOLE_TX       GPIO_USART6_TX
#    define STM32_CONSOLE_RX       GPIO_USART6_RX
#    ifdef CONFIG_USART6_RS485
#      define STM32_CONSOLE_RS485_DIR GPIO_USART6_RS485_DIR
#      if (CONFIG_USART6_RS485_DIR_POLARITY == 0)
#        define STM32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_UART7_BASE
#    define STM32_APBCLOCK         STM32_PCLK1_FREQUENCY
#    define STM32_CONSOLE_APBREG   STM32_RCC_APB1ENR
#    define STM32_CONSOLE_APBEN    RCC_APB1ENR_UART7EN
#    define STM32_CONSOLE_BAUD     CONFIG_UART7_BAUD
#    define STM32_CONSOLE_BITS     CONFIG_UART7_BITS
#    define STM32_CONSOLE_PARITY   CONFIG_UART7_PARITY
#    define STM32_CONSOLE_2STOP    CONFIG_UART7_2STOP
#    define STM32_CONSOLE_TX       GPIO_UART7_TX
#    define STM32_CONSOLE_RX       GPIO_UART7_RX
#    ifdef CONFIG_UART7_RS485
#      define STM32_CONSOLE_RS485_DIR GPIO_UART7_RS485_DIR
#      if (CONFIG_UART7_RS485_DIR_POLARITY == 0)
#        define STM32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART8_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_UART8_BASE
#    define STM32_APBCLOCK         STM32_PCLK1_FREQUENCY
#    define STM32_CONSOLE_APBREG   STM32_RCC_APB1ENR
#    define STM32_CONSOLE_APBEN    RCC_APB1ENR_UART8EN
#    define STM32_CONSOLE_BAUD     CONFIG_UART8_BAUD
#    define STM32_CONSOLE_BITS     CONFIG_UART8_BITS
#    define STM32_CONSOLE_PARITY   CONFIG_UART8_PARITY
#    define STM32_CONSOLE_2STOP    CONFIG_UART8_2STOP
#    define STM32_CONSOLE_TX       GPIO_UART8_TX
#    define STM32_CONSOLE_RX       GPIO_UART8_RX
#    ifdef CONFIG_UART8_RS485
#      define STM32_CONSOLE_RS485_DIR GPIO_UART8_RS485_DIR
#      if (CONFIG_UART8_RS485_DIR_POLARITY == 0)
#        define STM32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define STM32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  endif

  /* CR1 settings */

#  if STM32_CONSOLE_BITS == 9
#    define USART_CR1_M_VALUE USART_CR1_M
#  else
#    define USART_CR1_M_VALUE 0
#  endif

#  if STM32_CONSOLE_PARITY == 1
#    define USART_CR1_PARITY_VALUE (USART_CR1_PCE|USART_CR1_PS)
#  elif STM32_CONSOLE_PARITY == 2
#    define USART_CR1_PARITY_VALUE USART_CR1_PCE
#  else
#    define USART_CR1_PARITY_VALUE 0
#  endif

#  if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
#    define USART_CR1_CLRBITS\
      (USART_CR1_UESM | USART_CR1_RE | USART_CR1_TE | USART_CR1_PS |\
       USART_CR1_PCE |USART_CR1_WAKE | USART_CR1_M | USART_CR1_MME |\
       USART_CR1_OVER8 | USART_CR1_DEDT_MASK | USART_CR1_DEAT_MASK |\
       USART_CR1_ALLINTS)
#  else
#    define USART_CR1_CLRBITS\
      (USART_CR1_M | USART_CR1_PCE |USART_CR1_PS | USART_CR1_TE |\
       USART_CR1_RE | USART_CR1_ALLINTS)
#  endif

#  define USART_CR1_SETBITS (USART_CR1_M_VALUE|USART_CR1_PARITY_VALUE)

  /* CR2 settings */

#  if STM32_CONSOLE_2STOP != 0
#    define USART_CR2_STOP2_VALUE USART_CR2_STOP2
#  else
#    define USART_CR2_STOP2_VALUE 0
#  endif

#  if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
#    define USART_CR2_CLRBITS \
      (USART_CR2_ADDM7 | USART_CR2_LBDL | USART_CR2_LBDIE | USART_CR2_LBCL |\
       USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN | USART_CR2_STOP_MASK |\
       USART_CR2_LINEN | USART_CR2_RXINV | USART_CR2_TXINV | USART_CR2_DATAINV |\
       USART_CR2_MSBFIRST | USART_CR2_ABREN | USART_CR2_ABRMOD_MASK |\
       USART_CR2_RTOEN | USART_CR2_ADD8_MASK)
#  else
#    define USART_CR2_CLRBITS \
       (USART_CR2_STOP_MASK|USART_CR2_CLKEN|USART_CR2_CPOL|USART_CR2_CPHA|\
       USART_CR2_LBCL|USART_CR2_LBDIE)
#  endif
#  define USART_CR2_SETBITS USART_CR2_STOP2_VALUE

  /* CR3 settings */

#  if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)

#    define USART_CR3_CLRBITS \
      (USART_CR3_EIE | USART_CR3_IREN | USART_CR3_IRLP | USART_CR3_HDSEL |\
       USART_CR3_NACK | USART_CR3_SCEN | USART_CR3_DMAR | USART_CR3_DMAT |\
       USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_CTSIE | USART_CR1_ONEBIT |\
       USART_CR1_OVRDIS | USART_CR1_DDRE | USART_CR1_DEM | USART_CR1_DEP |\
       USART_CR1_SCARCNT_MASK | USART_CR1_WUS_MASK | USART_CR1_WUFIE)
#  else
#    define USART_CR3_CLRBITS \
      (USART_CR3_CTSIE|USART_CR3_CTSE|USART_CR3_RTSE|USART_CR3_EIE)
#  endif
#  define USART_CR3_SETBITS 0

  /* Only the STM32 F3 supports oversampling by 8 */

#  undef USE_OVER8

  /* Calculate USART BAUD rate divider */

#  if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)

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

#    define STM32_USARTDIV8 \
      (((STM32_APBCLOCK << 1) + (STM32_CONSOLE_BAUD >> 1)) / STM32_CONSOLE_BAUD)
#    define STM32_USARTDIV16 \
      ((STM32_APBCLOCK + (STM32_CONSOLE_BAUD >> 1)) / STM32_CONSOLE_BAUD)

     /* Use oversamply by 8 only if the divisor is small.  But what is small? */

#    if STM32_USARTDIV8 > 100
#      define STM32_BRR_VALUE STM32_USARTDIV16
#    else
#      define USE_OVER8 1
#      define STM32_BRR_VALUE \
        ((STM32_USARTDIV8 & 0xfff0) | ((STM32_USARTDIV8 & 0x000f) >> 1))
#    endif

#  else /* CONFIG_STM32_STM32F30XX */

    /* The baud rate for the receiver and transmitter (Rx and Tx) are both set
     * to the same value as programmed in the Mantissa and Fraction values of
     * USARTDIV.
     *
     *   baud     = fCK / (16 * usartdiv)
     *   usartdiv = fCK / (16 * baud)
     *
     * Where fCK is the input clock to the peripheral (PCLK1 for USART2, 3, 4,
     * 5 or PCLK2 for USART1).  Example, fCK=72MHz baud=115200,
     * usartdiv=39.0625=39 1/16th;
     *
     * First calculate:
     *
     *   usartdiv32 = 32 * usartdiv = fCK / (baud/2)
     *
     * (NOTE: all standard baud values are even so dividing by two does not
     * lose precision).  Eg. (same fCK and buad), usartdiv32 = 1250
     */

#    define STM32_USARTDIV32 (STM32_APBCLOCK / (STM32_CONSOLE_BAUD >> 1))

    /* The mantissa is then usartdiv32 / 32:
     *
     *   mantissa = usartdiv32 / 32/
     *
     * Eg. usartdiv32=1250, mantissa = 39
     */

#    define STM32_MANTISSA (STM32_USARTDIV32 >> 5)

    /* And the fraction:
     *
     *  fraction = (usartdiv32 - mantissa*32 + 1) / 2
     *
     * Eg., (1,250 - 39*32 + 1)/2 = 1 (or 0.0625)
     */

#    define STM32_FRACTION \
      ((STM32_USARTDIV32 - (STM32_MANTISSA << 5) + 1) >> 1)

    /* And, finally, the BRR value is: */

#    define STM32_BRR_VALUE \
      ((STM32_MANTISSA << USART_BRR_MANT_SHIFT) | \
       (STM32_FRACTION << USART_BRR_FRAC_SHIFT))

#  endif /* CONFIG_STM32_STM32F30XX */
#endif /* HAVE_CONSOLE */

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 **************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  /* Wait until the TX data register is empty */

  while ((getreg32(STM32_CONSOLE_BASE + STM32_USART_SR_OFFSET) & USART_SR_TXE) == 0);
#if STM32_CONSOLE_RS485_DIR
  stm32_gpiowrite(STM32_CONSOLE_RS485_DIR, STM32_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Then send the character */

  putreg32((uint32_t)ch, STM32_CONSOLE_BASE + STM32_USART_TDR_OFFSET);

#if STM32_CONSOLE_RS485_DIR
  while ((getreg32(STM32_CONSOLE_BASE + STM32_USART_SR_OFFSET) & USART_SR_TC) == 0);
  stm32_gpiowrite(STM32_CONSOLE_RS485_DIR, !STM32_CONSOLE_RS485_DIR_POLARITY);
#endif

#endif /* HAVE_CONSOLE */
}

/**************************************************************************
 * Name: stm32_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

#if defined(CONFIG_STM32_STM32F10XX)

void stm32_lowsetup(void)
{
#if defined(HAVE_UART)
  uint32_t mapr;
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t cr;
#endif

  /* Set up the pin mapping registers for the selected U[S]ARTs.
   *
   * NOTE: Clocking for selected U[S]ARTs was already provided in stm32_rcc.c
   */

  mapr = getreg32(STM32_AFIO_MAPR);

#ifdef CONFIG_STM32_USART1
  /* Assume default pin mapping:
   *
   *   Alternate  USART1_REMAP USART1_REMAP
   *   Function   = 0          = 1
   *   ---------- ------------ ------------
   *   USART1_TX  PA9          PB6
   *   USART1_RX  PA10         PB7
   */

#ifdef CONFIG_STM32_USART1_REMAP
  mapr |= AFIO_MAPR_USART1_REMAP;
#else
  mapr &= ~AFIO_MAPR_USART1_REMAP;
#endif
#endif /* CONFIG_STM32_USART1 */

#ifdef CONFIG_STM32_USART2
  /* Assume default pin mapping:
   *
   *   Alternate  USART2_REMAP USART2_REMAP
   *   Function   = 0          = 1
   *   ---------- ------------ ------------
   *   USART2_CTS PA0          PD3
   *   USART2_RTS PA1          PD4
   *   USART2_TX  PA2          PD5
   *   USART2_RX  PA3          PD6
   *   USART3_CK  PA4          PD7
   */

#ifdef CONFIG_STM32_USART2_REMAP
  mapr |= AFIO_MAPR_USART2_REMAP;
#else
  mapr &= ~AFIO_MAPR_USART2_REMAP;
#endif
#endif /* CONFIG_STM32_USART2 */

  /* Assume default pin mapping:
   *
   * Alternate  USART3_REMAP[1:0]  USART3_REMAP[1:0]      USART3_REMAP[1:0]
   * Function   = “00” (no remap)  = “01” (partial remap) = “11” (full remap)
   * ---------_ ------------------ ---------------------- --------------------
   * USART3_TX  PB10               PC10                   PD8
   * USART3_RX  PB11               PC11                   PD9
   * USART3_CK  PB12               PC12                   PD10
   * USART3_CTS PB13               PB13                   PD11
   * USART3_RTS PB14               PB14                   PD12
   */

  mapr &= ~AFIO_MAPR_USART3_REMAP_MASK;

#ifdef CONFIG_STM32_USART3
#if defined(CONFIG_STM32_USART3_PARTIAL_REMAP)
  mapr |= AFIO_MAPR_USART3_PARTREMAP;
#elif defined(CONFIG_STM32_USART3_FULL_REMAP)
  mapr |= AFIO_MAPR_USART3_FULLREMAP;
#endif
#endif /* CONFIG_STM32_USART3 */

  putreg32(mapr, STM32_AFIO_MAPR);

  /* Configure GPIO pins needed for rx/tx. */

#ifdef STM32_CONSOLE_TX
  stm32_configgpio(STM32_CONSOLE_TX);
#endif
#ifdef STM32_CONSOLE_RX
  stm32_configgpio(STM32_CONSOLE_RX);
#endif

#if STM32_CONSOLE_RS485_DIR
  stm32_configgpio(STM32_CONSOLE_RS485_DIR);
  stm32_gpiowrite(STM32_CONSOLE_RS485_DIR, !STM32_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Enable and configure the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Configure CR2 */

  cr  = getreg32(STM32_CONSOLE_BASE + STM32_USART_CR2_OFFSET);
  cr &= ~USART_CR2_CLRBITS;
  cr |= USART_CR2_SETBITS;
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR2_OFFSET);

  /* Configure CR1 */

  cr  = getreg32(STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
  cr &= ~USART_CR1_CLRBITS;
  cr |= USART_CR1_SETBITS;
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);

  /* Configure CR3 */

  cr  = getreg32(STM32_CONSOLE_BASE + STM32_USART_CR3_OFFSET);
  cr &= ~USART_CR3_CLRBITS;
  cr |= USART_CR3_SETBITS;
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR3_OFFSET);

  /* Configure the USART Baud Rate */

  putreg32(STM32_BRR_VALUE, STM32_CONSOLE_BASE + STM32_USART_BRR_OFFSET);

  /* Enable Rx, Tx, and the USART */

  cr  = getreg32(STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
  cr |= (USART_CR1_UE|USART_CR1_TE|USART_CR1_RE);
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);

#endif /* HAVE_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}

#elif defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F20XX) || \
      defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX) || \
      defined(CONFIG_STM32_STM32F40XX)

void stm32_lowsetup(void)
{
#if defined(HAVE_UART)
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t cr;
#endif

#if defined(HAVE_CONSOLE)
  /* Enable USART APB1/2 clock */

  modifyreg32(STM32_CONSOLE_APBREG, 0, STM32_CONSOLE_APBEN);
#endif

  /* Enable the console USART and configure GPIO pins needed for rx/tx.
   *
   * NOTE: Clocking for selected U[S]ARTs was already provided in stm32_rcc.c
   */

#ifdef STM32_CONSOLE_TX
  stm32_configgpio(STM32_CONSOLE_TX);
#endif
#ifdef STM32_CONSOLE_RX
  stm32_configgpio(STM32_CONSOLE_RX);
#endif

#if STM32_CONSOLE_RS485_DIR
  stm32_configgpio(STM32_CONSOLE_RS485_DIR);
  stm32_gpiowrite(STM32_CONSOLE_RS485_DIR, !STM32_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Enable and configure the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Configure CR2 */

  cr  = getreg32(STM32_CONSOLE_BASE + STM32_USART_CR2_OFFSET);
  cr &= ~USART_CR2_CLRBITS;
  cr |= USART_CR2_SETBITS;
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR2_OFFSET);

  /* Configure CR1 */

  cr  = getreg32(STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
  cr &= ~USART_CR1_CLRBITS;
  cr |= USART_CR1_SETBITS;
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);

  /* Configure CR3 */

  cr  = getreg32(STM32_CONSOLE_BASE + STM32_USART_CR3_OFFSET);
  cr &= ~USART_CR3_CLRBITS;
  cr |= USART_CR3_SETBITS;
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR3_OFFSET);

  /* Configure the USART Baud Rate */

  putreg32(STM32_BRR_VALUE, STM32_CONSOLE_BASE + STM32_USART_BRR_OFFSET);

  /* Select oversampling by 8 */

  cr  = getreg32(STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
#ifdef USE_OVER8
  cr |= USART_CR1_OVER8;
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
#endif

  /* Enable Rx, Tx, and the USART */

  cr |= (USART_CR1_UE|USART_CR1_TE|USART_CR1_RE);
  putreg32(cr, STM32_CONSOLE_BASE + STM32_USART_CR1_OFFSET);

#endif /* HAVE_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}

#else
#  error "Unsupported STM32 chip"
#endif
