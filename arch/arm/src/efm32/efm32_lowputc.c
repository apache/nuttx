/****************************************************************************
 * arch/arm/src/efm32/efm32_lowputc.c
 *
 *   Copyright (C) 2014 Richard Cochran. All rights reserved.
 *   Author: Richard Cochran <richardcochran@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "up_internal.h"
#include "efm32_lowputc.h"

#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BAUDRATE 115200

#if defined UART_PRESENT

#if UART_COUNT > 1
#define RXPIN      10
#define RXPORT     gpioPortB
#define TXPIN      9
#define TXPORT     gpioPortB
#define UART       UART1
#define UART_CLOCK cmuClock_UART1
#define UART_LOC   USART_ROUTE_LOCATION_LOC2
#else
#error unknown efm32 part
#endif

#elif defined USART_PRESENT

#define RXPIN      11
#define RXPORT     gpioPortE
#define TXPIN      10
#define TXPORT     gpioPortE
#define UART       USART0
#define UART_CLOCK cmuClock_USART0
#define UART_LOC   USART_ROUTE_LOCATION_LOC0

#endif /* USART_PRESENT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void efm32_lowsetup(void)
{
  USART_InitAsync_TypeDef settings = USART_INITASYNC_DEFAULT;

  CMU_ClockEnable(UART_CLOCK, true);
  GPIO_PinModeSet(TXPORT, TXPIN, gpioModePushPull, 1);
  GPIO_PinModeSet(RXPORT, RXPIN, gpioModeInput, 0);
  settings.baudrate = BAUDRATE;
  USART_InitAsync(UART, &settings);
  UART->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | UART_LOC;
}

void up_lowputc(char c)
{
  while (!(UART->STATUS & USART_STATUS_TXBL));
  UART->TXDATA = c;
}

void up_putc(char c)
{
  /* Convert LF into CRLF. */
  if (c == '\n')
      up_lowputc('\r');

  up_lowputc(c);
}
