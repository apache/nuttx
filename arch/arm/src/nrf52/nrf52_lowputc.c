/****************************************************************************
 * arch/arm/src/rnf52/nrf52_lowputc.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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

#include <stdbool.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/nrf52_memorymap.h"
#include "chip/nrf52_uarte.h"

#include "nrf52_config.h"
#include "nrf52_clockconfig.h"
#include "nrf52_gpio.h"
#include "nrf52_lowputc.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_UART_CONSOLE

#define CONSOLE_BASE     NRF52_UART0_BASE
#define CONSOLE_BAUD     CONFIG_UART0_BAUD
#define CONSOLE_BITS     CONFIG_UART0_BITS
#define CONSOLE_PARITY   CONFIG_UART0_PARITY
#define CONSOLE_2STOP    CONFIG_UART0_2STOP

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UART console configuration */

static const struct uart_config_s g_console_config=
{
  .baud      = CONSOLE_BAUD,
  .parity    = CONSOLE_PARITY,
  .bits      = CONSOLE_BITS,
  .stopbits2 = CONSOLE_2STOP,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .iflow     = CONSOLE_IFLOW,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .oflow     = CONSOLE_OFLOW,
#endif
  .txpin     = BOARD_UART0_TX_PIN,
  .rxpin     = BOARD_UART0_RX_PIN,
};
#endif /* HAVE_UART_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_setbaud
 *
 * Description:
 *   Configure the UART BAUD.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static void nrf52_setbaud(uintptr_t base, const struct uart_config_s *config)
{
  uint32_t br = 0x01D7E000;  /* 268.444444 */

  if (config->baud == 115200)
    {
      br = 0x01D7E000;
    }

  putreg32(br, base + NRF52_UART_BAUDRATE_OFFSET);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console UART.  This UART initialization is done
 *   early so that the serial console is available for debugging very early in
 *   the boot sequence.
 *
 ****************************************************************************/

void nrf52_lowsetup(void)
{
#ifdef HAVE_UART_DEVICE
#ifdef HAVE_UART_CONSOLE
  /* Configure the console UART (if any) */

  nrf52_usart_configure(CONSOLE_BASE, &g_console_config);

#endif /* HAVE_UART_CONSOLE */
#endif /* HAVE_UART_DEVICE */
}

/****************************************************************************
 * Name: nrf52_usart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void nrf52_usart_configure(uintptr_t base, const struct uart_config_s *config)
{
  uint32_t pin;

  putreg32(1, base + NRF52_UART_TASKS_STOPRX_OFFSET);
  putreg32(1, base + NRF52_UART_TASKS_STOPTX_OFFSET);
  putreg32(NRF52_UART_ENABLE_DISABLE, base + NRF52_UART_ENABLE_OFFSET);

  /* Configure baud */

  nrf52_setbaud(base, config);

  /* Config and select pins for uart */

  nrf52_gpio_config(config->txpin);
  nrf52_gpio_config(config->rxpin);

  pin = (config->txpin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  putreg32(pin, base + NRF52_UART_PSELTXD_OFFSET);
  pin = (config->rxpin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  putreg32(pin, base + NRF52_UART_PSELRXD_OFFSET);

  /* Enable */

  putreg32(NRF52_UART_ENABLE_ENABLE, base + NRF52_UART_ENABLE_OFFSET);
}
#endif

/****************************************************************************
 * Name: nrf52_usart_disable
 *
 * Description:
 *   Disable a UART.  it will be necessary to again call
 *   nrf52_usart_configure() in order to use this UART channel again.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void nrf52_usart_disable(uintptr_t base)
{
  /* Disable interrupts */
  /* Disable the UART */

  putreg32(1, base + NRF52_UART_TASKS_STOPRX_OFFSET);
  putreg32(1, base + NRF52_UART_TASKS_STOPTX_OFFSET);
  putreg32(NRF52_UART_ENABLE_DISABLE, base + NRF52_UART_ENABLE_OFFSET);

  putreg32(0xFFFFFFFF, base + NRF52_UART_PSELTXD_OFFSET);
  putreg32(0xFFFFFFFF, base + NRF52_UART_PSELRXD_OFFSET);
}
#endif

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_UART_CONSOLE
  putreg32(1, CONSOLE_BASE + NRF52_UART_TASKS_STARTTX_OFFSET);
  putreg32(0, CONSOLE_BASE + NRF52_UART_EVENTS_TXDRDY_OFFSET);
  putreg32(ch, CONSOLE_BASE + NRF52_UART_TXD_OFFSET);
  while (getreg32(CONSOLE_BASE + NRF52_UART_EVENTS_TXDRDY_OFFSET) == 0 )
    {
    }

  putreg32(1, CONSOLE_BASE + NRF52_UART_TASKS_STOPTX_OFFSET);
#endif
}
