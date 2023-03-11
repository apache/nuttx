/****************************************************************************
 * arch/arm/src/nrf53/nrf53_lowputc.c
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

#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
#include "hardware/nrf53_memorymap.h"
#include "hardware/nrf53_uarte.h"

#include "nrf53_config.h"
#include "nrf53_clockconfig.h"
#include "nrf53_gpio.h"
#include "nrf53_lowputc.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_UART_CONSOLE

#ifdef CONFIG_UART0_SERIAL_CONSOLE
#  define CONSOLE_BASE     NRF53_UART0_BASE
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART0_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART0_RX_PIN
#elif CONFIG_UART1_SERIAL_CONSOLE
#  define CONSOLE_BASE     NRF53_UART1_BASE
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define CONSOLE_2STOP    CONFIG_UART1_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART1_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART1_RX_PIN
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UART console configuration */

static const struct uart_config_s g_console_config =
{
  .baud      = CONSOLE_BAUD,
  .parity    = CONSOLE_PARITY,
  .bits      = CONSOLE_BITS,
  .stopbits2 = CONSOLE_2STOP,
  .txpin     = CONSOLE_TX_PIN,
  .rxpin     = CONSOLE_RX_PIN,
};
#endif /* HAVE_UART_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_setbaud
 *
 * Description:
 *   Configure the UART BAUD.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static void nrf53_setbaud(uintptr_t base, const struct uart_config_s *config)
{
  uint32_t br = 0;

  switch (config->baud)
    {
      case 1200:
        {
          br = UART_BAUDRATE_1200;
          break;
        }

      case 2400:
        {
          br = UART_BAUDRATE_2400;
          break;
        }

      case 4800:
        {
          br = UART_BAUDRATE_4800;
          break;
        }

      case 9600:
        {
          br = UART_BAUDRATE_9600;
          break;
        }

      case 14400:
        {
          br = UART_BAUDRATE_14400;
          break;
        }

      case 19200:
        {
          br = UART_BAUDRATE_19200;
          break;
        }

      case 28800:
        {
          br = UART_BAUDRATE_28800;
          break;
        }

#ifdef UART_BAUDRATE_31250
      case 31250:
        {
          br = UART_BAUDRATE_31250;
          break;
        }
#endif

      case 38400:
        {
          br = UART_BAUDRATE_38400;
          break;
        }

#ifdef UART_BAUDRATE_56000
      case 56000:
        {
          br = UART_BAUDRATE_56000;
          break;
        }
#endif

      case 57600:
        {
          br = UART_BAUDRATE_57600;
          break;
        }

      case 76000:
        {
          br = UART_BAUDRATE_76000;
          break;
        }

      case 115200:
        {
          br = UART_BAUDRATE_115200;
          break;
        }

      case 230400:
        {
          br = UART_BAUDRATE_230400;
          break;
        }

      case 250000:
        {
          br = UART_BAUDRATE_250000;
          break;
        }

      case 460800:
        {
          br = UART_BAUDRATE_460800;
          break;
        }

      case 921600:
        {
          br = UART_BAUDRATE_921600;
          break;
        }

      default:
        {
          DEBUGPANIC();
          break;
        }
    }

  putreg32(br, base + NRF53_UART_BAUDRATE_OFFSET);
}

/****************************************************************************
 * Name: nrf53_setparity
 ****************************************************************************/

static void nrf53_setparity(uintptr_t base,
                            const struct uart_config_s *config)
{
  uint32_t regval = 0;

  regval = getreg32(base + NRF53_UART_CONFIG_OFFSET);

  if (config->parity == 2)
    {
      /* Include even parity */

      regval |= UART_CONFIG_PARITY;
    }
  else
    {
      /* Exclude parity */

      regval &= ~UART_CONFIG_PARITY;
    }

  putreg32(regval, base + NRF53_UART_CONFIG_OFFSET);
}

/****************************************************************************
 * Name: nrf53_setstops
 ****************************************************************************/

#ifdef HAVE_UART_STOPBITS
static void nrf53_setstops(uintptr_t base,
                           const struct uart_config_s *config)
{
  uint32_t regval = 0;

  regval = getreg32(base + NRF53_UART_CONFIG_OFFSET);

  if (config->stopbits2 == true)
    {
      regval |= UART_CONFIG_STOP;
    }
  else
    {
      regval &= ~UART_CONFIG_STOP;
    }

  putreg32(regval, base + NRF53_UART_CONFIG_OFFSET);
}
#endif

/****************************************************************************
 * Name: nrf53_sethwflow
 ****************************************************************************/

static void nrf53_sethwflow(uintptr_t base,
                            const struct uart_config_s *config)
{
  /* TODO */
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start. Performs low level
 *   initialization including setup of the console UART.
 *   This UART initialization is done early so that the serial console is
 *   available for debugging very early in the boot sequence.
 *
 ****************************************************************************/

void nrf53_lowsetup(void)
{
#ifdef HAVE_UART_DEVICE
#ifdef HAVE_UART_CONSOLE
  /* Configure the console UART (if any) */

  nrf53_usart_configure(CONSOLE_BASE, &g_console_config);

#endif /* HAVE_UART_CONSOLE */
#endif /* HAVE_UART_DEVICE */
}

/****************************************************************************
 * Name: nrf53_usart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void nrf53_usart_configure(uintptr_t base,
                           const struct uart_config_s *config)
{
  uint32_t pin    = 0;
  uint32_t port   = 0;
  uint32_t regval = 0;

  putreg32(1, base + NRF53_UART_TASKS_STOPRX_OFFSET);
  putreg32(1, base + NRF53_UART_TASKS_STOPTX_OFFSET);
  putreg32(NRF53_UART_ENABLE_DISABLE, base + NRF53_UART_ENABLE_OFFSET);

  /* Set UART format */

  nrf53_usart_setformat(base, config);

  /* Config GPIO pins for uart */

  nrf53_gpio_config(config->txpin);
  nrf53_gpio_config(config->rxpin);

  /* Setect TX pins for UART */

  pin  = GPIO_PIN_DECODE(config->txpin);
  port = GPIO_PORT_DECODE(config->txpin);

  regval = (pin << UART_PSELTXD_PIN_SHIFT);
  regval |= (port << UART_PSELTXD_PORT_SHIFT);
  putreg32(regval, base + NRF53_UART_PSELTXD_OFFSET);

  /* Setect RX pins for UART */

  pin  = GPIO_PIN_DECODE(config->rxpin);
  port = GPIO_PORT_DECODE(config->rxpin);

  regval = (pin << UART_PSELRXD_PIN_SHIFT);
  regval |= (port << UART_PSELRXD_PORT_SHIFT);
  putreg32(regval, base + NRF53_UART_PSELRXD_OFFSET);

  /* Enable UART */

  putreg32(NRF53_UART_ENABLE_ENABLE, base + NRF53_UART_ENABLE_OFFSET);
}

/****************************************************************************
 * Name: nrf53_usart_disable
 *
 * Description:
 *   Disable a UART.  it will be necessary to again call
 *   nrf53_usart_configure() in order to use this UART channel again.
 *
 ****************************************************************************/

void nrf53_usart_disable(uintptr_t base, const struct uart_config_s *config)
{
  /* Disable interrupts */

  /* Disable the UART */

  putreg32(1, base + NRF53_UART_TASKS_STOPRX_OFFSET);
  putreg32(1, base + NRF53_UART_TASKS_STOPTX_OFFSET);
  putreg32(NRF53_UART_ENABLE_DISABLE, base + NRF53_UART_ENABLE_OFFSET);

  putreg32(0xffffffff, base + NRF53_UART_PSELTXD_OFFSET);
  putreg32(0xffffffff, base + NRF53_UART_PSELRXD_OFFSET);

  /* Unconfigure GPIO */

  nrf53_gpio_unconfig(config->rxpin);
  nrf53_gpio_unconfig(config->txpin);

  /* Deatach TWI from GPIO */

  putreg32(UART_PSELTXD_RESET, base + NRF53_UART_PSELTXD_OFFSET);
  putreg32(UART_PSELRXD_RESET, base + NRF53_UART_PSELRXD_OFFSET);
}

/****************************************************************************
 * Name: nrf53_usart_setformat
 *
 * Description:
 *   Set the USART line format and speed.
 *
 ****************************************************************************/

void nrf53_usart_setformat(uintptr_t base,
                           const struct uart_config_s *config)
{
  /* Configure baud */

  nrf53_setbaud(base, config);

  /* Configure polarity */

  nrf53_setparity(base, config);

#ifdef HAVE_UART_STOPBITS
  /* Configure STOP bits */

  nrf53_setstops(base, config);
#endif

  /* Configure hardware flow control */

  nrf53_sethwflow(base, config);
}
#endif

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_UART_CONSOLE
  putreg32(1, CONSOLE_BASE + NRF53_UART_TASKS_STARTTX_OFFSET);
  putreg32(0, CONSOLE_BASE + NRF53_UART_EVENTS_TXDRDY_OFFSET);
  putreg32(ch, CONSOLE_BASE + NRF53_UART_TXD_OFFSET);
  while (getreg32(CONSOLE_BASE + NRF53_UART_EVENTS_TXDRDY_OFFSET) == 0)
    {
    }

  putreg32(1, CONSOLE_BASE + NRF53_UART_TASKS_STOPTX_OFFSET);
#endif
}
