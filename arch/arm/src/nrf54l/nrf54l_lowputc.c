/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_lowputc.c
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

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "arm_internal.h"
#include "hardware/nrf54l_memorymap.h"
#include "hardware/nrf54l_uarte.h"

#include "nrf54l_config.h"
#include "nrf54l_clockconfig.h"
#include "nrf54l_gpio.h"
#include "nrf54l_lowputc.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_UART_CONSOLE

#ifdef CONFIG_UART0_SERIAL_CONSOLE
#  define CONSOLE_BASE     NRF54L_UART0_BASE
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART0_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART0_RX_PIN
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     NRF54L_UART1_BASE
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define CONSOLE_2STOP    CONFIG_UART1_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART1_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART1_RX_PIN
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     NRF54L_UART2_BASE
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define CONSOLE_BITS     CONFIG_UART2_BITS
#  define CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define CONSOLE_2STOP    CONFIG_UART2_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART2_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART2_RX_PIN
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_BASE     NRF54L_UART3_BASE
#  define CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define CONSOLE_BITS     CONFIG_UART3_BITS
#  define CONSOLE_PARITY   CONFIG_UART3_PARITY
#  define CONSOLE_2STOP    CONFIG_UART3_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART3_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART3_RX_PIN
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define CONSOLE_BASE     NRF54L_UART4_BASE
#  define CONSOLE_BAUD     CONFIG_UART4_BAUD
#  define CONSOLE_BITS     CONFIG_UART4_BITS
#  define CONSOLE_PARITY   CONFIG_UART4_PARITY
#  define CONSOLE_2STOP    CONFIG_UART4_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART4_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART4_RX_PIN
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define CONSOLE_BASE     NRF54L_UART5_BASE
#  define CONSOLE_BAUD     CONFIG_UART5_BAUD
#  define CONSOLE_BITS     CONFIG_UART5_BITS
#  define CONSOLE_PARITY   CONFIG_UART5_PARITY
#  define CONSOLE_2STOP    CONFIG_UART5_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART5_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART5_RX_PIN
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#  define CONSOLE_BASE     NRF54L_UART6_BASE
#  define CONSOLE_BAUD     CONFIG_UART6_BAUD
#  define CONSOLE_BITS     CONFIG_UART6_BITS
#  define CONSOLE_PARITY   CONFIG_UART6_PARITY
#  define CONSOLE_2STOP    CONFIG_UART6_2STOP
#  define CONSOLE_TX_PIN   BOARD_UART6_TX_PIN
#  define CONSOLE_RX_PIN   BOARD_UART6_RX_PIN
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

/* EasyDMA requires its source byte to be in RAM. */

static uint8_t g_console_tx;
#endif /* HAVE_UART_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_setbaud
 *
 * Description:
 *   Configure the UART BAUD.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static int nrf54l_setbaud(uintptr_t base, const struct uart_config_s *config)
{
  uint32_t br = 0;

  switch (config->baud)
    {
      case 1200:
        {
          br = UARTE_BAUDRATE_1200;
          break;
        }

      case 2400:
        {
          br = UARTE_BAUDRATE_2400;
          break;
        }

      case 4800:
        {
          br = UARTE_BAUDRATE_4800;
          break;
        }

      case 9600:
        {
          br = UARTE_BAUDRATE_9600;
          break;
        }

      case 14400:
        {
          br = UARTE_BAUDRATE_14400;
          break;
        }

      case 19200:
        {
          br = UARTE_BAUDRATE_19200;
          break;
        }

      case 28800:
        {
          br = UARTE_BAUDRATE_28800;
          break;
        }

      case 31250:
        {
          br = UARTE_BAUDRATE_31250;
          break;
        }

      case 38400:
        {
          br = UARTE_BAUDRATE_38400;
          break;
        }

      case 56000:
        {
          br = UARTE_BAUDRATE_56000;
          break;
        }

      case 57600:
        {
          br = UARTE_BAUDRATE_57600;
          break;
        }

      case 76800:
        {
          br = UARTE_BAUDRATE_76800;
          break;
        }

      case 115200:
        {
          br = UARTE_BAUDRATE_115200;
          break;
        }

      case 230400:
        {
          br = UARTE_BAUDRATE_230400;
          break;
        }

      case 250000:
        {
          br = UARTE_BAUDRATE_250000;
          break;
        }

      case 460800:
        {
          br = UARTE_BAUDRATE_460800;
          break;
        }

      case 921600:
        {
          br = UARTE_BAUDRATE_921600;
          break;
        }

      case 1000000:
        {
          br = UARTE_BAUDRATE_1000000;
          break;
        }

      default:
        {
          return -EINVAL;
        }
    }

  /* UARTE00 runs from the 128 MHz peripheral clock. The baud generator
   * ignores the low 12 bits, so round after scaling the 16 MHz value.
   */

  if (base == NRF54L_UART4_BASE)
    {
      br = ((br / 8) + 0x800) & ~0xfff;
    }

  putreg32(br, base + NRF54L_UARTE_BAUDRATE_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_setparity
 ****************************************************************************/

static void nrf54l_setparity(uintptr_t base,
                            const struct uart_config_s *config)
{
  uint32_t regval = 0;

  regval = getreg32(base + NRF54L_UARTE_CONFIG_OFFSET);

  regval &= ~(UARTE_CONFIG_PARITY_MASK | UARTE_CONFIG_PARITYTYPE);

  if (config->parity != 0)
    {
      /* Include the selected parity */

      regval |= UARTE_CONFIG_PARITY_INCLUDED;
      if (config->parity == 1)
        {
          regval |= UARTE_CONFIG_PARITYTYPE;
        }
    }

  putreg32(regval, base + NRF54L_UARTE_CONFIG_OFFSET);
}

/****************************************************************************
 * Name: nrf54l_setstops
 ****************************************************************************/

static void nrf54l_setstops(uintptr_t base,
                            const struct uart_config_s *config)
{
  uint32_t regval = 0;

  regval = getreg32(base + NRF54L_UARTE_CONFIG_OFFSET);

  if (config->stopbits2 == true)
    {
      regval |= UARTE_CONFIG_STOP;
    }
  else
    {
      regval &= ~UARTE_CONFIG_STOP;
    }

  putreg32(regval, base + NRF54L_UARTE_CONFIG_OFFSET);
}

/****************************************************************************
 * Name: nrf54l_sethwflow
 ****************************************************************************/

static void nrf54l_sethwflow(uintptr_t base,
                             const struct uart_config_s *config)
{
  modifyreg32(base + NRF54L_UARTE_CONFIG_OFFSET, UARTE_CONFIG_HWFC, 0);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start. Performs low level
 *   initialization including setup of the console UART.
 *   This UART initialization is done early so that the serial console is
 *   available for debugging very early in the boot sequence.
 *
 ****************************************************************************/

void nrf54l_lowsetup(void)
{
#ifdef HAVE_UART_DEVICE
#if defined(HAVE_UART_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Configure the console UART (if any) */

  nrf54l_usart_configure(CONSOLE_BASE, &g_console_config);

#endif /* HAVE_UART_CONSOLE */
#endif /* HAVE_UART_DEVICE */
}

/****************************************************************************
 * Name: nrf54l_usart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
int nrf54l_usart_configure(uintptr_t base,
                           const struct uart_config_s *config)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t pin    = 0;
  uint32_t port   = 0;
  uint32_t regval = 0;
  int ret;

  putreg32(0xffffffff, base + NRF54L_UARTE_INTENCLR_OFFSET);
  putreg32(0, base + NRF54L_UARTE_ENABLE_OFFSET);

  /* Set UART format */

  ret = nrf54l_usart_setformat(base, config);
  if (ret < 0)
    {
      return ret;
    }

  /* Config GPIO pins for uart */

  ret = nrf54l_gpio_config(config->txpin);
  if (ret < 0)
    {
      return ret;
    }

  ret = nrf54l_gpio_config(config->rxpin);
  if (ret < 0)
    {
      nrf54l_gpio_unconfig(config->txpin);
      return ret;
    }

  /* Select TX pins for UART */

  pin  = GPIO_PIN_DECODE(config->txpin);
  port = GPIO_PORT_DECODE(config->txpin);

  regval = (pin << UARTE_PSEL_TXD_PIN_SHIFT);
  regval |= (port << UARTE_PSEL_TXD_PORT_SHIFT);
  putreg32(regval, base + NRF54L_UARTE_PSEL_TXD_OFFSET);

  /* Select RX pins for UART */

  pin  = GPIO_PIN_DECODE(config->rxpin);
  port = GPIO_PORT_DECODE(config->rxpin);

  regval = (pin << UARTE_PSEL_RXD_PIN_SHIFT);
  regval |= (port << UARTE_PSEL_RXD_PORT_SHIFT);
  putreg32(regval, base + NRF54L_UARTE_PSEL_RXD_OFFSET);
  putreg32(0xffffffff, base + NRF54L_UARTE_PSEL_CTS_OFFSET);
  putreg32(0xffffffff, base + NRF54L_UARTE_PSEL_RTS_OFFSET);

  /* Stop each DMA transmission after the last byte leaves the UART. */

  putreg32(UARTE_SHORTS_DMA_TX_END_STOP, base + NRF54L_UARTE_SHORTS_OFFSET);

  /* Enable UART */

  putreg32(UARTE_ENABLE_ENABLE, base + NRF54L_UARTE_ENABLE_OFFSET);
#endif
  return OK;
}

/****************************************************************************
 * Name: nrf54l_usart_disable
 *
 * Description:
 *   Disable a UART.  it will be necessary to again call
 *   nrf54l_usart_configure() in order to use this UART channel again.
 *
 ****************************************************************************/

void nrf54l_usart_disable(uintptr_t base, const struct uart_config_s *config)
{
  /* Disable the UART */

  putreg32(0, base + NRF54L_UARTE_ENABLE_OFFSET);

  putreg32(0xffffffff, base + NRF54L_UARTE_PSEL_TXD_OFFSET);
  putreg32(0xffffffff, base + NRF54L_UARTE_PSEL_RXD_OFFSET);

  /* Unconfigure GPIO */

  nrf54l_gpio_unconfig(config->rxpin);
  nrf54l_gpio_unconfig(config->txpin);
}

/****************************************************************************
 * Name: nrf54l_usart_setformat
 *
 * Description:
 *   Set the USART line format and speed.
 *
 ****************************************************************************/

int nrf54l_usart_setformat(uintptr_t base,
                           const struct uart_config_s *config)
{
  int ret;

  if (config->bits < 5 || config->bits > 8 || config->parity > 2)
    {
      return -EINVAL;
    }

  /* Configure baud */

  ret = nrf54l_setbaud(base, config);
  if (ret < 0)
    {
      return ret;
    }

  modifyreg32(base + NRF54L_UARTE_CONFIG_OFFSET,
              UARTE_CONFIG_FRAMESIZE_MASK,
              config->bits << UARTE_CONFIG_FRAMESIZE_SHIFT);

  /* Configure polarity */

  nrf54l_setparity(base, config);

  /* Configure STOP bits */

  nrf54l_setstops(base, config);

  /* Configure hardware flow control */

  nrf54l_sethwflow(base, config);
  return OK;
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
  g_console_tx = ch;

  putreg32(0, CONSOLE_BASE + NRF54L_UARTE_EVENTS_TXSTOPPED_OFFSET);
  (void)getreg32(CONSOLE_BASE + NRF54L_UARTE_EVENTS_TXSTOPPED_OFFSET);

  putreg32(0, CONSOLE_BASE + NRF54L_UARTE_EVENTS_DMA_TX_END_OFFSET);
  (void)getreg32(CONSOLE_BASE + NRF54L_UARTE_EVENTS_DMA_TX_END_OFFSET);

  putreg32((uintptr_t)&g_console_tx,
           CONSOLE_BASE + NRF54L_UARTE_DMA_TX_PTR_OFFSET);

  putreg32(1, CONSOLE_BASE + NRF54L_UARTE_DMA_TX_MAXCNT_OFFSET);

  UP_DMB();

  putreg32(1, CONSOLE_BASE + NRF54L_UARTE_TASKS_STARTTX_OFFSET);

  while (getreg32(CONSOLE_BASE + NRF54L_UARTE_EVENTS_TXSTOPPED_OFFSET) == 0)
    {
    }

#endif
}
