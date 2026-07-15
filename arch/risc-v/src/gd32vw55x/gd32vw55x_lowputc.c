/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_lowputc.c
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

#include "riscv_internal.h"
#include "gd32vw55x_config.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_lowputc.h"
#include "hardware/gd32vw55x_rcu.h"
#include "hardware/gd32vw55x_gpio.h"
#include "hardware/gd32vw55x_uart.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select console UART parameters.  Pin mapping comes from board.h
 * (BOARD_xxx_TX_GPIO/PIN/AF, BOARD_xxx_RX_GPIO/PIN/AF).
 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     GD32VW55X_USART0_BASE
#  define CONSOLE_CLOCK    GD32VW55X_PCLK1_FREQ
#  define CONSOLE_BAUD     CONFIG_USART0_BAUD
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     GD32VW55X_UART1_BASE
#  define CONSOLE_CLOCK    GD32VW55X_PCLK1_FREQ
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     GD32VW55X_UART2_BASE
#  define CONSOLE_CLOCK    GD32VW55X_PCLK2_FREQ
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32vw55x_gpio_config_af
 ****************************************************************************/

void gd32vw55x_gpio_config_af(uint32_t port_base, int pin, int af, int pull)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  /* Pin mode: alternate function */

  regaddr = port_base + GD32VW55X_GPIO_CTL_OFFSET;
  shift   = pin << 1;
  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= GPIO_CTL_AF << shift;
  putreg32(regval, regaddr);

  /* Output speed 25 MHz, push-pull (OMODE bit stays 0) */

  regaddr = port_base + GD32VW55X_GPIO_OSPD_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= GPIO_OSPD_25MHZ << shift;
  putreg32(regval, regaddr);

  /* Pull-up/down */

  regaddr = port_base + GD32VW55X_GPIO_PUD_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= pull << shift;
  putreg32(regval, regaddr);

  /* Alternate function number: AFSEL0 for pins 0-7, AFSEL1 for 8-15 */

  if (pin < 8)
    {
      regaddr = port_base + GD32VW55X_GPIO_AFSEL0_OFFSET;
      shift   = pin << 2;
    }
  else
    {
      regaddr = port_base + GD32VW55X_GPIO_AFSEL1_OFFSET;
      shift   = (pin - 8) << 2;
    }

  regval  = getreg32(regaddr);
  regval &= ~(15 << shift);
  regval |= af << shift;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32vw55x_uart_configure
 ****************************************************************************/

void gd32vw55x_uart_configure(uint32_t uart_base, uint32_t uartclk,
                              uint32_t baud)
{
  /* Disable the UART while configuring */

  putreg32(0, uart_base + GD32VW55X_UART_CTL0_OFFSET);

  /* 8N1: WL0/WL1 = 0, no parity, 1 stop bit */

  putreg32(UART_CTL1_STB_1, uart_base + GD32VW55X_UART_CTL1_OFFSET);
  putreg32(0, uart_base + GD32VW55X_UART_CTL2_OFFSET);

  /* Baud rate: oversampling by 16 -> BAUD = uclk / baud */

  putreg32((uartclk + (baud / 2)) / baud,
           uart_base + GD32VW55X_UART_BAUD_OFFSET);

  /* Enable UART, transmitter and receiver */

  putreg32(UART_CTL0_UEN | UART_CTL0_TEN | UART_CTL0_REN,
           uart_base + GD32VW55X_UART_CTL0_OFFSET);
}

/****************************************************************************
 * Name: gd32vw55x_lowsetup
 ****************************************************************************/

void gd32vw55x_lowsetup(void)
{
#ifdef HAVE_UART_DEVICE

  /* Enable GPIO port clocks (A and B cover every UART pin option) */

  modifyreg32(GD32VW55X_RCU_AHB1EN, 0,
              RCU_AHB1EN_PAEN | RCU_AHB1EN_PBEN);

  /* Enable UART peripheral clocks and configure pins for every enabled
   * instance.  Pin assignments come from board.h.
   */

#ifdef CONFIG_GD32VW55X_USART0
  modifyreg32(GD32VW55X_RCU_APB1EN, 0, RCU_APB1EN_USART0EN);
  gd32vw55x_gpio_config_af(BOARD_USART0_TX_GPIO, BOARD_USART0_TX_PIN,
                           BOARD_USART0_TX_AF, GPIO_PUD_PULLUP);
  gd32vw55x_gpio_config_af(BOARD_USART0_RX_GPIO, BOARD_USART0_RX_PIN,
                           BOARD_USART0_RX_AF, GPIO_PUD_PULLUP);
#endif

#ifdef CONFIG_GD32VW55X_UART1
  modifyreg32(GD32VW55X_RCU_APB1EN, 0, RCU_APB1EN_UART1EN);
  gd32vw55x_gpio_config_af(BOARD_UART1_TX_GPIO, BOARD_UART1_TX_PIN,
                           BOARD_UART1_TX_AF, GPIO_PUD_PULLUP);
  gd32vw55x_gpio_config_af(BOARD_UART1_RX_GPIO, BOARD_UART1_RX_PIN,
                           BOARD_UART1_RX_AF, GPIO_PUD_PULLUP);
#endif

#ifdef CONFIG_GD32VW55X_UART2
  modifyreg32(GD32VW55X_RCU_APB2EN, 0, RCU_APB2EN_UART2EN);
  gd32vw55x_gpio_config_af(BOARD_UART2_TX_GPIO, BOARD_UART2_TX_PIN,
                           BOARD_UART2_TX_AF, GPIO_PUD_PULLUP);
  gd32vw55x_gpio_config_af(BOARD_UART2_RX_GPIO, BOARD_UART2_RX_PIN,
                           BOARD_UART2_RX_AF, GPIO_PUD_PULLUP);
#endif

#ifdef HAVE_SERIAL_CONSOLE

  /* Configure and enable the console */

  gd32vw55x_uart_configure(CONSOLE_BASE, CONSOLE_CLOCK, CONSOLE_BAUD);

#endif
#endif /* HAVE_UART_DEVICE */
}

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE

  /* Wait for the transmit data register to be empty */

  while ((getreg32(CONSOLE_BASE + GD32VW55X_UART_STAT_OFFSET) &
          UART_STAT_TBE) == 0)
    {
    }

  putreg32((uint32_t)ch, CONSOLE_BASE + GD32VW55X_UART_TDATA_OFFSET);

#endif /* HAVE_SERIAL_CONSOLE */
}
