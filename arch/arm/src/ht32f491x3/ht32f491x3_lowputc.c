/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_lowputc.c
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

#include <nuttx/compiler.h>

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "ht32f491x3_config.h"
#include "ht32f491x3_gpio.h"
#include "ht32f491x3_lowputc.h"
#include "ht32f491x3_serial.h"

#include "hardware/ht32f491x3_crm.h"
#include "hardware/ht32f491x3_gpio.h"
#include "hardware/ht32f491x3_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART1_SERIAL_CONSOLE) && \
      defined(CONFIG_HT32F491X3_USART1_SERIALDRIVER)
#    define HT32_CONSOLE_BASE      HT32_USART1_BASE
#    define HT32_CONSOLE_APBREG    HT32_CRM_APB2EN
#    define HT32_CONSOLE_APBEN     HT32_CRM_APB2EN_USART1EN
#    define HT32_CONSOLE_CLOCK     HT32_PCLK2_FREQUENCY
#    define HT32_CONSOLE_BAUD      CONFIG_USART1_BAUD
#    define HT32_CONSOLE_BITS      CONFIG_USART1_BITS
#    define HT32_CONSOLE_PARITY    CONFIG_USART1_PARITY
#    define HT32_CONSOLE_2STOP     CONFIG_USART1_2STOP
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE) && \
        defined(CONFIG_HT32F491X3_USART2_SERIALDRIVER)
#    define HT32_CONSOLE_BASE      HT32_USART2_BASE
#    define HT32_CONSOLE_APBREG    HT32_CRM_APB1EN
#    define HT32_CONSOLE_APBEN     HT32_CRM_APB1EN_USART2EN
#    define HT32_CONSOLE_CLOCK     HT32_PCLK1_FREQUENCY
#    define HT32_CONSOLE_BAUD      CONFIG_USART2_BAUD
#    define HT32_CONSOLE_BITS      CONFIG_USART2_BITS
#    define HT32_CONSOLE_PARITY    CONFIG_USART2_PARITY
#    define HT32_CONSOLE_2STOP     CONFIG_USART2_2STOP
#  elif defined(CONFIG_USART3_SERIAL_CONSOLE) && \
        defined(CONFIG_HT32F491X3_USART3_SERIALDRIVER)
#    define HT32_CONSOLE_BASE      HT32_USART3_BASE
#    define HT32_CONSOLE_APBREG    HT32_CRM_APB1EN
#    define HT32_CONSOLE_APBEN     HT32_CRM_APB1EN_USART3EN
#    define HT32_CONSOLE_CLOCK     HT32_PCLK1_FREQUENCY
#    define HT32_CONSOLE_BAUD      CONFIG_USART3_BAUD
#    define HT32_CONSOLE_BITS      CONFIG_USART3_BITS
#    define HT32_CONSOLE_PARITY    CONFIG_USART3_PARITY
#    define HT32_CONSOLE_2STOP     CONFIG_USART3_2STOP
#  endif

#  define HT32_USART_CTRL1_CLRBITS \
     (HT32_USART_CTRL1_SBF | HT32_USART_CTRL1_RM | HT32_USART_CTRL1_REN | \
      HT32_USART_CTRL1_TEN | HT32_USART_CTRL1_IDLEIEN | \
      HT32_USART_CTRL1_RDBFIEN | HT32_USART_CTRL1_TDCIEN | \
      HT32_USART_CTRL1_TDBEIEN | HT32_USART_CTRL1_PERRIEN | \
      HT32_USART_CTRL1_PSEL | HT32_USART_CTRL1_PEN | \
      HT32_USART_CTRL1_WUM | HT32_USART_CTRL1_DBN0 | \
      HT32_USART_CTRL1_UEN | HT32_USART_CTRL1_DBN1)

#  define HT32_USART_CTRL2_CLRBITS \
     (HT32_USART_CTRL2_BFIEN | HT32_USART_CTRL2_LBCP | \
      HT32_USART_CTRL2_CLKPHA | HT32_USART_CTRL2_CLKPOL | \
      HT32_USART_CTRL2_CLKEN | HT32_USART_CTRL2_STOPBN_MASK)

#  define HT32_USART_CTRL3_CLRBITS \
     (HT32_USART_CTRL3_ERRIEN | HT32_USART_CTRL3_DMAREN | \
      HT32_USART_CTRL3_DMATEN | HT32_USART_CTRL3_RTSEN | \
      HT32_USART_CTRL3_CTSEN | HT32_USART_CTRL3_CTSCFIEN | \
      HT32_USART_CTRL3_RS485EN | HT32_USART_CTRL3_DEP)
#endif

static uint32_t ht32f491x3_bauddiv(uint32_t clock, uint32_t baud)
{
  uint32_t div;

  div = (clock * 10u) / baud;
  return ((div % 10u) < 5u) ? (div / 10u) : (div / 10u + 1u);
}

void ht32f491x3_usart_config(uintptr_t uartbase)
{
  if (uartbase == HT32_USART1_BASE)
    {
      modifyreg32(HT32_CRM_PICLKS,
                  HT32_CRM_PICLKS_USART1SEL_MASK,
                  HT32_CRM_PICLKS_USARTSEL_PCLK <<
                  HT32_CRM_PICLKS_USART1SEL_SHIFT);
      modifyreg32(HT32_CRM_APB2RST, 0, HT32_CRM_APB2RST_USART1RST);
      modifyreg32(HT32_CRM_APB2RST, HT32_CRM_APB2RST_USART1RST, 0);
    }
  else if (uartbase == HT32_USART2_BASE)
    {
      modifyreg32(HT32_CRM_PICLKS,
                  HT32_CRM_PICLKS_USART2SEL_MASK,
                  HT32_CRM_PICLKS_USARTSEL_PCLK <<
                  HT32_CRM_PICLKS_USART2SEL_SHIFT);
      modifyreg32(HT32_CRM_APB1RST, 0, HT32_CRM_APB1RST_USART2RST);
      modifyreg32(HT32_CRM_APB1RST, HT32_CRM_APB1RST_USART2RST, 0);
    }
  else if (uartbase == HT32_USART3_BASE)
    {
      modifyreg32(HT32_CRM_PICLKS,
                  HT32_CRM_PICLKS_USART3SEL_MASK,
                  HT32_CRM_PICLKS_USARTSEL_PCLK <<
                  HT32_CRM_PICLKS_USART3SEL_SHIFT);
      modifyreg32(HT32_CRM_APB1RST, 0, HT32_CRM_APB1RST_USART3RST);
      modifyreg32(HT32_CRM_APB1RST, HT32_CRM_APB1RST_USART3RST, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Configure the board-specific pinmux for the selected USART. The physical
 * mapping comes from BOARD_USARTx_* macros in <arch/board/board.h>.
 */

void ht32f491x3_usart_pins(uintptr_t uartbase)
{
#ifdef BOARD_USART1_GPIO_CLKEN
  if (uartbase == HT32_USART1_BASE)
    {
      modifyreg32(HT32_CRM_AHBEN1, 0, BOARD_USART1_GPIO_CLKEN);
      ht32f491x3_gpioconfig(BOARD_USART1_TX_GPIO_BASE,
                            BOARD_USART1_TX_PIN, HT32_GPIO_MODE_ALTFN,
                            false, HT32_GPIO_DRIVE_MEDIUM,
                            HT32_GPIO_PULL_NONE,
                            BOARD_USART1_TX_AF);
      ht32f491x3_gpioconfig(BOARD_USART1_RX_GPIO_BASE,
                            BOARD_USART1_RX_PIN, HT32_GPIO_MODE_ALTFN,
                            false, HT32_GPIO_DRIVE_MEDIUM,
                            HT32_GPIO_PULL_UP,
                            BOARD_USART1_RX_AF);
      return;
    }
#endif

#ifdef BOARD_USART2_GPIO_CLKEN
  if (uartbase == HT32_USART2_BASE)
    {
      modifyreg32(HT32_CRM_AHBEN1, 0, BOARD_USART2_GPIO_CLKEN);
      ht32f491x3_gpioconfig(BOARD_USART2_TX_GPIO_BASE,
                            BOARD_USART2_TX_PIN, HT32_GPIO_MODE_ALTFN,
                            false, HT32_GPIO_DRIVE_MEDIUM,
                            HT32_GPIO_PULL_NONE,
                            BOARD_USART2_TX_AF);
      ht32f491x3_gpioconfig(BOARD_USART2_RX_GPIO_BASE,
                            BOARD_USART2_RX_PIN, HT32_GPIO_MODE_ALTFN,
                            false, HT32_GPIO_DRIVE_MEDIUM,
                            HT32_GPIO_PULL_UP,
                            BOARD_USART2_RX_AF);
      return;
    }
#endif

#ifdef BOARD_USART3_GPIO_CLKEN
  if (uartbase == HT32_USART3_BASE)
    {
      modifyreg32(HT32_CRM_AHBEN1, 0, BOARD_USART3_GPIO_CLKEN);
      ht32f491x3_gpioconfig(BOARD_USART3_TX_GPIO_BASE,
                            BOARD_USART3_TX_PIN, HT32_GPIO_MODE_ALTFN,
                            false, HT32_GPIO_DRIVE_MEDIUM,
                            HT32_GPIO_PULL_NONE,
                            BOARD_USART3_TX_AF);
      ht32f491x3_gpioconfig(BOARD_USART3_RX_GPIO_BASE,
                            BOARD_USART3_RX_PIN, HT32_GPIO_MODE_ALTFN,
                            false, HT32_GPIO_DRIVE_MEDIUM,
                            HT32_GPIO_PULL_UP,
                            BOARD_USART3_RX_AF);
    }
#endif
}

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  while ((getreg32(HT32_CONSOLE_BASE + HT32_USART_STS_OFFSET) &
          HT32_USART_STS_TDBE) == 0)
    {
    }

  putreg32((uint32_t)ch & HT32_USART_DT_MASK,
           HT32_CONSOLE_BASE + HT32_USART_DT_OFFSET);
#endif
}

void ht32f491x3_lowsetup(void)
{
#if defined(HAVE_CONSOLE)
  uint32_t regval;

  modifyreg32(HT32_CONSOLE_APBREG, 0, HT32_CONSOLE_APBEN);
  ht32f491x3_usart_config(HT32_CONSOLE_BASE);
  ht32f491x3_usart_pins(HT32_CONSOLE_BASE);

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  regval  = getreg32(HT32_CONSOLE_BASE + HT32_USART_CTRL1_OFFSET);
  regval &= ~HT32_USART_CTRL1_UEN;
  putreg32(regval, HT32_CONSOLE_BASE + HT32_USART_CTRL1_OFFSET);

  regval  = getreg32(HT32_CONSOLE_BASE + HT32_USART_CTRL2_OFFSET);
  regval &= ~HT32_USART_CTRL2_CLRBITS;
  if (HT32_CONSOLE_2STOP != 0)
    {
      regval |= HT32_USART_CTRL2_STOPBN_20;
    }
  else
    {
      regval |= HT32_USART_CTRL2_STOPBN_10;
    }

  putreg32(regval, HT32_CONSOLE_BASE + HT32_USART_CTRL2_OFFSET);

  regval  = getreg32(HT32_CONSOLE_BASE + HT32_USART_CTRL1_OFFSET);
  regval &= ~HT32_USART_CTRL1_CLRBITS;

  if (HT32_CONSOLE_PARITY == 1)
    {
      regval |= HT32_USART_CTRL1_PEN;
    }
  else if (HT32_CONSOLE_PARITY == 2)
    {
      regval |= HT32_USART_CTRL1_PEN | HT32_USART_CTRL1_PSEL;
    }

  if (HT32_CONSOLE_BITS == 9 ||
      (HT32_CONSOLE_BITS == 8 && HT32_CONSOLE_PARITY != 0))
    {
      regval |= HT32_USART_CTRL1_DBN0;
    }
  else if (HT32_CONSOLE_BITS == 7)
    {
      regval |= HT32_USART_CTRL1_DBN1;
    }

  putreg32(regval, HT32_CONSOLE_BASE + HT32_USART_CTRL1_OFFSET);

  regval  = getreg32(HT32_CONSOLE_BASE + HT32_USART_CTRL3_OFFSET);
  regval &= ~HT32_USART_CTRL3_CLRBITS;
  putreg32(regval, HT32_CONSOLE_BASE + HT32_USART_CTRL3_OFFSET);

  putreg32(ht32f491x3_bauddiv(HT32_CONSOLE_CLOCK, HT32_CONSOLE_BAUD),
           HT32_CONSOLE_BASE + HT32_USART_BAUDR_OFFSET);
#endif

  regval  = getreg32(HT32_CONSOLE_BASE + HT32_USART_CTRL1_OFFSET);
  regval |= HT32_USART_CTRL1_UEN |
            HT32_USART_CTRL1_TEN |
            HT32_USART_CTRL1_REN;
  putreg32(regval, HT32_CONSOLE_BASE + HT32_USART_CTRL1_OFFSET);
#endif
}
