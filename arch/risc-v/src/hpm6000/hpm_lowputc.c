/****************************************************************************
 * arch/risc-v/src/hpm6000/hpm_lowputc.c
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

#include "riscv_internal.h"
#include "chip.h"
#include "hpm.h"
#include "hpm_config.h"
#include "hpm_clockconfig.h"
#include "hpm_iomux.h"
#include "hardware/hpm_uart.h"
#include "hpm_gpio.h"
#include "hpm_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_UART_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE        HPM_UART0_BASE
#    define HPM_CONSOLE_BAUD        CONFIG_UART0_BAUD
#    define HPM_CONSOLE_BITS        CONFIG_UART0_BITS
#    define HPM_CONSOLE_PARITY      CONFIG_UART0_PARITY
#    define HPM_CONSOLE_2STOP       CONFIG_UART0_2STOP
#    define HPM_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART0
#    define HPM_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART0
#    define HAVE_UART
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE        HPM_UART1_BASE
#    define HPM_CONSOLE_BAUD        CONFIG_UART1_BAUD
#    define HPM_CONSOLE_BITS        CONFIG_UART1_BITS
#    define HPM_CONSOLE_PARITY      CONFIG_UART1_PARITY
#    define HPM_CONSOLE_2STOP       CONFIG_UART1_2STOP
#    define HPM_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART1
#    define HPM_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART1
#    define HAVE_UART
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE        HPM_UART2_BASE
#    define HPM_CONSOLE_BAUD        CONFIG_UART2_BAUD
#    define HPM_CONSOLE_BITS        CONFIG_UART2_BITS
#    define HPM_CONSOLE_PARITY      CONFIG_UART2_PARITY
#    define HPM_CONSOLE_2STOP       CONFIG_UART2_2STOP
#    define HPM_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART2
#    define HPM_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART2
#    define HAVE_UART
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE        HPM_UART3_BASE
#    define HPM_CONSOLE_BAUD        CONFIG_UART3_BAUD
#    define HPM_CONSOLE_BITS        CONFIG_UART3_BITS
#    define HPM_CONSOLE_PARITY      CONFIG_UART3_PARITY
#    define HPM_CONSOLE_2STOP       CONFIG_UART3_2STOP
#    define HPM_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART3
#    define HPM_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART3
#    define HAVE_UART
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE        HPM_UART4_BASE
#    define HPM_CONSOLE_BAUD        CONFIG_UART4_BAUD
#    define HPM_CONSOLE_BITS        CONFIG_UART4_BITS
#    define HPM_CONSOLE_PARITY      CONFIG_UART4_PARITY
#    define HPM_CONSOLE_2STOP       CONFIG_UART4_2STOP
#    define HPM_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART4
#    define HPM_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART4
#    define HAVE_UART
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE        HPM_UART5_BASE
#    define HPM_CONSOLE_BAUD        CONFIG_UART5_BAUD
#    define HPM_CONSOLE_BITS        CONFIG_UART5_BITS
#    define HPM_CONSOLE_PARITY      CONFIG_UART5_PARITY
#    define HPM_CONSOLE_2STOP       CONFIG_UART5_2STOP
#    define HPM_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART5
#    define HPM_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART5
#    define HAVE_UART
#  elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE        HPM_UART6_BASE
#    define HPM_CONSOLE_BAUD        CONFIG_UART6_BAUD
#    define HPM_CONSOLE_BITS        CONFIG_UART6_BITS
#    define HPM_CONSOLE_PARITY      CONFIG_UART6_PARITY
#    define HPM_CONSOLE_2STOP       CONFIG_UART6_2STOP
#    define HPM_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART6
#    define HPM_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART6
#    define HAVE_UART
#  elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE        HPM_UART7_BASE
#    define HPM_CONSOLE_BAUD        CONFIG_UART7_BAUD
#    define HPM_CONSOLE_BITS        CONFIG_UART7_BITS
#    define HPM_CONSOLE_PARITY      CONFIG_UART7_PARITY
#    define HPM_CONSOLE_2STOP       CONFIG_UART7_2STOP
#    define HPM_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART7
#    define HPM_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART7
#    define HAVE_UART
#  elif defined(HAVE_UART)
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#endif /* HAVE_UART_CONSOLE */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !defined(CONFIG_SUPPRESS_UART_CONFIG)

/****************************************************************************
 * Name: config_baud_divisors
 *
 * Description:
 *   Configure the UART baudrate divisors.
 *
 ****************************************************************************/

static bool hpm_uart_calculate_baudrate(uint32_t freq, uint32_t baudrate,
                                         uint16_t *div_out, uint8_t *osc_out)
{
  uint16_t div;
  uint16_t osc;
  uint16_t delta;
  float tmp;

  if ((div_out == NULL) || (!freq) || (!baudrate)
    || (baudrate < HPM_UART_MINIMUM_BAUDRATE)
    || (freq / HPM_UART_BAUDRATE_DIV_MIN < baudrate * HPM_UART_OSC_MIN)
    || (freq / HPM_UART_BAUDRATE_DIV_MAX > (baudrate * HPM_UART_OSC_MAX)))
    {
      return 0;
    }

  tmp = (float) freq / baudrate;
  for (uint8_t i = 0; i < HPM_UART_OSC_MAX; i += 2)
    {
      /* osc range: 0 - 32, even number */

      if (i == 0)
        {
          /* osc == 0 in bitfield, oversample rate is 32 */

          osc = HPM_UART_OSC_MAX;
        }
      else if (i <= 8)
        {
          /* osc <= 8 in bitfield, oversample rate is 8 */

          osc = HPM_UART_OSC_MIN;
        }
      else
        {
          /* osc > 8 && osc < 32 in bitfield, oversample rate is osc */

          osc = i;
        }

      delta = 0;
      div = (uint16_t)(tmp / osc);
      if (div < HPM_UART_BAUDRATE_DIV_MIN)
        {
          /* invalid div */

          continue;
        }

      if (div * osc > tmp)
        {
          delta = div * osc - tmp;
        }
      else if (div * osc < tmp)
        {
          delta = tmp - div * osc;
        }
      else
        {
          /* Do Nothing */
        }

      if (delta && ((delta * 100 / tmp) > HPM_UART_BAUDRATE_TOLERANCE))
        {
          continue;
        }
      else
        {
          *div_out = div;
          *osc_out = (i <= 8 && i) ? osc : i;
          return true;
        }
    }

  return false;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#if defined HAVE_UART_CONSOLE && defined HAVE_UART

  /* Wait until the TX data register is empty */

  while ((getreg32(HPM_CONSOLE_BASE + HPM_UART_LSR_OFFSET)
          & UART_LSR_THRE) == 0)
    ;

  /* Then send the character */

  putreg32(ch, HPM_CONSOLE_BASE + HPM_UART_THR_OFFSET);

#endif
}

/****************************************************************************
 * Name: hpm6750_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void hpm_lowsetup(void)
{
#if defined(HAVE_UART_DEVICE)

  /* Enable and configure the selected console device */

#if defined(HAVE_UART_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

#ifdef CONFIG_HPM_UART0
  hpm_gpio_config(GPIO_UART0_RXD);
  hpm_gpio_config(GPIO_UART0_TXD);
#ifdef CONFIG_UART0_OFLOWCONTROL
  hpm_gpio_config(GPIO_UART0_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_UART0_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART0_IFLOWCONTROL)))
  hpm_config_gpio(GPIO_UART0_RTS);
#endif
#endif

#ifdef CONFIG_HPM_UART1
  hpm_gpio_config(GPIO_UART1_RXD);
  hpm_gpio_config(GPIO_UART1_TXD)
#ifdef CONFIG_UART1_OFLOWCONTROL
  hpm_gpio_config(GPIO_UART1_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_UART1_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART1_IFLOWCONTROL)))
  hpm_config_gpio(GPIO_UART1_RTS);
#endif
#endif

#ifdef CONFIG_HPM_UART2
  hpm_gpio_config(GPIO_UART2_RXD);
  hpm_gpio_config(GPIO_UART2_TXD)
#ifdef CONFIG_UART2_OFLOWCONTROL
  hpm_gpio_config(GPIO_UART2_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_UART2_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART2_IFLOWCONTROL)))
  hpm_config_gpio(GPIO_UART2_RTS);
#endif
#endif

#ifdef CONFIG_HPM_UART3
  hpm_gpio_config(GPIO_UART3_RXD);
  hpm_gpio_config(GPIO_UART3_TXD)
#ifdef CONFIG_UART3_OFLOWCONTROL
  hpm_gpio_config(GPIO_UART3_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_UART3_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART3_IFLOWCONTROL)))
  hpm_config_gpio(GPIO_UART3_RTS);
#endif
#endif

#ifdef CONFIG_HPM_UART4
  hpm_gpio_config(GPIO_UART4_RXD);
  hpm_gpio_config(GPIO_UART4_TXD)
#ifdef CONFIG_UART4_OFLOWCONTROL
  hpm_gpio_config(GPIO_UART4_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_UART4_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART4_IFLOWCONTROL)))
  hpm_config_gpio(GPIO_UART4_RTS);
#endif
#endif

#ifdef CONFIG_HPM_UART5
  hpm_gpio_config(GPIO_UART5_RXD);
  hpm_gpio_config(GPIO_UART5_TXD)
#ifdef CONFIG_UART5_OFLOWCONTROL
  hpm_gpio_config(GPIO_UART5_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_UART5_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART5_IFLOWCONTROL)))
  hpm_config_gpio(GPIO_UART5_RTS);
#endif
#endif

#ifdef CONFIG_HPM_UART6
  hpm_gpio_config(GPIO_UART6_RXD);
  hpm_gpio_config(GPIO_UART6_TXD)
#ifdef CONFIG_UART6_OFLOWCONTROL
  hpm_gpio_config(GPIO_UART6_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_UART6_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART6_IFLOWCONTROL)))
  hpm_config_gpio(GPIO_UART6_RTS);
#endif
#endif

#ifdef CONFIG_HPM_UART7
  hpm_gpio_config(GPIO_UART7_RXD);
  hpm_gpio_config(GPIO_UART7_TXD)
#ifdef CONFIG_UART7_OFLOWCONTROL
  hpm_gpio_config(GPIO_UART7_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_UART7_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART7_IFLOWCONTROL)))
  hpm_config_gpio(GPIO_UART7_RTS);
#endif
#endif

  hpm_uart_clockconfig();

#endif /* HAVE_UART_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}

/****************************************************************************
 * Name: hpm_uart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
int hpm_uart_configure(uint32_t base, const struct uart_config_s *config)
{
  uint8_t osc = 0;
  uint16_t div = 0;
  uint8_t lcr = 0;

  /* disable all interrupts */

  putreg32(0, base + HPM_UART_IER_OFFSET);

  /* set DLAB to 1 */

  lcr = getreg32(base + HPM_UART_LCR_OFFSET) | UART_LCR_DLAB;
  while ((getreg32(base + HPM_UART_LCR_OFFSET) & UART_LCR_DLAB)
         != UART_LCR_DLAB)
    {
      putreg32(lcr, base + HPM_UART_LCR_OFFSET);
    }

  hpm_uart_calculate_baudrate(24000000, config->baud, &div, &osc);
  putreg32(osc, base + HPM_UART_OSCR_OFFSET);
  putreg32(div & 0xff, base + HPM_UART_DLL_OFFSET);
  putreg32(div >> 8, base + HPM_UART_DLM_OFFSET);

  /* DLAB bit needs to be cleared once baudrate is configured */

  while ((getreg32(base + HPM_UART_LCR_OFFSET) & UART_LCR_DLAB)
         == UART_LCR_DLAB)
    {
      lcr &= ~UART_LCR_DLAB;
      putreg32(lcr, base + HPM_UART_LCR_OFFSET);
    }

  lcr = 0;
  switch (config->bits)
    {
    case 5:
      lcr |= UART_LCR_WLS_5BITS;
      break;

    case 6:
      lcr |= UART_LCR_WLS_6BITS;
      break;

    case 7:
      lcr |= UART_LCR_WLS_7BITS;
      break;

    case 8:
    default:
      lcr |= UART_LCR_WLS_8BITS;
      break;
    }

  if (config->stopbits2)
    {
      lcr |= UART_LCR_STB;
    }

  if (config->parity == 1)
    {
      lcr |= UART_LCR_PEN;
    }
  else if (config->parity == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

  putreg32(lcr, base + HPM_UART_LCR_OFFSET);

  return OK;
}

#if defined (HAVE_UART_DEVICE)
void hpm_lowputc(int ch)
{
#ifdef HAVE_UART_CONSOLE
  while ((getreg32(HPM_CONSOLE_BASE + HPM_UART_LSR_OFFSET) &
          UART_LSR_THRE) == 0)
    {
    }

  if (ch == '\n')
    {
      putreg32((uint32_t)'\r', HPM_CONSOLE_BASE + HPM_UART_THR_OFFSET);
      while ((getreg32(HPM_CONSOLE_BASE + HPM_UART_LSR_OFFSET) &
              UART_LSR_THRE) == 0)
        {
        }
    }

  putreg32((uint32_t)ch, HPM_CONSOLE_BASE + HPM_UART_THR_OFFSET);
#endif
}
#endif
#endif /* HAVE_UART_DEVICE */
