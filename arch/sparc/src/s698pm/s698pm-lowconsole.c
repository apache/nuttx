/****************************************************************************
 * arch/sparc/src/s698pm/s698pm-lowconsole.c
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
#include <arch/board/board.h>

#include "s698pm-config.h"

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "sparc_internal.h"
#include "s698pm-uart.h"
#include "s698pm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define S698PM_CONSOLE_BASE     S698PM_UART1_BASE
#    define S698PM_CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define S698PM_CONSOLE_BITS     CONFIG_UART1_BITS
#    define S698PM_CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define S698PM_CONSOLE_2STOP    CONFIG_UART1_2STOP
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define S698PM_CONSOLE_BASE     S698PM_UART2_BASE
#    define S698PM_CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define S698PM_CONSOLE_BITS     CONFIG_UART2_BITS
#    define S698PM_CONSOLE_PARITY   CONFIG_UART2_PARITY
#    define S698PM_CONSOLE_2STOP    CONFIG_UART2_2STOP
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define S698PM_CONSOLE_BASE     S698PM_UART3_BASE
#    define S698PM_CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define S698PM_CONSOLE_BITS     CONFIG_UART3_BITS
#    define S698PM_CONSOLE_PARITY   CONFIG_UART3_PARITY
#    define S698PM_CONSOLE_2STOP    CONFIG_UART3_2STOP
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define S698PM_CONSOLE_BASE     S698PM_UART4_BASE
#    define S698PM_CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define S698PM_CONSOLE_BITS     CONFIG_UART4_BITS
#    define S698PM_CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define S698PM_CONSOLE_2STOP    CONFIG_UART4_2STOP
#  else
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#endif

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
 * Name: s698pm_putreg
 *
 * Description:
 *   Write a value to a UART register
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static inline void s698pm_putreg(uintptr_t uart_base, unsigned int offset,
                                 uint32_t value)
{
  putreg32(value, uart_base + offset);
}
#endif

/****************************************************************************
 * Name: s698pm_getreg
 *
 * Description:
 *   Get a value from a UART register
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static inline uint32_t s698pm_getreg(uintptr_t uart_base,
                                     unsigned int offset)
{
  return getreg32(uart_base + offset);
}
#endif

/****************************************************************************
 * Name: s698pm_uartreset
 *
 * Description:
 *   Reset UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void s698pm_uartreset(uintptr_t uart_base)
{
  uint32_t reg;

  /* Clear USART configuration */

  reg = s698pm_getreg(uart_base, S698PM_UART_CTRLREG_OFFSET);
  uart_disable(reg);
  uart_parity_config(reg, NONE);
  uart_flow_ctrl_config(reg, OFF);
  uart_loopback_config(reg, OFF);
  s698pm_putreg(uart_base, S698PM_UART_CTRLREG_OFFSET, reg);
}
#endif

/****************************************************************************
 * Name: s698pm_uartconfigure
 *
 * Description:
 *   Configure a UART as a console.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void s698pm_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                          unsigned int parity, unsigned int nbits,
                          bool stop2)
{
  uint32_t reg;

  /* Select baud. */

  s698pm_putreg(S698PM_CONSOLE_BASE, S698PM_UART_SCALREG_OFFSET,
                uart_set_baudrate(baudrate));

  reg = s698pm_getreg(S698PM_CONSOLE_BASE, S698PM_UART_CTRLREG_OFFSET);

  /* Select parity */

  if (parity == 1)
    {
      uart_parity_config(reg, ODD); /* Odd parity */
    }
  else if (parity == 2)
    {
      uart_parity_config(reg, EVEN); /* Even parity */
    }
  else
    {
      uart_parity_config(reg, NONE); /* Even none */
    }

  uart_flow_ctrl_config(reg, OFF);
  uart_loopback_config(reg, OFF);

  uart_enable(reg);

  s698pm_putreg(S698PM_CONSOLE_BASE, S698PM_UART_CTRLREG_OFFSET, reg);
}
#endif

/****************************************************************************
 * Name: s698pm_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the initialization sequence to configure the serial console
 *   uart
 *   (only).
 *
 ****************************************************************************/

void s698pm_consoleinit(void)
{
  uint32_t gpreg;
#ifdef HAVE_UART_DEVICE

  /* Setup up pin selection registers for all configured UARTs.  The board.h
   * header file must provide these definitions to select the correct pin
   * configuration for each enabled UART.
   */

  gpreg = getreg32(S698PM_GPREG_BASE);

#ifdef CONFIG_S698PM_UART3
  /* Configure UART3 RX (input) and TX (output) pins */

  gpreg |= 0x1;
  putreg32(gpreg, S698PM_GPREG_BASE);

#endif /* CONFIG_S698PM_UART3 */

#ifdef CONFIG_S698PM_UART4
  /* Configure UART4 RX (input) and TX (output) pins */

  gpreg |= 0x2;
  putreg32(gpreg, S698PM_GPREG_BASE);

#endif /* CONFIG_S698PM_UART4 */

#ifdef HAVE_SERIAL_CONSOLE
  /* Configure the console uart */

  s698pm_uartconfigure(S698PM_CONSOLE_BASE, S698PM_CONSOLE_BAUD,
                       S698PM_CONSOLE_PARITY, S698PM_CONSOLE_BITS,
                       S698PM_CONSOLE_2STOP);

#endif /* HAVE_SERIAL_CONSOLE */
#endif /* HAVE_UART_DEVICE */
}

/****************************************************************************
 * Name: sparc_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void sparc_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  while ((s698pm_getreg(S698PM_CONSOLE_BASE, S698PM_UART_STATREG_OFFSET) &
          UART_STA_TE) == 0);

  /* Then write the character to the TX data register */

  s698pm_putreg(S698PM_CONSOLE_BASE, S698PM_UART_TXREG_OFFSET,
                 (uint32_t)ch);
#endif
}
