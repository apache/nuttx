/****************************************************************************
 * arch/sparc/src/bm3823/bm3823-lowconsole.c
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

#include "bm3823-config.h"

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_internal.h"
#include "bm3823-uart.h"
#include "bm3823.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usart0_reset and usart1_reset
 *
 * Description:
 *   Reset USART0 or USART1.
 *
 ****************************************************************************/

#ifdef CONFIG_BM3823_UART1
void uart1_reset(void)
{
  /* Clear USART configuration */

  uart1_disable();
  uart1_parity_config(NONE);
  uart1_flow_ctrl_config(OFF);
  uart1_loopback_config(OFF);
}
#endif

#ifdef CONFIG_BM3823_UART2
void uart2_reset(void)
{
  /* Clear USART configuration */

  uart2_disable();
  uart2_parity_config(NONE);
  uart2_flow_ctrl_config(OFF);
  uart2_loopback_config(OFF);
}
#endif

#ifdef CONFIG_BM3823_UART3
void uart3_reset(void)
{
  /* Clear USART configuration */

  uart3_disable();
  uart3_parity_config(NONE);
  uart3_flow_ctrl_config(OFF);
  uart3_loopback_config(OFF);
}
#endif

/****************************************************************************
 * Name: bm3823_uartconfigure
 *
 * Description:
 *   Configure a UART as a console.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void bm3823_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                        unsigned int parity, unsigned int nbits, bool stop2)
{
  if (uart_base == BM3823_UART1_BASE)
    {
      /* Select baud. */

      uart1_set_baudrate(baudrate);

      /* Select parity */

      if (parity == 1)
        {
          uart1_parity_config(ODD);  /* Odd parity */
        }
      else if (parity == 2)
        {
          uart1_parity_config(EVEN); /* Even parity */
        }
      else
        {
          uart1_parity_config(NONE); /* Even none */
        }

      uart1_flow_ctrl_config(OFF);
      uart1_loopback_config(OFF);

      uart1_enable();
    }
  else if (uart_base == BM3823_UART2_BASE)
    {
      /* Select baud. */

      uart2_set_baudrate(baudrate);

      /* Select parity */

      if (parity == 1)
        {
          uart2_parity_config(ODD);  /* Odd parity */
        }
      else if (parity == 2)
        {
          uart2_parity_config(EVEN); /* Even parity */
        }
      else
        {
          uart2_parity_config(NONE); /* Even none */
        }

      uart2_flow_ctrl_config(OFF);
      uart2_loopback_config(OFF);

      uart2_enable();
    }
  else
    {
      return;
    }
}
#endif

#ifdef CONFIG_BM3823_UART1
void uart1_configure(void)
{
  uint32_t i = 0;

  /* Select baud. */

  uart1_set_baudrate(CONFIG_UART1_BAUD);

  i = EXTER_REG.uart_data1;
  i = EXTER_REG.uart_data1;
  i = i;
  EXTER_REG.uart_status1 = 0;

  /* Select parity */

#if CONFIG_UART1_PARITY == 1
  uart1_parity_config(ODD);  /* Odd parity */
#elif CONFIG_UART1_PARITY == 2
  uart1_parity_config(EVEN); /* Even parity */
#else
  uart1_parity_config(NONE); /* Even none */
#endif

  uart1_flow_ctrl_config(OFF);
  uart1_loopback_config(OFF);

  uart1_enable();

  GPIO_DIR = GPIO_DIR | 0x8000;
  GPIO_DIR = GPIO_DIR & 0xffffbfff;
}
#endif

#ifdef CONFIG_BM3823_UART2
void uart2_configure(void)
{
  /* Select baud. */

  uart2_set_baudrate(CONFIG_UART2_BAUD);

  /* Select parity */

#if CONFIG_UART2_PARITY == 1
  uart2_parity_config(ODD);  /* Odd parity */
#elif CONFIG_UART2_PARITY == 2
  uart2_parity_config(EVEN); /* Even parity */
#else
  uart2_parity_config(NONE); /* Even none */
#endif

  uart2_flow_ctrl_config(OFF);
  uart2_loopback_config(OFF);

  uart2_enable();
}
#endif

#ifdef CONFIG_BM3823_UART3
void uart3_configure(void)
{
  /* Select baud. */

  uart3_set_baudrate(CONFIG_UART3_BAUD);

  /* Select parity */

#if CONFIG_UART3_PARITY == 1
  uart2_parity_config(ODD);  /* Odd parity */
#elif CONFIG_UART3_PARITY == 2
  uart3_parity_config(EVEN); /* Even parity */
#else
  uart3_parity_config(NONE); /* Even none */
#endif

  uart3_flow_ctrl_config(OFF);
  uart3_loopback_config(OFF);

  uart3_enable();
}
#endif

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *  Initialize a console for debug output.  This function is called very
 *  early in the initialization sequence to configure the serial console uart
 *  (only).
 *
 ****************************************************************************/

void up_consoleinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
  uart1_configure();
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  uart2_configure();
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  uart3_configure();
#  endif
#endif
}

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
  while (!uart1_tx_ready())
    {
    }

  uart1_send_byte(ch);

#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  while (!uart2_tx_ready())
    {
    }

  uart2_send_byte(ch);
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  while (!uart3_tx_ready())
    {
    }

  uart3_send_byte(ch);
#  endif
#endif
}
