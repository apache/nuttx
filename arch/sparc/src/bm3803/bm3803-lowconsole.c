/****************************************************************************
 * arch/sparc/src/bm3803/bm3803_lowconsole.c
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

#include "bm3803-config.h"

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "up_internal.h"
#include "bm3803-uart.h"
#include "bm3803.h"

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

#ifdef CONFIG_BM3803_UART1
void uart1_reset(void)
{
  /* Clear USART configuration */
  Uart1_disable();
  Uart1_parity_config(NONE);
  Uart1_flow_ctrl_config(OFF);
  Uart1_loopback_config(OFF);
}
#endif

#ifdef CONFIG_BM3803_UART2
void uart2_reset(void)
{
  /* Clear USART configuration */
  Uart2_disable();
  Uart2_parity_config(NONE);
  Uart2_flow_ctrl_config(OFF);
  Uart2_loopback_config(OFF);
}
#endif

#ifdef CONFIG_BM3803_UART3
void uart3_reset(void)
{
  /* Clear USART configuration */
  Uart3_disable();
  Uart3_parity_config(NONE);
  Uart3_flow_ctrl_config(OFF);
  Uart3_loopback_config(OFF);
}
#endif
/****************************************************************************
 * Name: bm3803_uartconfigure
 *
 * Description:
 *   Configure a UART as a console.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void bm3803_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                           unsigned int parity, unsigned int nbits, bool stop2)
{


  if(uart_base==BM3803_UART1_BASE)
  {
      /* Select baud. */

    Uart1_set_baudrate(baudrate);


    /* Select parity */

    if(parity==1)
    {
       Uart1_parity_config(ODD);  /* Odd parity */
    }
    else if(parity==2)
    {
       Uart1_parity_config(EVEN); /* Even parity */
    }
    else
    {
       Uart1_parity_config(NONE); /* Even none */
    }

    Uart1_flow_ctrl_config(OFF);
    Uart1_loopback_config(OFF);

    Uart1_enable();
  }
  else if(uart_base==BM3803_UART2_BASE)
  {
      /* Select baud. */

    Uart2_set_baudrate(baudrate);


    /* Select parity */

    if(parity==1)
    {
       Uart2_parity_config(ODD);  /* Odd parity */
    }
    else if(parity==2)
    {
       Uart2_parity_config(EVEN); /* Even parity */
    }
    else
    {
       Uart2_parity_config(NONE); /* Even none */
    }

    Uart2_flow_ctrl_config(OFF);
    Uart2_loopback_config(OFF);

    Uart2_enable();
  }
  else
  {
     return;
  }
}
#endif


#ifdef CONFIG_BM3803_UART1
void uart1_configure(void)
{
  /* Select baud. */

  Uart1_set_baudrate(CONFIG_UART1_BAUD);


  /* Select parity */

#if CONFIG_UART1_PARITY == 1
  Uart1_parity_config(ODD);  /* Odd parity */
#elif CONFIG_UART1_PARITY == 2
  Uart1_parity_config(EVEN); /* Even parity */
#else
  Uart1_parity_config(NONE); /* Even none */
#endif

  Uart1_flow_ctrl_config(OFF);
  Uart1_loopback_config(OFF);

  Uart1_enable();
}
#endif

#ifdef CONFIG_BM3803_UART2
void uart2_configure(void)
{
  /* Select baud. */

  Uart2_set_baudrate(CONFIG_UART2_BAUD);


  /* Select parity */

#if CONFIG_UART2_PARITY == 1
  Uart2_parity_config(ODD);  /* Odd parity */
#elif CONFIG_UART2_PARITY == 2
  Uart2_parity_config(EVEN); /* Even parity */
#else
  Uart2_parity_config(NONE); /* Even none */
#endif

  Uart2_flow_ctrl_config(OFF);
  Uart2_loopback_config(OFF);

  Uart2_enable();
}
#endif

#ifdef CONFIG_BM3803_UART3
void uart3_configure(void)
{
  /* Select baud. */

  Uart3_set_baudrate(CONFIG_UART3_BAUD);


  /* Select parity */

#if CONFIG_UART3_PARITY == 1
  Uart2_parity_config(ODD);  /* Odd parity */
#elif CONFIG_UART3_PARITY == 2
  Uart3_parity_config(EVEN); /* Even parity */
#else
  Uart3_parity_config(NONE); /* Even none */
#endif

  Uart3_flow_ctrl_config(OFF);
  Uart3_loopback_config(OFF);

  Uart3_enable();
}
#endif

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the initialization sequence to configure the serial console uart
 *   (only).
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
  while(!Uart1_tx_ready())
  {
  }
  Uart1_send_byte(ch);
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  while(!Uart2_tx_ready())
  {
  }
  Uart2_send_byte(ch);
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  while(!Uart3_tx_ready())
  {
  }
  Uart3_send_byte(ch);
#  endif
#endif
}
