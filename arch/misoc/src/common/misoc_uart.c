/****************************************************************************
 * arch/misoc/src/common/misoc_uart.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
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
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>
#include <arch/board/generated/csr.h>

#include "hw/flags.h"
#include "misoc_uart.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buffer sizes must be a power of 2 so that modulos can be computed
 * with logical AND.
 */

#define UART_RINGBUFFER_SIZE_RX 128
#define UART_RINGBUFFER_MASK_RX (UART_RINGBUFFER_SIZE_RX-1)

#define UART_RINGBUFFER_SIZE_TX 128
#define UART_RINGBUFFER_MASK_TX (UART_RINGBUFFER_SIZE_TX-1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char rx_buf[UART_RINGBUFFER_SIZE_RX];
static volatile unsigned int rx_produce;
static unsigned int rx_consume;

static char tx_buf[UART_RINGBUFFER_SIZE_TX];
static unsigned int tx_produce;
static volatile unsigned int tx_consume;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_interrupt
 ****************************************************************************/

static int uart_interrupt(int irq, void *context)
{
  unsigned int stat, rx_produce_next;

  stat = uart_ev_pending_read();

  if ((stat & UART_EV_RX) != 0)
    {
      while (!uart_rxempty_read())
        {
          rx_produce_next = (rx_produce + 1) & UART_RINGBUFFER_MASK_RX;
          if (rx_produce_next != rx_consume)
            {
              rx_buf[rx_produce] = uart_rxtx_read();
              rx_produce = rx_produce_next;
            }

          uart_ev_pending_write(UART_EV_RX);
        }
    }

  if ((stat & UART_EV_TX) != 0)
    {
      uart_ev_pending_write(UART_EV_TX);
      while ((tx_consume != tx_produce) && !uart_txfull_read())
        {
          uart_rxtx_write(tx_buf[tx_consume]);
          tx_consume = (tx_consume + 1) & UART_RINGBUFFER_MASK_TX;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_read
 *
 * Do not use in interrupt handlers!
 *
 ****************************************************************************/

char uart_read(void)
{
  char c;

  if (irq_getie())
    {
      while (rx_consume == rx_produce);
    }
  else if (rx_consume == rx_produce)
    {
      return 0;
    }

  c = rx_buf[rx_consume];
  rx_consume = (rx_consume + 1) & UART_RINGBUFFER_MASK_RX;
  return c;
}

/****************************************************************************
 * Name: uart_read_nonblock
 ****************************************************************************/

int uart_read_nonblock(void)
{
  return (rx_consume != rx_produce);
}

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

int up_putc(int ch)
{
  unsigned int oldmask;
  unsigned int tx_produce_next = (tx_produce + 1) & UART_RINGBUFFER_MASK_TX;

  if (irq_getie())
    {
      while (tx_produce_next == tx_consume);
    }
  else if (tx_produce_next == tx_consume)
    {
      return ch;
    }

  oldmask = irq_getmask();
  irq_setmask(oldmask & ~(1 << UART_INTERRUPT));

  if ((tx_consume != tx_produce) || uart_txfull_read())
    {
      tx_buf[tx_produce] = ch;
      tx_produce = tx_produce_next;
    }
  else
    {
      uart_rxtx_write(ch);
    }

  irq_setmask(oldmask);
  return ch;
}

/****************************************************************************
 * Name: uart_init
 ****************************************************************************/

void uart_init(void)
{
  rx_produce = 0;
  rx_consume = 0;

  tx_produce = 0;
  tx_consume = 0;

  uart_ev_pending_write(uart_ev_pending_read());
  uart_ev_enable_write(UART_EV_TX | UART_EV_RX);
  irq_setmask(irq_getmask() | (1 << UART_INTERRUPT));

  irq_attach(1 << UART_INTERRUPT, uart_interrupt);
}

/****************************************************************************
 * Name: uart_sync
 ****************************************************************************/

void uart_sync(void)
{
  while (tx_consume != tx_produce);
}
