/****************************************************************************
 * drivers/serial/serial_io.c
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

#ifdef CONFIG_SMP
#  include <nuttx/irq.h>
#endif

#include <assert.h>
#include <sys/types.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/signal.h>
#include <nuttx/serial/serial.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_xmitchars
 *
 * Description:
 *   This function is called from the UART interrupt handler when an
 *   interrupt is received indicating that there is more space in the
 *   transmit FIFO.  This function will send characters from the tail of
 *   the xmit buffer while the driver write() logic adds data to the head
 *   of the xmit buffer.
 *
 ****************************************************************************/

void uart_xmitchars(FAR uart_dev_t *dev)
{
  uint16_t nbytes = 0;

#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif

  /* Send while we still have data in the TX buffer & room in the fifo */

  while (dev->xmit.head != dev->xmit.tail && uart_txready(dev))
    {
      /* Send the next byte */

      uart_send(dev, dev->xmit.buffer[dev->xmit.tail]);
      nbytes++;

      /* Increment the tail index */

      if (++(dev->xmit.tail) >= dev->xmit.size)
        {
          dev->xmit.tail = 0;
        }
    }

  /* When all of the characters have been sent from the buffer disable the TX
   * interrupt.
   *
   * Potential bug?  If nbytes == 0 && (dev->xmit.head == dev->xmit.tail) &&
   * dev->xmitwaiting == true, then disabling the TX interrupt will leave
   * the uart_write() logic waiting to TX to complete with no TX interrupts.
   * Can that happen?
   */

  if (dev->xmit.head == dev->xmit.tail)
    {
      uart_disabletxint(dev);
    }

  /* If any bytes were removed from the buffer, inform any waiters that
   * there is space available.
   */

  if (nbytes)
    {
      uart_datasent(dev);
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Name: uart_recvchars
 *
 * Description:
 *   This function is called from the UART interrupt handler when an
 *   interrupt is received indicating that are bytes available in the
 *   receive FIFO.  This function will add chars to head of receive buffer.
 *   Driver read() logic will take characters from the tail of the buffer.
 *
 ****************************************************************************/

void uart_recvchars(FAR uart_dev_t *dev)
{
  FAR struct uart_buffer_s *rxbuf = &dev->recv;
#ifdef CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS
  unsigned int watermark;
#endif
  unsigned int status;
  int nexthead = rxbuf->head + 1;
#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP) || \
    defined(CONFIG_TTY_FORCE_PANIC) || defined(CONFIG_TTY_LAUNCH)
  int signo = 0;
#endif
  uint16_t nbytes = 0;

  if (nexthead >= rxbuf->size)
    {
      nexthead = 0;
    }

#ifdef CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS
  /* Pre-calculate the watermark level that we will need to test against. */

  watermark = (CONFIG_SERIAL_IFLOWCONTROL_UPPER_WATERMARK * rxbuf->size) /
              100;
#endif

  /* Loop putting characters into the receive buffer until there are no
   * further characters to available.
   */

  while (uart_rxavailable(dev))
    {
      bool is_full = (nexthead == rxbuf->tail);
      char ch;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
#ifdef CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS
      unsigned int nbuffered;

      /* How many bytes are buffered */

      if (rxbuf->head >= rxbuf->tail)
        {
          nbuffered = rxbuf->head - rxbuf->tail;
        }
      else
        {
          nbuffered = rxbuf->size - rxbuf->tail + rxbuf->head;
        }

      /* Is the level now above the watermark level that we need to report? */

      if (nbuffered >= watermark)
        {
          /* Let the lower level driver know that the watermark level has
           * been crossed.  It will probably activate RX flow control.
           */

          if (uart_rxflowcontrol(dev, nbuffered, true))
            {
              /* Low-level driver activated RX flow control, exit loop now. */

              break;
            }
        }
#else
      /* Check if RX buffer is full and allow serial low-level driver to
       * pause processing. This allows proper utilization of hardware flow
       * control.
       */

      if (is_full)
        {
          if (uart_rxflowcontrol(dev, rxbuf->size, true))
            {
              /* Low-level driver activated RX flow control, exit loop now. */

              break;
            }
        }
#endif
#endif

      /* Get this next character from the hardware */

      ch = uart_receive(dev, &status);

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP) || \
    defined(CONFIG_TTY_FORCE_PANIC) || defined(CONFIG_TTY_LAUNCH)
      signo = uart_check_special(dev, &ch, 1);
#endif

      /* If the RX buffer becomes full, then the serial data is discarded.
       * This is necessary because on most serial hardware, you must read
       * the data in order to clear the RX interrupt. An option on some
       * hardware might be to simply disable RX interrupts until the RX
       * buffer becomes non-FULL.  However, that would probably just cause
       * the overrun to occur in hardware (unless it has some large internal
       * buffering).
       */

      if (!is_full)
        {
          /* Add the character to the buffer */

          rxbuf->buffer[rxbuf->head] = ch;
          nbytes++;

          /* Increment the head index */

          rxbuf->head = nexthead;
          if (++nexthead >= rxbuf->size)
            {
               nexthead = 0;
            }
        }
    }

  /* If any bytes were added to the buffer, inform any waiters there is new
   * incoming data available.
   */

  if (nbytes)
    {
      uart_datareceived(dev);
    }

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP) || \
    defined(CONFIG_TTY_FORCE_PANIC) || defined(CONFIG_TTY_LAUNCH)
  /* Send the signal if necessary */

  if (signo != 0)
    {
      nxsig_kill(dev->pid, signo);
      uart_reset_sem(dev);
    }
#endif
}
