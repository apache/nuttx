/************************************************************************************
 * drivers/serialirq.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <semaphore.h>
#include <debug.h>
#include <nuttx/serial.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Private Variables
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: uart_xmitchars
 *
 * Description:
 *   This function is called from the UART interrupt handler when an interrupt
 *   is received indicating that there is more space in the transmit FIFO.  This
 *   function will send characters from the tail of the xmit buffer while the driver
 *   write() logic adds data to the head of the xmit buffer.
 *
 ************************************************************************************/

void uart_xmitchars(FAR uart_dev_t *dev)
{
  /* Send while we still have data & room in the fifo */

  while (dev->xmit.head != dev->xmit.tail && uart_txready(dev))
    {
      uart_send(dev, dev->xmit.buffer[dev->xmit.tail]);

      if (++(dev->xmit.tail) >= dev->xmit.size)
        {
          dev->xmit.tail = 0;
        }

      if (dev->xmitwaiting)
        {
          dev->xmitwaiting = FALSE;
          uart_givesem(&dev->xmitsem);
        }
    }

  /* When all of the characters have been sent from the buffer
   * disable the TX interrupt.
   */

  if (dev->xmit.head == dev->xmit.tail)
    {
      uart_disabletxint(dev);
    }
}

/************************************************************************************
 * Name: uart_receivechars
 *
 * Description:
 *   This function is called from the UART interrupt handler when an interrupt
 *   is received indicating that are bytes available in the receive fifo.  This
 *   function will add chars to head of receive buffer.  Driver read() logic will take
 *   characters from the tail of the buffer.
 *
 ************************************************************************************/

void uart_recvchars(FAR uart_dev_t *dev)
{
  unsigned int status;
  int nexthead = dev->recv.head + 1;

  if (nexthead >= dev->recv.size)
    {
      nexthead = 0;
    }

  while (nexthead != dev->recv.tail && uart_rxavailable(dev))
    {
      dev->recv.buffer[dev->recv.head] = uart_receive(dev, &status);

      dev->recv.head = nexthead;
      if (++nexthead >= dev->recv.size)
        {
           nexthead = 0;
        }

      if (dev->recvwaiting)
        {
          dev->recvwaiting = FALSE;
          uart_givesem(&dev->recvsem);
        }
    }
}

