/****************************************************************************
 * drivers/serial/serial_dma.c
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author:  Max Neklyudov <macscomp@gmail.com>
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
#include <debug.h>

#include <nuttx/serial/serial.h>

#if defined(CONFIG_SERIAL_TXDMA) || defined(CONFIG_SERIAL_RXDMA)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_check_signo
 *
 * Description:
 *   Check if the SIGINT or SIGSTP character is in the contiguous Rx DMA
 *   buffer region.  The first signal associated with the first such
 *   character is returned.
 *
 *   If there multiple such characters in the buffer, only the signal
 *   associated with the first is returned (this a bug!)
 *
 * Returned Value:
 *   0 if a signal-related character does not appear in the.  Otherwise,
 *   SIGKILL or SIGSTP may be returned to indicate the appropriate signal
 *   action.
 *
 ****************************************************************************/

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGSTP)
static int uart_check_signo(const char *buf, size_t size)
{
  size_t i;

  for (i = 0; i < size; i++)
    {
#ifdef CONFIG_TTY_SIGINT
      if (buf[i] == CONFIG_TTY_SIGINT_CHAR)
        {
          return SIGINT;
        }
#endif

#ifdef CONFIG_TTY_SIGSTP
      if (buf[i] == CONFIG_TTY_SIGSTP_CHAR)
        {
          return SIGSTP;
        }
#endif
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: uart_recvchars_signo
 *
 * Description:
 *   Check if the SIGINT character is anywhere in the newly received DMA
 *   buffer.
 *
 *   REVISIT:  We must also remove the SIGINT/SIGSTP character from the Rx
 *   buffer.  It should not be read as normal data by the caller.
 *
 ****************************************************************************/

#if defined(CONFIG_SERIAL_RXDMA) && \
   (defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGSTP))
static int uart_recvchars_signo(FAR uart_dev_t *dev)
{
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  int signo;

  /* Check if the valid DMAed data is in one or two contiguous regions */

  if (xfer->nbytes <= xfer->length)
    {
      return uart_check_signo(xfer->buffer, xfer->nbytes);
    }
  else
    {
      /* REVISIT:  Additional signals could be in the second region. */

      signo = uart_check_signo(xfer->buffer, xfer->length);
      if (signo != 0)
        {
          return signo;
        }

      return uart_check_signo(xfer->nbuffer, xfer->nbytes - xfer->length);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_xmitchars_dma
 *
 * Description:
 *   Set up to transfer bytes from the TX circular buffer using DMA
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
void uart_xmitchars_dma(FAR uart_dev_t *dev)
{
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;

  if (dev->xmit.head == dev->xmit.tail)
    {
      /* No data to transfer. */

      return;
    }

  if (dev->xmit.tail < dev->xmit.head)
    {
      xfer->buffer  = &dev->xmit.buffer[dev->xmit.tail];
      xfer->length  = dev->xmit.head - dev->xmit.tail;
      xfer->nbuffer = NULL;
      xfer->nlength = 0;
    }
  else
    {
      xfer->buffer  = &dev->xmit.buffer[dev->xmit.tail];
      xfer->length  = dev->xmit.size - dev->xmit.tail;
      xfer->nbuffer = dev->xmit.buffer;
      xfer->nlength = dev->xmit.head;
    }

  uart_dmasend(dev);
}
#endif

/****************************************************************************
 * Name: uart_xmitchars_done
 *
 * Description:
 *   Perform operations necessary at the complete of DMA including adjusting
 *   the TX circular buffer indices and waking up of any threads that may
 *   have been waiting for space to become available in the TX circular
 *   buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
void uart_xmitchars_done(FAR uart_dev_t *dev)
{
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  size_t nbytes = xfer->nbytes;
  struct uart_buffer_s *txbuf = &dev->xmit;

  /* Skip the update if the tail position change which mean
   * someone reset (e.g. TCOFLUSH) the xmit buffer during DMA.
   */

  if (xfer->buffer == &txbuf->buffer[txbuf->tail])
    {
      /* Move tail for nbytes. */

      txbuf->tail  = (txbuf->tail + nbytes) % txbuf->size;
    }

  /* Reset xmit buffer. */

  xfer->nbytes = 0;
  xfer->length = xfer->nlength = 0;

  /* If any bytes were removed from the buffer, inform any waiters there
   * there is space available.
   */

  if (nbytes)
    {
      uart_datasent(dev);
    }
}
#endif

/****************************************************************************
 * Name: uart_recvchars_dma
 *
 * Description:
 *   Set up to receive bytes into the RX circular buffer using DMA
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_RXDMA
void uart_recvchars_dma(FAR uart_dev_t *dev)
{
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR struct uart_buffer_s *rxbuf = &dev->recv;
#ifdef CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS
  unsigned int nbuffered;
  unsigned int watermark;
#endif
  bool is_full;
  int nexthead;

  /* If RX buffer is empty move tail and head to zero position */

  if (rxbuf->head == rxbuf->tail)
    {
      rxbuf->head = 0;
      rxbuf->tail = 0;
    }

  /* Get the next head index and check if there is room to adding another
   * byte to the buffer.
   */

  nexthead = rxbuf->head + 1;
  if (nexthead >= rxbuf->size)
    {
      nexthead = 0;
    }

  is_full = nexthead == rxbuf->tail;

#ifdef CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS
  /* Pre-calcuate the watermark level that we will need to test against. */

  watermark = (CONFIG_SERIAL_IFLOWCONTROL_UPPER_WATERMARK * rxbuf->size) /
              100;
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
#ifdef CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS
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
      /* Let the lower level driver know that the watermark level has been
       * crossed.  It will probably activate RX flow control.
       */

      if (uart_rxflowcontrol(dev, nbuffered, true))
        {
          /* Low-level driver activated RX flow control, return now. */

          return;
        }
    }

#else
  /* Check if RX buffer is full and allow serial low-level driver to pause
   * processing. This allows proper utilization of hardware flow control.
   */

  if (is_full)
    {
      if (uart_rxflowcontrol(dev, rxbuf->size, true))
        {
          /* Low-level driver activated RX flow control, return now. */

          return;
        }
    }
#endif
#endif

  if (is_full)
    {
      /* If there is no free space in receive buffer we cannot start DMA
       * transfer.
       */

      return;
    }

  if (rxbuf->tail <= rxbuf->head)
    {
      xfer->buffer  = &rxbuf->buffer[rxbuf->head];
      xfer->nbuffer = rxbuf->buffer;

      if (rxbuf->tail > 0)
        {
          xfer->length  = rxbuf->size - rxbuf->head;
          xfer->nlength = rxbuf->tail - 1;
        }
      else
        {
          xfer->length  = rxbuf->size - rxbuf->head - 1;
          xfer->nlength = 0;
        }
    }
  else
    {
      xfer->buffer  = &rxbuf->buffer[rxbuf->head];
      xfer->length  = rxbuf->tail - rxbuf->head - 1;
      xfer->nbuffer = NULL;
      xfer->nlength = 0;
    }

  uart_dmareceive(dev);
}
#endif

/****************************************************************************
 * Name: uart_recvchars_done
 *
 * Description:
 *   Perform operations necessary at the complete of DMA including adjusting
 *   the RX circular buffer indices and waking up of any threads that may
 *   have been waiting for new data to become available in the RX circular
 *   buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_RXDMA
void uart_recvchars_done(FAR uart_dev_t *dev)
{
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR struct uart_buffer_s *rxbuf = &dev->recv;
  size_t nbytes = xfer->nbytes;
#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGSTP)
  int signo = 0;

  /* Check if the SIGINT character is anywhere in the newly received DMA
   * buffer.
   */

  if (dev->pid >= 0 && (dev->tc_lflag & ISIG))
    {
      signo = uart_recvchars_signo(dev);
    }
#endif

  /* Move head for nbytes. */

  rxbuf->head  = (rxbuf->head + nbytes) % rxbuf->size;
  xfer->nbytes = 0;
  xfer->length = xfer->nlength = 0;

  /* If any bytes were added to the buffer, inform any waiters there is new
   * incoming data available.
   */

  if (nbytes)
    {
      uart_datareceived(dev);
    }

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGSTP)
  /* Send the signal if necessary */

  if (signo != 0)
    {
      kill(dev->pid, signo);
      uart_reset_sem(dev);
    }
#endif
}
#endif

#endif /* CONFIG_SERIAL_TXDMA || CONFIG_SERIAL_RXDMA */
