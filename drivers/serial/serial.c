/****************************************************************************
 * drivers/serial/serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/cancelpt.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>
#include <nuttx/kthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check watermark levels */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) && \
    defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS)
#  if CONFIG_SERIAL_IFLOWCONTROL_LOWER_WATERMARK < 1
#    warning CONFIG_SERIAL_IFLOWCONTROL_LOWER_WATERMARK too small
#  endif
#  if CONFIG_SERIAL_IFLOWCONTROL_UPPER_WATERMARK > 99
#    warning CONFIG_SERIAL_IFLOWCONTROL_UPPER_WATERMARK too large
#  endif
#  if CONFIG_SERIAL_IFLOWCONTROL_LOWER_WATERMARK >= CONFIG_SERIAL_IFLOWCONTROL_UPPER_WATERMARK
#    warning CONFIG_SERIAL_IFLOWCONTROL_LOWER_WATERMARK too large
#    warning Must be less than CONFIG_SERIAL_IFLOWCONTROL_UPPER_WATERMARK
#  endif
#endif

/* Timing */

#define POLL_DELAY_USEC 1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     uart_takesem(FAR sem_t *sem, bool errout);

/* Write support */

static int     uart_putxmitchar(FAR uart_dev_t *dev, int ch,
                                bool oktoblock);
static inline ssize_t uart_irqwrite(FAR uart_dev_t *dev,
                                    FAR const char *buffer,
                                    size_t buflen);
static int     uart_tcdrain(FAR uart_dev_t *dev,
                            bool cancelable, clock_t timeout);

/* Character driver methods */

static int     uart_open(FAR struct file *filep);
static int     uart_close(FAR struct file *filep);
static ssize_t uart_read(FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
static ssize_t uart_write(FAR struct file *filep,
                          FAR const char *buffer,
                          size_t buflen);
static int     uart_ioctl(FAR struct file *filep,
                          int cmd, unsigned long arg);
static int     uart_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_TTY_LAUNCH_ENTRY
/* Lanch program entry, this must be supplied by the application. */

int CONFIG_TTY_LAUNCH_ENTRYPOINT(int argc, char *argv[]);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_serialops =
{
  uart_open,  /* open */
  uart_close, /* close */
  uart_read,  /* read */
  uart_write, /* write */
  NULL,       /* seek */
  uart_ioctl, /* ioctl */
  uart_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
};

#ifdef CONFIG_TTY_LAUNCH
static struct work_s g_serial_work;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_takesem
 ****************************************************************************/

static int uart_takesem(FAR sem_t *sem, bool errout)
{
  if (errout)
    {
      return nxsem_wait(sem);
    }
  else
    {
      return nxsem_wait_uninterruptible(sem);
    }
}

/****************************************************************************
 * Name: uart_givesem
 ****************************************************************************/

#define uart_givesem(sem) nxsem_post(sem)

/****************************************************************************
 * Name: uart_putxmitchar
 ****************************************************************************/

static int uart_putxmitchar(FAR uart_dev_t *dev, int ch, bool oktoblock)
{
  irqstate_t flags;
  int nexthead;
  int ret;

  /* Increment to see what the next head pointer will be.
   * We need to use the "next" head pointer to determine when the circular
   *  buffer would overrun
   */

  nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  /* Loop until we are able to add the character to the TX buffer. */

  for (; ; )
    {
      /* Check if the TX buffer is full */

      if (nexthead != dev->xmit.tail)
        {
          /* No.. not full.  Add the character to the TX buffer and return. */

          dev->xmit.buffer[dev->xmit.head] = ch;
          dev->xmit.head = nexthead;
          return OK;
        }

      /* The TX buffer is full.  Should be block, waiting for the hardware
       * to remove some data from the TX buffer?
       */

      else if (oktoblock)
        {
          /* The following steps must be atomic with respect to serial
           * interrupt handling.
           */

          flags = enter_critical_section();

          /* Check again...  In certain race conditions an interrupt may
           * have occurred between the test at the top of the loop and
           * entering the critical section and the TX buffer may no longer
           * be full.
           *
           * NOTE: On certain devices, such as USB CDC/ACM, the entire TX
           * buffer may have been emptied in this race condition.  In that
           * case, the logic would hang below waiting for space in the TX
           * buffer without this test.
           */

          if (nexthead != dev->xmit.tail)
            {
              ret = OK;
            }

#ifdef CONFIG_SERIAL_REMOVABLE
          /* Check if the removable device is no longer connected while we
           * have interrupts off.  We do not want the transition to occur
           * as a race condition before we begin the wait.
           */

          else if (dev->disconnected)
            {
              ret = -ENOTCONN;
            }
#endif
          else
            {
              /* Inform the interrupt level logic that we are waiting. */

              dev->xmitwaiting = true;

              /* Wait for some characters to be sent from the buffer with
               * the TX interrupt enabled.  When the TX interrupt is enabled,
               * uart_xmitchars() should execute and remove some of the data
               * from the TX buffer.
               *
               * NOTE that interrupts will be re-enabled while we wait for
               * the semaphore.
               */

#ifdef CONFIG_SERIAL_TXDMA
              uart_dmatxavail(dev);
#endif
              uart_enabletxint(dev);
              ret = uart_takesem(&dev->xmitsem, true);
              uart_disabletxint(dev);
            }

          leave_critical_section(flags);

#ifdef CONFIG_SERIAL_REMOVABLE
          /* Check if the removable device was disconnected while we were
           * waiting.
           */

          if (dev->disconnected)
            {
              return -ENOTCONN;
            }
#endif

          /* Check if we were awakened by signal. */

          if (ret < 0)
            {
              /* A signal received while waiting for the xmit buffer to
               * become non-full will abort the transfer.
               */

              return -EINTR;
            }
        }

      /* The caller has request that we not block for data.  So return the
       * EAGAIN error to signal this situation.
       */

      else
        {
          return -EAGAIN;
        }
    }

  /* We won't get here.  Some compilers may complain that this code is
   * unreachable.
   */

  return OK;
}

/****************************************************************************
 * Name: uart_putc
 ****************************************************************************/

static inline void uart_putc(FAR uart_dev_t *dev, int ch)
{
  while (!uart_txready(dev))
    {
    }

  uart_send(dev, ch);
}

/****************************************************************************
 * Name: uart_irqwrite
 ****************************************************************************/

static inline ssize_t uart_irqwrite(FAR uart_dev_t *dev,
                                    FAR const char *buffer,
                                    size_t buflen)
{
  ssize_t ret = buflen;

  /* Force each character through the low level interface */

  for (; buflen; buflen--)
    {
      int ch = *buffer++;

#ifdef CONFIG_SERIAL_TERMIOS
      /* Do output post-processing */

      if ((dev->tc_oflag & OPOST) != 0)
        {
          /* Mapping CR to NL? */

          if ((ch == '\r') && (dev->tc_oflag & OCRNL) != 0)
            {
              ch = '\n';
            }

          /* Are we interested in newline processing? */

          if ((ch == '\n') && (dev->tc_oflag & (ONLCR | ONLRET)) != 0)
            {
              uart_putc(dev, '\r');
            }
        }

#else /* !CONFIG_SERIAL_TERMIOS */
      /* If this is the console, then we should replace LF with CR-LF */

      if (dev->isconsole && ch == '\n')
        {
          uart_putc(dev, '\r');
        }
#endif

      /* Output the character, using the low-level direct UART interfaces */

      uart_putc(dev, ch);
    }

  return ret;
}

/****************************************************************************
 * Name: uart_tcdrain
 *
 * Description:
 *   Block further TX input.
 *   Wait until all data has been transferred from the TX buffer and
 *   until the hardware TX FIFOs are empty.
 *
 ****************************************************************************/

static int uart_tcdrain(FAR uart_dev_t *dev,
                        bool cancelable, clock_t timeout)
{
  int ret;

  /* tcdrain is a cancellation point */

  if (cancelable && enter_cancellation_point())
    {
#ifdef CONFIG_CANCELLATION_POINTS
      /* If there is a pending cancellation, then do not perform
       * the wait.  Exit now with ECANCELED.
       */

      leave_cancellation_point();
      return -ECANCELED;
#endif
    }

  /* Get exclusive access to the to dev->tmit.  We cannot permit new data to
   * be written while we are trying to flush the old data.
   *
   * A signal received while waiting for access to the xmit.head will abort
   * the operation with EINTR.
   */

  ret = (ssize_t)uart_takesem(&dev->xmit.sem, true);
  if (ret >= 0)
    {
      irqstate_t flags;
      clock_t start;

      /* Trigger emission to flush the contents of the tx buffer */

      flags = enter_critical_section();

#ifdef CONFIG_SERIAL_REMOVABLE
      /* Check if the removable device is no longer connected while we have
       * interrupts off.  We do not want the transition to occur as a race
       * condition before we begin the wait.
       */

      if (dev->disconnected)
        {
          dev->xmit.tail = dev->xmit.head;  /* Drop the buffered TX data */
          ret = -ENOTCONN;
        }
      else
#endif
        {
          /* Continue waiting while the TX buffer is not empty.
           *
           * NOTE: There is no timeout on the following loop.  In
           * situations were this loop could hang (with hardware flow
           * control, as an example),  the caller should call
           * tcflush() first to discard this buffered Tx data.
           */

          ret = OK;
          while (ret >= 0 && dev->xmit.head != dev->xmit.tail)
            {
              /* Inform the interrupt level logic that we are waiting. */

              dev->xmitwaiting = true;

              /* Wait for some characters to be sent from the buffer with
               * the TX interrupt enabled.  When the TX interrupt is
               * enabled, uart_xmitchars() should execute and remove some
               * of the data from the TX buffer.  We may have to wait several
               * times for the TX buffer to be entirely emptied.
               *
               * NOTE that interrupts will be re-enabled while we wait for
               * the semaphore.
               */

#ifdef CONFIG_SERIAL_TXDMA
              uart_dmatxavail(dev);
#endif
              uart_enabletxint(dev);
              ret = uart_takesem(&dev->xmitsem, true);
              uart_disabletxint(dev);
            }
        }

      leave_critical_section(flags);

      /* The TX buffer is empty (or an error occurred).  But there still may
       * be data in the UART TX FIFO.  We get no asynchronous indication of
       * this event, so we have to do a busy wait poll.
       */

      /* Set up for the timeout
       *
       * REVISIT:  This is a kludge.  The correct fix would be add an
       * interface to the lower half driver so that the tcflush() operation
       * all also cause the lower half driver to clear and reset the Tx FIFO.
       */

      start = clock_systime_ticks();

      if (ret >= 0)
        {
          while (!uart_txempty(dev))
            {
              clock_t elapsed;

              nxsig_usleep(POLL_DELAY_USEC);

              /* Check for a timeout */

              elapsed = clock_systime_ticks() - start;
              if (elapsed >= timeout)
                {
                  return -ETIMEDOUT;
                }
            }
        }

      uart_givesem(&dev->xmit.sem);
    }

  if (cancelable)
    {
      leave_cancellation_point();
    }

  return ret;
}

/****************************************************************************
 * Name: uart_open
 *
 * Description:
 *   This routine is called whenever a serial port is opened.
 *
 ****************************************************************************/

static int uart_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR uart_dev_t   *dev   = inode->i_private;
  uint8_t           tmp;
  int               ret;

  /* If the port is the middle of closing, wait until the close is finished.
   * If a signal is received while we are waiting, then return EINTR.
   */

  ret = uart_takesem(&dev->closesem, true);
  if (ret < 0)
    {
      /* A signal received while waiting for the last close operation. */

      return ret;
    }

#ifdef CONFIG_SERIAL_REMOVABLE
  /* If the removable device is no longer connected, refuse to open the
   * device.  We check this after obtaining the close semaphore because
   * we might have been waiting when the device was disconnected.
   */

  if (dev->disconnected)
    {
      ret = -ENOTCONN;
      goto errout_with_sem;
    }
#endif

  /* Start up serial port */

  /* Increment the count of references to the device. */

  tmp = dev->open_count + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Check if this is the first time that the driver has been opened. */

  if (tmp == 1)
    {
      irqstate_t flags = enter_critical_section();

      /* If this is the console, then the UART has already been
       * initialized.
       */

      if (!dev->isconsole)
        {
          /* Perform one time hardware initialization */

          ret = uart_setup(dev);
          if (ret < 0)
            {
              leave_critical_section(flags);
              goto errout_with_sem;
            }
        }

      /* In any event, we do have to configure for interrupt driven mode of
       * operation.  Attach the hardware IRQ(s). Hmm.. should shutdown() the
       * the device in the rare case that uart_attach() fails, tmp==1, and
       * this is not the console.
       */

      ret = uart_attach(dev);
      if (ret < 0)
        {
          uart_shutdown(dev);
          leave_critical_section(flags);
          goto errout_with_sem;
        }

#ifdef CONFIG_SERIAL_RXDMA
      /* Notify DMA that there is free space in the RX buffer */

      uart_dmarxfree(dev);
#endif

      /* Enable the RX interrupt */

      uart_enablerxint(dev);
      leave_critical_section(flags);
    }

  /* Save the new open count on success */

  dev->open_count = tmp;

errout_with_sem:
  uart_givesem(&dev->closesem);
  return ret;
}

/****************************************************************************
 * Name: uart_close
 *
 * Description:
 *   This routine is called when the serial port gets closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int uart_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR uart_dev_t   *dev   = inode->i_private;
  irqstate_t        flags;

  /* Get exclusive access to the close semaphore (to synchronize open/close
   * operations.
   * NOTE: that we do not let this wait be interrupted by a signal.
   * Technically, we should, but almost no one every checks the return value
   * from close() so we avoid a potential memory leak by ignoring signals in
   * this case.
   */

  uart_takesem(&dev->closesem, false);
  if (dev->open_count > 1)
    {
      dev->open_count--;
      uart_givesem(&dev->closesem);
      return OK;
    }

  /* There are no more references to the port */

  dev->open_count = 0;

  /* Stop accepting input */

  uart_disablerxint(dev);

  /* Prevent blocking if the device is opened with O_NONBLOCK */

  if ((filep->f_oflags & O_NONBLOCK) == 0)
    {
      /* Now we wait for the transmit buffer(s) to clear */

      uart_tcdrain(dev, false, 4 * TICK_PER_SEC);
    }

  /* Free the IRQ and disable the UART */

  flags = enter_critical_section();  /* Disable interrupts */
  uart_detach(dev);                  /* Detach interrupts */
  if (!dev->isconsole)               /* Check for the serial console UART */
    {
      uart_shutdown(dev);            /* Disable the UART */
    }

  leave_critical_section(flags);

  /* Wake up read and poll functions */

  uart_datareceived(dev);

  /* We need to re-initialize the semaphores if this is the last close
   * of the device, as the close might be caused by pthread_cancel() of
   * a thread currently blocking on any of them.
   */

  uart_reset_sem(dev);
  uart_givesem(&dev->closesem);
  return OK;
}

/****************************************************************************
 * Name: uart_read
 ****************************************************************************/

static ssize_t uart_read(FAR struct file *filep,
                         FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR uart_dev_t *dev = inode->i_private;
  FAR struct uart_buffer_s *rxbuf = &dev->recv;
#ifdef CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS
  unsigned int nbuffered;
  unsigned int watermark;
#endif
  irqstate_t flags;
  ssize_t recvd = 0;
  int16_t tail;
  char ch;
  int ret;

  /* Only one user can access rxbuf->tail at a time */

  ret = uart_takesem(&rxbuf->sem, true);
  if (ret < 0)
    {
      /* A signal received while waiting for access to the recv.tail will
       * abort the transfer.  After the transfer has started, we are
       * committed and signals will be ignored.
       */

      return ret;
    }

  /* Loop while we still have data to copy to the receive buffer.
   * we add data to the head of the buffer; uart_xmitchars takes the
   * data from the end of the buffer.
   */

  while ((size_t)recvd < buflen)
    {
#ifdef CONFIG_SERIAL_REMOVABLE
      /* If the removable device is no longer connected, refuse to read any
       * further from the device.
       */

      if (dev->disconnected)
        {
          if (recvd == 0)
            {
              recvd = -ENOTCONN;
            }

          break;
        }
#endif

      /* Check if there is more data to return in the circular buffer.
       * NOTE: Rx interrupt handling logic may asynchronously increment
       * the head index but must not modify the tail index.  The tail
       * index is only modified in this function.  Therefore, no
       * special handshaking is required here.
       *
       * The head and tail pointers are 16-bit values.  The only time that
       * the following could be unsafe is if the CPU made two non-atomic
       * 8-bit accesses to obtain the 16-bit head index.
       */

      tail = rxbuf->tail;
      if (rxbuf->head != tail)
        {
          /* Take the next character from the tail of the buffer */

          ch = rxbuf->buffer[tail];

          /* Increment the tail index.  Most operations are done using the
           * local variable 'tail' so that the final rxbuf->tail update
           * is atomic.
           */

          if (++tail >= rxbuf->size)
            {
              tail = 0;
            }

          rxbuf->tail = tail;

#ifdef CONFIG_SERIAL_TERMIOS
          /* Do input processing if any is enabled */

          if (dev->tc_iflag & (INLCR | IGNCR | ICRNL))
            {
              /* \n -> \r or \r -> \n translation? */

              if ((ch == '\n') && (dev->tc_iflag & INLCR))
                {
                  ch = '\r';
                }
              else if ((ch == '\r') && (dev->tc_iflag & ICRNL))
                {
                  ch = '\n';
                }

              /* Discarding \r ? */

              if ((ch == '\r') & (dev->tc_iflag & IGNCR))
                {
                  continue;
                }
            }

          /* Specifically not handled:
           *
           * All of the local modes; echo, line editing, etc.
           * Anything to do with break or parity errors.
           * ISTRIP - we should be 8-bit clean.
           * IUCLC - Not Posix
           * IXON/OXOFF - no xon/xoff flow control.
           */
#endif

          /* Store the received character */

          *buffer++ = ch;
          recvd++;
        }

#ifdef CONFIG_DEV_SERIAL_FULLBLOCKS
      /* No... then we would have to wait to get receive more data.
       * If the user has specified the O_NONBLOCK option, then just
       * return what we have.
       */

      else if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          /* If nothing was transferred, then return the -EAGAIN
           * error (not zero which means end of file).
           */

          if (recvd < 1)
            {
              recvd = -EAGAIN;
            }

          break;
        }
#else
      /* No... the circular buffer is empty.  Have we returned anything
       * to the caller?
       */

      else if (recvd > 0)
        {
          /* Yes.. break out of the loop and return the number of bytes
           * received up to the wait condition.
           */

          break;
        }

      else if (filep->f_inode == 0)
        {
          /* File has been closed.
           * Descriptor is not valid.
           */

          recvd = -EBADFD;
          break;
        }

      /* No... then we would have to wait to get receive some data.
       * If the user has specified the O_NONBLOCK option, then do not
       * wait.
       */

      else if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          /* Break out of the loop returning -EAGAIN */

          recvd = -EAGAIN;
          break;
        }
#endif

      /* Otherwise we are going to have to wait for data to arrive */

      else
        {
          /* Disable all interrupts and test again... */

          flags = enter_critical_section();

          /* Disable Rx interrupts and test again... */

          uart_disablerxint(dev);

          /* If the Rx ring buffer still empty?  Bytes may have been added
           * between the last time that we checked and when we disabled
           * interrupts.
           */

          if (rxbuf->head == rxbuf->tail)
            {
              /* Yes.. the buffer is still empty.  We will need to wait for
               * additional data to be received.
               */

#ifdef CONFIG_SERIAL_RXDMA
              /* Notify DMA that there is free space in the RX buffer */

              uart_dmarxfree(dev);
#endif
              /* Wait with the RX interrupt re-enabled.  All interrupts are
               * disabled briefly to assure that the following operations
               * are atomic.
               */

              /* Re-enable UART Rx interrupts */

              uart_enablerxint(dev);

              /* Check again if the RX buffer is empty.  The UART driver
               * might have buffered data received between disabling the
               * RX interrupt and entering the critical section.  Some
               * drivers (looking at you, cdcacm...) will push the buffer
               * to the receive queue during uart_enablerxint().
               * Just continue processing the RX queue if this happens.
               */

              if (rxbuf->head != rxbuf->tail)
                {
                  leave_critical_section(flags);
                  continue;
                }

#ifdef CONFIG_SERIAL_REMOVABLE
              /* Check again if the removable device is still connected
               * while we have interrupts off.  We do not want the transition
               * to occur as a race condition before we begin the wait.
               */

              if (dev->disconnected)
                {
                  ret = -ENOTCONN;
                }
              else
#endif
                {
                  /* Now wait with the Rx interrupt enabled.  NuttX will
                   * automatically re-enable global interrupts when this
                   * thread goes to sleep.
                   */

                  dev->recvwaiting = true;
                  ret = uart_takesem(&dev->recvsem, true);
                }

              leave_critical_section(flags);

              /* Was a signal received while waiting for data to be
               * received?  Was a removable device disconnected while
               * we were waiting?
               */

#ifdef CONFIG_SERIAL_REMOVABLE
              if (ret < 0 || dev->disconnected)
#else
              if (ret < 0)
#endif
                {
                  /* POSIX requires that we return after a signal is
                   * received.
                   * If some bytes were read, we need to return the
                   * number of bytes read; if no bytes were read, we
                   * need to return -1 with the errno set correctly.
                   */

                  if (recvd == 0)
                    {
                      /* No bytes were read, return -EINTR
                       * (the VFS layer will set the errno value
                       * appropriately).
                       */

#ifdef CONFIG_SERIAL_REMOVABLE
                      recvd = dev->disconnected ? -ENOTCONN : -EINTR;
#else
                      recvd = -EINTR;
#endif
                    }

                  break;
                }
            }
          else
            {
              /* No... the ring buffer is no longer empty.  Just re-enable Rx
               * interrupts and accept the new data on the next time through
               * the loop.
               */

              leave_critical_section(flags);

              uart_enablerxint(dev);
            }
        }
    }

#ifdef CONFIG_SERIAL_RXDMA
  /* Notify DMA that there is free space in the RX buffer */

  flags = enter_critical_section();
  uart_dmarxfree(dev);
  leave_critical_section(flags);
#endif

  /* RX interrupt could be disabled by RX buffer overflow. Enable it now. */

  uart_enablerxint(dev);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
#ifdef CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS
  /* How many bytes are now buffered */

  rxbuf = &dev->recv;
  if (rxbuf->head >= rxbuf->tail)
    {
      nbuffered = rxbuf->head - rxbuf->tail;
    }
  else
    {
      nbuffered = rxbuf->size - rxbuf->tail + rxbuf->head;
    }

  /* Is the level now below the watermark level that we need to report? */

  watermark = (CONFIG_SERIAL_IFLOWCONTROL_LOWER_WATERMARK *
               rxbuf->size) / 100;
  if (nbuffered <= watermark)
    {
      /* Let the lower level driver know that the watermark level has been
       * crossed.  It will probably deactivate RX flow control.
       */

      uart_rxflowcontrol(dev, nbuffered, false);
    }
#else
  /* Is the RX buffer empty? */

  if (rxbuf->head == rxbuf->tail)
    {
      /* Deactivate RX flow control. */

      uart_rxflowcontrol(dev, 0, false);
    }
#endif
#endif

  uart_givesem(&dev->recv.sem);
  return recvd;
}

/****************************************************************************
 * Name: uart_write
 ****************************************************************************/

static ssize_t uart_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode    = filep->f_inode;
  FAR uart_dev_t   *dev      = inode->i_private;
  ssize_t           nwritten = buflen;
  bool              oktoblock;
  int               ret;
  char              ch;

  /* We may receive serial writes through this path from interrupt handlers
   * and from debug output in the IDLE task!  In these cases, we will need to
   * do things a little differently.
   */

  if (up_interrupt_context() || sched_idletask())
    {
      irqstate_t flags;

#ifdef CONFIG_SERIAL_REMOVABLE
      /* If the removable device is no longer connected, refuse to write to
       * the device.
       */

      if (dev->disconnected)
        {
          return -ENOTCONN;
        }
#endif

      flags = enter_critical_section();
      ret = uart_irqwrite(dev, buffer, buflen);
      leave_critical_section(flags);

      return ret;
    }

  /* Only one user can access dev->xmit.head at a time */

  ret = (ssize_t)uart_takesem(&dev->xmit.sem, true);
  if (ret < 0)
    {
      /* A signal received while waiting for access to the xmit.head will
       * abort the transfer.  After the transfer has started, we are
       * committed and signals will be ignored.
       */

      return ret;
    }

#ifdef CONFIG_SERIAL_REMOVABLE
  /* If the removable device is no longer connected, refuse to write to the
   * device.  This check occurs after taking the xmit.sem because the
   * disconnection event might have occurred while we were waiting for
   * access to the transmit buffers.
   */

  if (dev->disconnected)
    {
      uart_givesem(&dev->xmit.sem);
      return -ENOTCONN;
    }
#endif

  /* Can the following loop block, waiting for space in the TX
   * buffer?
   */

  oktoblock = ((filep->f_oflags & O_NONBLOCK) == 0);

  /* Loop while we still have data to copy to the transmit buffer.
   * we add data to the head of the buffer; uart_xmitchars takes the
   * data from the end of the buffer.
   */

  uart_disabletxint(dev);
  for (; buflen; buflen--)
    {
      ch  = *buffer++;
      ret = OK;

#ifdef CONFIG_SERIAL_TERMIOS
      /* Do output post-processing */

      if ((dev->tc_oflag & OPOST) != 0)
        {
          /* Mapping CR to NL? */

          if ((ch == '\r') && (dev->tc_oflag & OCRNL) != 0)
            {
              ch = '\n';
            }

          /* Are we interested in newline processing? */

          if ((ch == '\n') && (dev->tc_oflag & (ONLCR | ONLRET)) != 0)
            {
              ret = uart_putxmitchar(dev, '\r', oktoblock);
            }

          /* Specifically not handled:
           *
           * OXTABS - primarily a full-screen terminal optimization
           * ONOEOT - Unix interoperability hack
           * OLCUC  - Not specified by POSIX
           * ONOCR  - low-speed interactive optimization
           */
        }

#else /* !CONFIG_SERIAL_TERMIOS */
      /* If this is the console, convert \n -> \r\n */

      if (dev->isconsole && ch == '\n')
        {
          ret = uart_putxmitchar(dev, '\r', oktoblock);
        }
#endif

      /* Put the character into the transmit buffer */

      if (ret >= 0)
        {
          ret = uart_putxmitchar(dev, ch, oktoblock);
        }

      /* uart_putxmitchar() might return an error under one of two
       * conditions:  (1) The wait for buffer space might have been
       * interrupted by a signal (ret should be -EINTR), (2) if
       * CONFIG_SERIAL_REMOVABLE is defined, then uart_putxmitchar()
       * might also return if the serial device was disconnected
       * (with -ENOTCONN), or (3) if O_NONBLOCK is specified, then
       * then uart_putxmitchar() might return -EAGAIN if the output
       * TX buffer is full.
       */

      if (ret < 0)
        {
          /* POSIX requires that we return -1 and errno set if no data was
           * transferred.  Otherwise, we return the number of bytes in the
           * interrupted transfer.
           */

          if (buflen < (size_t)nwritten)
            {
              /* Some data was transferred.  Return the number of bytes that
               * were successfully transferred.
               */

              nwritten -= buflen;
            }
          else
            {
              /* No data was transferred. Return the negated errno value.
               * The VFS layer will set the errno value appropriately).
               */

              nwritten = ret;
            }

          break;
        }
    }

  if (dev->xmit.head != dev->xmit.tail)
    {
#ifdef CONFIG_SERIAL_TXDMA
      uart_dmatxavail(dev);
#endif
      uart_enabletxint(dev);
    }

  uart_givesem(&dev->xmit.sem);
  return nwritten;
}

/****************************************************************************
 * Name: uart_ioctl
 ****************************************************************************/

static int uart_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR uart_dev_t   *dev   = inode->i_private;

  /* Handle TTY-level IOCTLs here */

  /* Let low-level driver handle the call first */

  int ret = dev->ops->ioctl ? dev->ops->ioctl(filep, cmd, arg) : -ENOTTY;

  /* The device ioctl() handler returns -ENOTTY when it doesn't know
   * how to handle the command. Check if we can handle it here.
   */

  if (ret == -ENOTTY)
    {
      switch (cmd)
        {
          /* Get the number of bytes that may be read from the RX buffer
           * (without waiting)
           */

          case FIONREAD:
            {
              int count;
              irqstate_t flags = enter_critical_section();

              /* Determine the number of bytes available in the RX buffer */

              if (dev->recv.tail <= dev->recv.head)
                {
                  count = dev->recv.head - dev->recv.tail;
                }
              else
                {
                  count = dev->recv.size - (dev->recv.tail - dev->recv.head);
                }

              leave_critical_section(flags);

              *(FAR int *)((uintptr_t)arg) = count;
              ret = 0;
            }
            break;

          /* Get the number of bytes that have been written to the TX
           * buffer.
           */

          case FIONWRITE:
            {
              int count;
              irqstate_t flags = enter_critical_section();

              /* Determine the number of bytes waiting in the TX buffer */

              if (dev->xmit.tail <= dev->xmit.head)
                {
                  count = dev->xmit.head - dev->xmit.tail;
                }
              else
                {
                  count = dev->xmit.size - (dev->xmit.tail - dev->xmit.head);
                }

              leave_critical_section(flags);

              *(FAR int *)((uintptr_t)arg) = count;
              ret = 0;
            }
            break;

          /* Get the number of free bytes in the TX buffer */

          case FIONSPACE:
            {
              int count;
              irqstate_t flags = enter_critical_section();

              /* Determine the number of bytes free in the TX buffer */

              if (dev->xmit.head < dev->xmit.tail)
                {
                  count = dev->xmit.tail - dev->xmit.head - 1;
                }
              else
                {
                  count = dev->xmit.size -
                         (dev->xmit.head - dev->xmit.tail) - 1;
                }

              leave_critical_section(flags);

              *(FAR int *)((uintptr_t)arg) = count;
              ret = 0;
            }
            break;

          case TCFLSH:
            {
              /* Empty the tx/rx buffers */

              irqstate_t flags = enter_critical_section();

              if (arg == TCIFLUSH || arg == TCIOFLUSH)
                {
                  dev->recv.tail = dev->recv.head;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
                  /* De-activate RX flow control. */

                  uart_rxflowcontrol(dev, 0, false);
#endif
                }

              if (arg == TCOFLUSH || arg == TCIOFLUSH)
                {
                  dev->xmit.tail = dev->xmit.head;

                  /* Inform any waiters there there is space available. */

                  uart_datasent(dev);
                }

              leave_critical_section(flags);
              ret = 0;
            }
            break;

          case TCDRN:
            {
              ret = uart_tcdrain(dev, true, 10 * TICK_PER_SEC);
            }
            break;

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP)
          /* Make the controlling terminal of the calling process */

          case TIOCSCTTY:
            {
              /* Save the PID of the recipient of the SIGINT signal. */

              if ((int)arg < 0 || dev->pid >= 0)
                {
                  ret = -EINVAL;
                }
              else
                {
                  dev->pid = (pid_t)arg;
                  ret = 0;
                }
            }
            break;

          case TIOCNOTTY:
            {
              dev->pid = INVALID_PROCESS_ID;
              ret = 0;
            }
            break;
#endif
        }
    }

#ifdef CONFIG_SERIAL_TERMIOS
  /* Append any higher level TTY flags */

  if (ret == OK || ret == -ENOTTY)
    {
      switch (cmd)
        {
          case TCGETS:
            {
              FAR struct termios *termiosp = (FAR struct termios *)arg;

              if (!termiosp)
                {
                  ret = -EINVAL;
                  break;
                }

              /* And update with flags from this layer */

              termiosp->c_iflag = dev->tc_iflag;
              termiosp->c_oflag = dev->tc_oflag;
              termiosp->c_lflag = dev->tc_lflag;

              ret = 0;
            }
            break;

          case TCSETS:
            {
              FAR struct termios *termiosp = (FAR struct termios *)arg;

              if (!termiosp)
                {
                  ret = -EINVAL;
                  break;
                }

              /* Update the flags we keep at this layer */

              dev->tc_iflag = termiosp->c_iflag;
              dev->tc_oflag = termiosp->c_oflag;
              dev->tc_lflag = termiosp->c_lflag;

              ret = 0;
            }
            break;
        }
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: uart_poll
 ****************************************************************************/

static int uart_poll(FAR struct file *filep,
                     FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR uart_dev_t   *dev   = inode->i_private;
  pollevent_t       eventset;
  int               ndx;
  int               ret;
  int               i;

  /* Some sanity checking */

#ifdef CONFIG_DEBUG_FEATURES
  if (dev == NULL || fds == NULL)
    {
      return -ENODEV;
    }
#endif

  /* Are we setting up the poll?  Or tearing it down? */

  ret = uart_takesem(&dev->pollsem, true);
  if (ret < 0)
    {
      /* A signal received while waiting for access to the poll data
       * will abort the operation.
       */

      return ret;
    }

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_SERIAL_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!dev->fds[i])
            {
              /* Bind the poll structure and this slot */

              dev->fds[i]  = fds;
              fds->priv    = &dev->fds[i];
              break;
            }
        }

      if (i >= CONFIG_SERIAL_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events?
       * First, check if the xmit buffer is full.
       *
       * Get exclusive access to the xmit buffer indices.
       * NOTE: that we do not let this wait be interrupted by a signal
       * (we probably should, but that would be a little awkward).
       */

      eventset = 0;
      uart_takesem(&dev->xmit.sem, false);

      ndx = dev->xmit.head + 1;
      if (ndx >= dev->xmit.size)
        {
          ndx = 0;
        }

      if (ndx != dev->xmit.tail)
        {
          eventset |= POLLOUT;
        }

      uart_givesem(&dev->xmit.sem);

      /* Check if the receive buffer is empty.
       *
       * Get exclusive access to the recv buffer indices.
       * NOTE: that we do not let this wait be interrupted by a signal
       * (we probably should, but that would be a little awkward).
       */

      uart_takesem(&dev->recv.sem, false);
      if (dev->recv.head != dev->recv.tail)
        {
          eventset |= POLLIN;
        }

      uart_givesem(&dev->recv.sem);

#ifdef CONFIG_SERIAL_REMOVABLE
      /* Check if a removable device has been disconnected. */

      if (dev->disconnected)
        {
           eventset |= (POLLERR | POLLHUP);
        }
#endif

      poll_notify(dev->fds, CONFIG_SERIAL_NPOLLWAITERS, eventset);
    }
  else if (fds->priv != NULL)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG_FEATURES
      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  uart_givesem(&dev->pollsem);
  return ret;
}

/****************************************************************************
 * Name: uart_nxsched_foreach_cb
 ****************************************************************************/

#ifdef CONFIG_TTY_LAUNCH
static void uart_launch_foreach(FAR struct tcb_s *tcb, FAR void *arg)
{
#ifdef CONFIG_TTY_LAUNCH_ENTRY
  if (!strcmp(tcb->name, CONFIG_TTY_LAUNCH_ENTRYNAME))
#else
  if (!strcmp(tcb->name, CONFIG_TTY_LAUNCH_FILEPATH))
#endif
    {
      *(int *)arg = 1;
    }
}

static void uart_launch_worker(void *arg)
{
#ifdef CONFIG_TTY_LAUNCH_ARGS
  FAR char *const argv[] =
  {
    CONFIG_TTY_LAUNCH_ARGS,
    NULL,
  };
#else
  FAR char *const *argv = NULL;
#endif
  int found = 0;

  nxsched_foreach(uart_launch_foreach, &found);
  if (!found)
    {
#ifdef CONFIG_TTY_LAUNCH_ENTRY
      nxtask_create(CONFIG_TTY_LAUNCH_ENTRYNAME,
                    CONFIG_TTY_LAUNCH_PRIORITY,
                    CONFIG_TTY_LAUNCH_STACKSIZE,
                    (main_t)CONFIG_TTY_LAUNCH_ENTRYPOINT,
                    argv);
#else
      posix_spawnattr_t attr;

      posix_spawnattr_init(&attr);

      attr.priority  = CONFIG_TTY_LAUNCH_PRIORITY;
      attr.stacksize = CONFIG_TTY_LAUNCH_STACKSIZE;
      exec_spawn(CONFIG_TTY_LAUNCH_FILEPATH, argv, NULL, 0, &attr);
#endif
    }
}

static void uart_launch(void)
{
  work_queue(HPWORK, &g_serial_work, uart_launch_worker, NULL, 0);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_register
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ****************************************************************************/

int uart_register(FAR const char *path, FAR uart_dev_t *dev)
{
#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP)
  /* Initialize  of the task that will receive SIGINT signals. */

  dev->pid = INVALID_PROCESS_ID;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
  /* If this UART is a serial console */

  if (dev->isconsole)
    {
      /* Enable signals by default */

      dev->tc_lflag |= ISIG;

      /* Enable \n -> \r\n translation for the console */

      dev->tc_oflag = OPOST | ONLCR;
    }
#endif

  /* Initialize semaphores */

  nxsem_init(&dev->xmit.sem, 0, 1);
  nxsem_init(&dev->recv.sem, 0, 1);
  nxsem_init(&dev->closesem, 0, 1);
  nxsem_init(&dev->xmitsem,  0, 0);
  nxsem_init(&dev->recvsem,  0, 0);
  nxsem_init(&dev->pollsem,  0, 1);

  /* The recvsem and xmitsem are used for signaling and, hence, should
   * not have priority inheritance enabled.
   */

  nxsem_set_protocol(&dev->xmitsem, SEM_PRIO_NONE);
  nxsem_set_protocol(&dev->recvsem, SEM_PRIO_NONE);

  /* Register the serial driver */

  sinfo("Registering %s\n", path);
  return register_driver(path, &g_serialops, 0666, dev);
}

/****************************************************************************
 * Name: uart_datareceived
 *
 * Description:
 *   This function is called from uart_recvchars when new serial data is
 *   place in the driver's circular buffer.  This function will wake-up any
 *   stalled read() operations that are waiting for incoming data.
 *
 ****************************************************************************/

void uart_datareceived(FAR uart_dev_t *dev)
{
  /* Notify all poll/select waiters that they can read from the recv buffer */

  poll_notify(dev->fds, CONFIG_SERIAL_NPOLLWAITERS, POLLIN);

  /* Is there a thread waiting for read data?  */

  if (dev->recvwaiting)
    {
      /* Yes... wake it up */

      dev->recvwaiting = false;
      nxsem_post(&dev->recvsem);
    }

#if defined(CONFIG_PM) && defined(CONFIG_SERIAL_CONSOLE)
  /* Call pm_activity when characters are received on the console device */

  if (dev->isconsole)
    {
      pm_activity(CONFIG_SERIAL_PM_ACTIVITY_DOMAIN,
                  CONFIG_SERIAL_PM_ACTIVITY_PRIORITY);
    }
#endif
}

/****************************************************************************
 * Name: uart_datasent
 *
 * Description:
 *   This function is called from uart_xmitchars after serial data has been
 *   sent, freeing up some space in the driver's circular buffer. This
 *   function will wake-up any stalled write() operations that was waiting
 *   for space to buffer outgoing data.
 *
 ****************************************************************************/

void uart_datasent(FAR uart_dev_t *dev)
{
  /* Notify all poll/select waiters that they can write to xmit buffer */

  poll_notify(dev->fds, CONFIG_SERIAL_NPOLLWAITERS, POLLOUT);

  /* Is there a thread waiting for space in xmit.buffer?  */

  if (dev->xmitwaiting)
    {
      /* Yes... wake it up */

      dev->xmitwaiting = false;
      nxsem_post(&dev->xmitsem);
    }
}

/****************************************************************************
 * Name: uart_connected
 *
 * Description:
 *   Serial devices (like USB serial) can be removed.
 *   In that case, the "upper half" serial driver must be informed that there
 *   is no longer a valid serial channel associated with the driver.
 *
 *   In this case, the driver will terminate all pending transfers wint
 *   ENOTCONN and will refuse all further transactions while the "lower half"
 *   is disconnected.
 *   The driver will continue to be registered, but will be in an unusable
 *   state.
 *
 *   Conversely, the "upper half" serial driver needs to know when the serial
 *   device is reconnected so that it can resume normal operations.
 *
 * Assumptions/Limitations:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_REMOVABLE
void uart_connected(FAR uart_dev_t *dev, bool connected)
{
  irqstate_t flags;

  /* Is the device disconnected?  Interrupts are disabled because this
   * function may be called from interrupt handling logic.
   */

  flags = enter_critical_section();
  dev->disconnected = !connected;
  if (!connected)
    {
      /* Notify all poll/select waiters that a hangup occurred */

      poll_notify(dev->fds, CONFIG_SERIAL_NPOLLWAITERS, POLLERR | POLLHUP);

      /* Yes.. wake up all waiting threads.  Each thread should detect the
       * disconnection and return the ENOTCONN error.
       */

      /* Is there a thread waiting for space in xmit.buffer?  */

      if (dev->xmitwaiting)
        {
          /* Yes... wake it up */

          dev->xmitwaiting = false;
          nxsem_post(&dev->xmitsem);
        }

      /* Is there a thread waiting for read data?  */

      if (dev->recvwaiting)
        {
          /* Yes... wake it up */

          dev->recvwaiting = false;
          nxsem_post(&dev->recvsem);
        }
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: uart_reset_sem
 *
 * Description:
 *   This function is called when need reset uart semaphore, this may used in
 *   kill one process, but this process was reading/writing with the
 *   semaphore.
 *
 ****************************************************************************/

void uart_reset_sem(FAR uart_dev_t *dev)
{
  nxsem_reset(&dev->xmitsem,  0);
  nxsem_reset(&dev->recvsem,  0);
  nxsem_reset(&dev->xmit.sem, 1);
  nxsem_reset(&dev->recv.sem, 1);
  nxsem_reset(&dev->pollsem,  1);
}

/****************************************************************************
 * Name: uart_check_special
 *
 * Description:
 *   Check if the SIGINT or SIGTSTP character is in the contiguous Rx DMA
 *   buffer region.  The first signal associated with the first such
 *   character is returned.
 *
 *   If there multiple such characters in the buffer, only the signal
 *   associated with the first is returned (this a bug!)
 *
 * Returned Value:
 *   0 if a signal-related character does not appear in the.  Otherwise,
 *   SIGKILL or SIGTSTP may be returned to indicate the appropriate signal
 *   action.
 *
 ****************************************************************************/

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP) || \
    defined(CONFIG_TTY_FORCE_PANIC) || defined(CONFIG_TTY_LAUNCH)
int uart_check_special(FAR uart_dev_t *dev, const char *buf, size_t size)
{
  size_t i;

#ifdef CONFIG_SERIAL_TERMIOS
  if ((dev->tc_lflag & ISIG) == 0)
#else
  if (!dev->isconsole)
#endif
    {
      return 0;
    }

  for (i = 0; i < size; i++)
    {
#ifdef CONFIG_TTY_FORCE_PANIC
      if (buf[i] == CONFIG_TTY_FORCE_PANIC_CHAR)
        {
          PANIC();
          return 0;
        }
#endif

#ifdef CONFIG_TTY_LAUNCH
      if (buf[i] == CONFIG_TTY_LAUNCH_CHAR)
        {
          uart_launch();
          return 0;
        }
#endif

#ifdef CONFIG_TTY_SIGINT
      if (dev->pid > 0 && buf[i] == CONFIG_TTY_SIGINT_CHAR)
        {
          return SIGINT;
        }
#endif

#ifdef CONFIG_TTY_SIGTSTP
      if (dev->pid > 0 && buf[i] == CONFIG_TTY_SIGTSTP_CHAR)
        {
          return SIGTSTP;
        }
#endif
    }

  return 0;
}
#endif
