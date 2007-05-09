/************************************************************************************
 * serial.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs.h>
#include <nuttx/serial.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#ifdef CONFIG_HAVE_LOWPUTC
# define uart_putc(ch) up_lowputc(ch);
#else
# define uart_putc(ch) up_putc(ch);
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static int     uart_open(struct file *filep);
static int     uart_close(struct file *filep);
static ssize_t uart_read(struct file *filep, char *buffer, size_t buflen);
static ssize_t uart_write(struct file *filep, const char *buffer, size_t buflen);
static int     uart_ioctl(struct file *filep, int cmd, unsigned long arg);

/************************************************************************************
 * Private Variables
 ************************************************************************************/

struct file_operations g_serialops =
{
  uart_open,  /* open */
  uart_close, /* close */
  uart_read,  /* read */
  uart_write, /* write */
  0,          /* seek */
  uart_ioctl  /* ioctl */
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: uart_takesem
 ************************************************************************************/

static void uart_takesem(sem_t *sem)
{
  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

/************************************************************************************
 * Name: uart_givesem
 ************************************************************************************/

static inline void uart_givesem(sem_t *sem)
{
  (void)sem_post(sem);
}

/************************************************************************************
 * Name: uart_putxmitchar
 ************************************************************************************/

static void uart_putxmitchar(uart_dev_t *dev, int ch)
{
  int nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  for(;;)
    {
      if (nexthead != dev->xmit.tail)
        {
          dev->xmit.buffer[dev->xmit.head] = ch;
          dev->xmit.head = nexthead;
          return;
        }
      else
        {
          /* Inform the interrupt level logic that we are waiting */

          dev->xmitwaiting = TRUE;

          /* Wait for some characters to be sent from the buffer
           * with the TX interrupt enabled.  When the TX interrupt
           * is enabled, uart_xmitchars should execute and remove
           * some of the data from the TX buffer.
           */

          uart_enabletxint(dev);
          uart_takesem(&dev->xmitsem);
          uart_disabletxint(dev);
        }
    }
}

/************************************************************************************
 * Name: uart_irqwrite
 ************************************************************************************/

static ssize_t uart_irqwrite(uart_dev_t *dev, const char *buffer, size_t buflen)
{
  ssize_t ret = buflen;

  /* Force each character through the low level interface */

  for (; buflen; buflen--)
    {
      int ch = *buffer++;
      uart_putc(ch);

     /* If this is the console, then we should replace LF with LF-CR */

      if (ch == '\n')
        {
          uart_putc('\r');
        }
    }

  return ret;
}

/************************************************************************************
 * Name: uart_write
 ************************************************************************************/

static ssize_t uart_write(struct file *filep, const char *buffer, size_t buflen)
{
  struct inode *inode    = filep->f_inode;
  uart_dev_t   *dev      = inode->i_private;
  ssize_t       ret      = buflen;

  /* We may receive console writes through this path from
   * interrupt handlers and from debug output in the IDLE task!
   * In these cases, we will need to do things a little
   * differently.
   */

  if (up_interrupt_context() || getpid() == 0)
    {
      if (dev->isconsole)
        {
          irqstate_t flags = irqsave();
          ret = uart_irqwrite(dev, buffer, buflen);
          irqrestore(flags);
          return ret;
        }
      else
        {
          return ERROR;
        }
    }

  /* Only one user can be accessing dev->xmit.head at once */

  uart_takesem(&dev->xmit.sem);

  /* Loop while we still have data to copy to the transmit buffer.
   * we add data to the head of the buffer; uart_xmitchars takes the
   * data from the end of the buffer.
   */

  uart_disabletxint(dev);
  for (; buflen; buflen--)
    {
      int ch = *buffer++;

      /* Put the character into the transmit buffer */

      uart_putxmitchar(dev, ch);

     /* If this is the console, then we should replace LF with LF-CR */

      if (dev->isconsole && ch == '\n')
        {
          uart_putxmitchar(dev, '\r');
        }
    }

  if (dev->xmit.head != dev->xmit.tail)
    {
      uart_enabletxint(dev);
    }

  uart_givesem(&dev->xmit.sem);
  return ret;
}

/************************************************************************************
 * Name: uart_read
 ************************************************************************************/

static ssize_t uart_read(struct file *filep, char *buffer, size_t buflen)
{
  struct inode *inode = filep->f_inode;
  uart_dev_t     *dev   = inode->i_private;
  ssize_t       ret   = buflen;

  /* Only one user can be accessing dev->recv.tail at once */

  uart_takesem(&dev->recv.sem);

  /* Loop while we still have data to copy to the receive buffer.
   * we add data to the head of the buffer; uart_xmitchars takes the
   * data from the end of the buffer.
   */

  uart_disablerxint(dev);
  while (buflen)
    {
      if (dev->recv.head != dev->recv.tail)
        {
          *buffer++ = dev->recv.buffer[dev->recv.tail];
          buflen--;

          if (++(dev->recv.tail) >= dev->recv.size)
            {
              dev->recv.tail = 0;
            }
        }
      else
        {
          /* Wait for some characters to be sent from the buffer
           * with the TX interrupt disabled.
           */

          dev->recvwaiting = TRUE;
          uart_enablerxint(dev);
          uart_takesem(&dev->recvsem);
          uart_disablerxint(dev);
        }
    }

  uart_enablerxint(dev);
  uart_givesem(&dev->recv.sem);
  return ret;
}

/************************************************************************************
 * Name: uart_ioctl
 ************************************************************************************/

static int uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  uart_dev_t   *dev   = inode->i_private;

  return dev->ops->ioctl(filep, cmd, arg);
}

/************************************************************************************
 * Name: uart_close
 *
 * Description:
 *   This routine is called when the serial port gets closed.
 *   It waits for the last remaining data to be sent.
 *
 ************************************************************************************/

static int uart_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  uart_dev_t   *dev   = inode->i_private;
  irqstate_t    flags;

  uart_takesem(&dev->closesem);
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

  /* Now we wait for the transmit buffer to clear */

  while (dev->xmit.head != dev->xmit.tail)
    {
      usleep(500*1000);
    }

  /* And wait for the TX fifo to drain */

  while (!uart_txfifoempty(dev))
    {
      usleep(500*1000);
    }

  /* Free the IRQ and disable the UART */

  flags = irqsave(); /* Disable interrupts */
  up_disable_irq(dev->irq);
  irq_detach(dev->irq);
  uart_shutdown(dev);
  irqrestore(flags);

  uart_givesem(&dev->closesem);
  return OK;
 }

/************************************************************************************
 * Name: uart_open
 *
 * Description:
 *   This routine is called whenever a serial port is opened.
 *
 ************************************************************************************/

static int uart_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  uart_dev_t     *dev   = inode->i_private;
  int           ret   = OK;

  /* If the port is the middle of closing, wait until the close
   * is finished
   */

  uart_takesem(&dev->closesem);

  /* Start up serial port */

  if (++dev->open_count == 1)
    {
      irqstate_t flags = irqsave();

      /* If this is the console, then the UART has already
       * been initialized.
       */

      if (!dev->isconsole)
        {
          uart_setup(dev);
        }

      /* But, in any event, we do have to configure for
       * interrupt driven mode of operation.
       */

      /* Attache and enabled the IRQ */

      ret = irq_attach(dev->irq, dev->ops->handler);
      if (ret == OK)
        {
          /* Mark the io buffers empty */

          dev->xmit.head = 0;
          dev->xmit.tail = 0;
          dev->recv.head = 0;
          dev->recv.tail = 0;

          /* Finally, enable interrupts */

          up_enable_irq(dev->irq);
          uart_enablerxint(dev);
        }
      irqrestore(flags);
    }

  uart_givesem(&dev->closesem);
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: uart_register
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ************************************************************************************/

int uart_register(const char *path, uart_dev_t *dev)
{
  sem_init(&dev->xmit.sem, 0, 1);
  sem_init(&dev->recv.sem, 0, 1);
  sem_init(&dev->closesem, 0, 1);
  sem_init(&dev->xmitsem,  0, 0);
  sem_init(&dev->recvsem,  0, 0);

  dbg("Registering %s\n", path);
  return register_driver(path, &g_serialops, 0666, dev);
}

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

void uart_xmitchars(uart_dev_t *dev)
{
  /* Send while we still have data & room in the fifo */

  while (dev->xmit.head != dev->xmit.tail && uart_txfifonotfull(dev))
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

void uart_recvchars(uart_dev_t *dev)
{
  unsigned int status;
  int nexthead = dev->recv.head + 1;

  if (nexthead >= dev->recv.size)
    {
      nexthead = 0;
    }

  while (nexthead != dev->recv.tail && uart_rxfifonotempty(dev))
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

