/************************************************************************************
 * serial.h
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

#ifndef __SERIAL_H
#define __SERIAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <semaphore.h>
#include <nuttx/fs.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#define uart_setup(dev)          dev->ops->setup(dev)
#define uart_shutdown(dev)       dev->ops->shutdown(dev)
#define uart_enabletxint(dev)    dev->ops->txint(dev, TRUE)
#define uart_disabletxint(dev)   dev->ops->txint(dev, FALSE)
#define uart_enablerxint(dev)    dev->ops->rxint(dev, TRUE)
#define uart_disablerxint(dev)   dev->ops->rxint(dev, FALSE)
#define uart_rxfifonotempty(dev) dev->ops->rxfifonotempty(dev)
#define uart_txfifonotfull(dev)  dev->ops->txfifonotfull(dev)
#define uart_txfifoempty(dev)    dev->ops->txfifoempty(dev)
#define uart_send(dev,ch)        dev->ops->send(dev,ch)
#define uart_receive(dev,s)      dev->ops->receive(dev,s)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This structure defines one serial I/O buffer.  The serial infrastructure will
 * initialize the 'sem' field but all other fields must be initialized by the
 * caller of uart_register().
 */

struct uart_buffer_s
{
  sem_t     sem;    /* Used to control exclusive access to the buffer */
  sint16    head;   /* Index to the head [IN] index in the buffer */
  sint16    tail;   /* Index to the tail [OUT] index in the buffer */
  sint16    size;   /* The allocated size of the buffer */
  FAR char *buffer; /* Pointer to the allocated buffer memory */
};

/* This structure defines all of the operations providd by the architecture specific
 * logic.  All fields must be provided with non-NULL function pointers by the
 * caller of uart_register().
 */

struct uart_dev_s;
struct uart_ops_s
{
  /* Configure the UART baud, bits, parity, fifos, etc. This method is called
   * the first time that the serial port is opened.
   */

  int (*setup)(struct uart_dev_s *dev);

  /* Disable the UART.  This method is called when the serial port is closed */

  void (*shutdown)(struct uart_dev_s *dev);

  /* This is the UART interrupt handler.  It will be invoked when an interrupt
   * received on the 'irq'  It should call uart_transmitchars or uart_receivechar
   * to perform the appropriate data transfers.  The interrupt handling logic\
   * must be able to map the 'irq' number into the approprite uart_dev_s
   * structure in order to call these functions.
   */

  int (*handler)(int irq, void *context);

  /* All ioctl calls will be routed through this method */

  int (*ioctl)(struct file *filep, int cmd, unsigned long arg);

  /* Called (usually) from the interrupt level to receive one character from
   * the UART.  Error bits associated with the receipt are provided in the
   * the return 'status'.
   */

  int (*receive)(struct uart_dev_s *dev, unsigned int *status);

  /* Call to enable or disable RX interrupts */

  void (*rxint)(struct uart_dev_s *dev, boolean enable);

  /* Return TRUE if the receive fifo is not empty */

  boolean (*rxfifonotempty)(struct uart_dev_s *dev);

  /* This method will send one byte on the UART */

  void (*send)(struct uart_dev_s *dev, int ch);

  /* Call to enable or disable TX interrupts */

  void (*txint)(struct uart_dev_s *dev, boolean enable);

  /* Return TRUE if the tranmsit fifo is not full.  This is used to
   * determine if *send can be called.
   */

  boolean (*txfifonotfull)(struct uart_dev_s *dev);

  /* Return TRUE if the transmit fifo is empty.  This is used when the
   * driver needs to make sure that all characters are "drained" from
   * the TX fifo.
   */

  boolean (*txfifoempty)(struct uart_dev_s *dev);
};

/* This is the device structure used by the driver.  The caller of
 * uart_register() must allocate and initialize this structure.  The
 * calling logic need only set all fields to zero except:
 *
 *   'irq', 'isconsole', 'xmit.buffer', 'rcv.buffer', the elements
 *   of 'ops', and 'private'
 *
 * The common logic will initialize all semaphores.
 */

struct uart_dev_s
{
  int       open_count;			/* The number of times
					* the device has been opened */
  ubyte     irq;			/* IRQ associated with
					 * this UART */
  boolean   xmitwaiting;		/* TRUE: User is waiting
					 * for space in xmit.buffer */
  boolean   recvwaiting;		/* TRUE: User is waiting
					 * for space in recv.buffer */
  boolean   isconsole;			/* TRUE: This is the serial console */
  sem_t     closesem;			/* Locks out new opens while
					 * close is in progress */
  sem_t     xmitsem;			/* Used to wakeup user waiting
					 * for space in xmit.buffer */
  sem_t     recvsem;			/* Used to wakeup user waiting
					 * for sapce in recv.buffer */
  struct uart_buffer_s xmit;		/* Describes transmit buffer */
  struct uart_buffer_s recv;		/* Describes receive buffer */
  const struct uart_ops_s *ops;		/* Arch-specifics operations */
  void     *priv;			/* Used by the arch-specific logic */
};
typedef struct uart_dev_s uart_dev_t;

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: uart_register
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ************************************************************************************/

EXTERN int uart_register(const char *path, uart_dev_t *dev);

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

EXTERN void uart_xmitchars(uart_dev_t *dev);

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

EXTERN void uart_recvchars(uart_dev_t *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __SERIAL_H */
