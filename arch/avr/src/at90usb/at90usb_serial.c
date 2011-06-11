/****************************************************************************
 * arch/avr/src/at90usb/at90usb_serial.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "at90usb_config.h"

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"
#include "at90usb_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there at least one USART enabled and configured as a RS-232 device? */

#ifndef HAVE_USART_DEVICE
#  warning "No USARTs enabled as RS-232 devices"
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef CONFIG_USE_SERIALDRIVER

#if defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart1port     /* USART1 is console */
#endif
#define TTYS0_DEV         g_usart1port     /* USART1 is ttyS0 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  usart1_setup(struct uart_dev_s *dev);
static void usart1_shutdown(struct uart_dev_s *dev);
static int  usart1_attach(struct uart_dev_s *dev);
static void usart1_detach(struct uart_dev_s *dev);
static int  usart1_interrupt(int irq, void *context);
static int  usart1_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  usart1_receive(struct uart_dev_s *dev, uint32_t *status);
static void usart1_rxint(struct uart_dev_s *dev, bool enable);
static bool usart1_rxavailable(struct uart_dev_s *dev);
static void usart1_send(struct uart_dev_s *dev, int ch);
static void usart1_txint(struct uart_dev_s *dev, bool enable);
static bool usart1_txready(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart1_ops =
{
  .setup          = usart1_setup,
  .shutdown       = usart1_shutdown,
  .attach         = usart1_attach,
  .detach         = usart1_detach,
  .ioctl          = usart1_ioctl,
  .receive        = usart1_receive,
  .rxint          = usart1_rxint,
  .rxavailable    = usart1_rxavailable,
  .send           = usart1_send,
  .txint          = usart1_txint,
  .txready        = usart1_txready,
  .txempty        = usart1_txready,
};

/* I/O buffers */

static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];

/* This describes the state of the AT90USB USART1 port. */

#ifdef CONFIG_AVR_USART1_RS232
static uart_dev_t g_usart1port =
{
  .recv     =
  {
    .size   = CONFIG_USART1_RXBUFSIZE,
    .buffer = g_usart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART1_TXBUFSIZE,
    .buffer = g_usart1txbuffer,
   },
  .ops      = &g_uart1_ops,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usart1_restoreusartint
 ****************************************************************************/

static void usart1_restoreusartint(uint8_t imr)
{
# warning "Missing logic"
}

/****************************************************************************
 * Name: usart1_disableusartint
 ****************************************************************************/

static inline void usart1_disableusartint(uint8_t *imr)
{
# warning "Missing logic"
}

/****************************************************************************
 * Name: usart1_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int usart1_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG

  /* Configure the USART1 */

  usart1_configure();
#endif

  return OK;
}

/****************************************************************************
 * Name: usart1_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void usart1_shutdown(struct uart_dev_s *dev)
{
  /* Reset, disable interrupts, and disable Rx and Tx */

  usart1_reset();
}

/****************************************************************************
 * Name: usart1_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int usart1_attach(struct uart_dev_s *dev)
{
  /* Attach the USART1 IRQs */

  return irq_attach(AT90USB_IRQ_U1RX, usart1_interrupt);
  return irq_attach(AT90USB_IRQ_U1DRE, usart1_interrupt);
  return irq_attach(AT90USB_IRQ_U1TX, usart1_interrupt);
}

/****************************************************************************
 * Name: usart1_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void usart1_detach(struct uart_dev_s *dev)
{
  /* Disable USART1 interrupts */

# warning "Missing logic"

  /* Detach USART1 interrupts */

  return irq_deattach(AT90USB_IRQ_U1RX);
  return irq_deattach(AT90USB_IRQ_U1DRE);
  return irq_deattach(AT90USB_IRQ_U1TX);
}

/****************************************************************************
 * Name: usart1_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int usart1_interrupt(int irq, void *context)
{
  uint8_t           csr;
  int                passes;
  bool               handled;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

# warning "Missing logic"
      /* Handle incoming, receive bytes (with or without timeout) */

# warning "Missing logic"
        {
           /* Received data ready... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes */

# warning "Missing logic"
        {
           /* Transmit data regiser empty ... process outgoing bytes */

           uart_xmitchars(dev);
           handled = true;
        }
    }
    return OK;
}

/****************************************************************************
 * Name: usart1_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int usart1_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  int  ret = OK;

  switch (cmd)
    {
    case xxx: /* Add commands here */
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
#else
  return -ENOTTY;
#endif
}

/****************************************************************************
 * Name: usart1_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int usart1_receive(struct uart_dev_s *dev, uint32_t *status)
{
  /* Get the Rx byte.  The USART Rx interrupt flag is cleared by side effect
   * when reading the received character.
   */

# warning "Missing logic"

  /* Return status information */

  if (status)
    {
# warning "Missing logic"
    }

  /* Then return the actual received byte */

# warning "Missing logic"
  return 0;
}

/****************************************************************************
 * Name: usart1_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void usart1_rxint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
# warning "Missing logic"
#endif
    }
  else
    {
# warning "Missing logic"
    }
}

/****************************************************************************
 * Name: usart1_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool usart1_rxavailable(struct uart_dev_s *dev)
{
# warning "Missing logic"
  return 0;
}

/****************************************************************************
 * Name: usart1_send
 *
 * Description:
 *   This method will send one byte on the USART.
 *
 ****************************************************************************/

static void usart1_send(struct uart_dev_s *dev, int ch)
{
# warning "Missing logic"
  return 0;
}

/****************************************************************************
 * Name: usart1_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void usart1_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
# warning "Missing logic"

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

# warning "Missing logic"
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: usart1_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool usart1_txready(struct uart_dev_s *dev)
{
# warning "Missing logic"
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Disable all USARTS */

  up_disableusartint(NULL);

  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  usart1_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */

  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  uint8_t imr;

  up_disableusartint(priv, &imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  up_restoreusartint(priv, imr);
#endif
  return ch;
}

#else /* CONFIG_USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* CONFIG_USE_SERIALDRIVER */

