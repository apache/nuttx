/****************************************************************************
 * arch/avr/src/atmega/atmega_serial.c
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
#include "atmega_config.h"

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
#include <avr/io.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"
#include "atmega_internal.h"

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

/* Which USART with be tty0/console and which tty1? */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart0port     /* USART0 is console */
#  define TTYS0_DEV       g_usart0port     /* USART0 is ttyS0 */
#  ifdef CONFIG_AVR_USART1
#    define TTYS1_DEV     g_usart1port     /* USART1 is ttyS1 */
#  else
#    undef TTYS1_DEV                       /* No ttyS1 */
#  endif
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart1port     /* USART1 is console */
#  define TTYS0_DEV       g_usart1port     /* USART1 is ttyS0 */
#  ifdef CONFIG_AVR_USART0
#    define TTYS1_DEV     g_usart0port     /* USART0 is ttyS1 */
#  else
#    undef TTYS1_DEV                       /* No ttyS1 */
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int  usart0_setup(struct uart_dev_s *dev);
static void usart0_shutdown(struct uart_dev_s *dev);
static int  usart0_attach(struct uart_dev_s *dev);
static void usart0_detach(struct uart_dev_s *dev);
static int  usart0_interrupt(int irq, void *context);
static int  usart0_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  usart0_receive(struct uart_dev_s *dev, uint32_t *status);
static void usart0_rxint(struct uart_dev_s *dev, bool enable);
static bool usart0_rxavailable(struct uart_dev_s *dev);
static void usart0_send(struct uart_dev_s *dev, int ch);
static void usart0_txint(struct uart_dev_s *dev, bool enable);
static bool usart0_txready(struct uart_dev_s *dev);
#endif

#ifdef CONFIG_AVR_USART1
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
#endif

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* USART0 operations */

#ifdef CONFIG_AVR_USART0
struct uart_ops_s g_usart0_ops =
{
  .setup          = usart0_setup,
  .shutdown       = usart0_shutdown,
  .attach         = usart0_attach,
  .detach         = usart0_detach,
  .ioctl          = usart0_ioctl,
  .receive        = usart0_receive,
  .rxint          = usart0_rxint,
  .rxavailable    = usart0_rxavailable,
  .send           = usart0_send,
  .txint          = usart0_txint,
  .txready        = usart0_txready,
  .txempty        = usart0_txready,
};

/* USART0 I/O buffers */

static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];

/* This describes the state of the ATMega USART0 port. */

static uart_dev_t g_usart0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_usart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_usart0txbuffer,
  },
  .ops      = &g_usart0_ops,
};
#endif

/* USART1 operations */

#ifdef CONFIG_AVR_USART1
struct uart_ops_s g_usart1_ops =
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

/* USART 1 I/O buffers */

static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];

/* This describes the state of the ATMega USART1 port. */

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
  .ops      = &g_usart1_ops,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usart0/1_restoreusartint
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_restoreusartint(uint8_t imr)
{
# warning "Missing logic"
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_restoreusartint(uint8_t imr)
{
# warning "Missing logic"
}
#endif

/****************************************************************************
 * Name: usart0/1_disableusartint
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static inline void usart0_disableusartint(uint8_t *imr)
{
# warning "Missing logic"
}
#endif

#ifdef CONFIG_AVR_USART1
static inline void usart1_disableusartint(uint8_t *imr)
{
# warning "Missing logic"
}
#endif

/****************************************************************************
 * Name: usart0/1_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the USART as an RS-232 UART */

  usart0_configure();
#endif

  return OK;
}
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the USART as an RS-232 UART */

  usart1_configure();
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: usart0/1_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_shutdown(struct uart_dev_s *dev)
{
  /* Reset, disable interrupts, and disable Rx and Tx */

  usart0_reset();
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_shutdown(struct uart_dev_s *dev)
{
  /* Reset, disable interrupts, and disable Rx and Tx */

  usart1_reset();
}
#endif

/****************************************************************************
 * Name: usart0/1_attach
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

#ifdef CONFIG_AVR_USART0
static int usart0_attach(struct uart_dev_s *dev)
{
  /* Attach the USART0 IRQs */

  return irq_attach(ATMEGA_IRQ_U0RX, usart0_interrupt);
  return irq_attach(ATMEGA_IRQ_U0DRE, usart0_interrupt);
  return irq_attach(ATMEGA_IRQ_U0TX, usart0_interrupt);
}
#endif

#ifdef CONFIG_AVR_USART0
static int usart0_attach(struct uart_dev_s *dev)
{
  /* Attach the USART0 IRQs */

  return irq_attach(ATMEGA_IRQ_U1RX, usart1_interrupt);
  return irq_attach(ATMEGA_IRQ_U1DRE, usart1_interrupt);
  return irq_attach(ATMEGA_IRQ_U1TX, usart1_interrupt);
}
#endif

/****************************************************************************
 * Name: usart0/1_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_detach(struct uart_dev_s *dev)
{
  /* Disable all USART0 interrupts */
# warning "Missing logic"

  /* Detach the USART0 IRQs */

  return irq_detach(ATMEGA_IRQ_U0RX);
  return irq_detach(ATMEGA_IRQ_U0DRE);
  return irq_detach(ATMEGA_IRQ_U0TX);
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_detach(struct uart_dev_s *dev)
{
  /* Disable all USART0 interrupts */
# warning "Missing logic"

  /* Detach the USART0 IRQs */

  return irq_deattach(ATMEGA_IRQ_U1RX);
  return irq_deattach(ATMEGA_IRQ_U1DRE);
  return irq_deattach(ATMEGA_IRQ_U1TX);
}
#endif

/****************************************************************************
 * Name: usart0/1_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_interrupt(int irq, void *context)
{
  uint8_t csr;
  int     passes;
  bool    handled;

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
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_interrupt(int irq, void *context)
{
  uint8_t csr;
  int     passes;
  bool    handled;

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
#endif

/****************************************************************************
 * Name: usart0/1_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  int ret = OK;

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
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  int ret = OK;

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
#endif

/****************************************************************************
 * Name: usart0/1_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_receive(struct uart_dev_s *dev, uint32_t *status)
{
  /* Return status information */

  if (status)
    {
	  *status = (uint32_t)UCSR0A;
    }

  /* Then return the actual received byte */

  return UDR0;
}
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_receive(struct uart_dev_s *dev, uint32_t *status)
{
  /* Return status information */

  if (status)
    {
	  *status = (uint32_t)UCSR1A;
    }

  /* Then return the actual received byte */

  return UDR1;
}
#endif

/****************************************************************************
 * Name: usart0/1_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_rxint(struct uart_dev_s *dev, bool enable)
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
#endif

#ifdef CONFIG_AVR_USART1
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
#endif

/****************************************************************************
 * Name: usart0/1_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static bool usart0_rxavailable(struct uart_dev_s *dev)
{
  return (UCSR0A & (1 << RXC0)) != 0;
}
#endif

#ifdef CONFIG_AVR_USART1
static bool usart1_rxavailable(struct uart_dev_s *dev)
{
  return (UCSR1A & (1 << RXC1)) != 0;
}
#endif

/****************************************************************************
 * Name: usart0/1_send
 *
 * Description:
 *   This method will send one byte on the USART.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_send(struct uart_dev_s *dev, int ch)
{
  UDR0 = ch;
}
#endif

#ifdef CONFIG_AVR_USART0
static void usart0_send(struct uart_dev_s *dev, int ch)
{
  UDR1 = ch;
}
#endif

/****************************************************************************
 * Name: usart0/1_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_txint(struct uart_dev_s *dev, bool enable)
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
#endif

#ifdef CONFIG_AVR_USART1
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
#endif

/****************************************************************************
 * Name: usart0/1_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static bool usart0_txready(struct uart_dev_s *dev)
{
  return (UCSR0A & (1 << UDRE0)) != 0;
}
#endif

#ifdef CONFIG_AVR_USART1
static bool usart1_txready(struct uart_dev_s *dev)
{
  return (UCSR1A & (1 << UDRE1)) != 0;
}
#endif

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

  usart0_disableusartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  usart0_disableusartint(TTYS1_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
  usart0_setup(&CONSOLE_DEV);
#  elif defined(CONFIG_USART1_SERIAL_CONSOLE)
  usart1_setup(&CONSOLE_DEV);
#  endif
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

  up_disableusartint(&imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  up_restoreusartint(imr);
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

