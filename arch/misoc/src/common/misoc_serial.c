/****************************************************************************
 * arch/misoc/src/lm32/lm32_blocktask.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include <arch/board/generated/csr.h>

#include "chip.h"
#include "hw/flags.h"
#include "misoc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_UART_DEVICE
#if defined(CONFIG_MISOC_UART1) || defined(CONFIG_MISOC_UART2)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,.. CHIP_NUARTS
 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_MISOC_UART1)
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_MISOC_UART2)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of misoc_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port     /* UART1 is console */
#    define TTYS0_DEV       g_uart1port     /* UART1 is ttyS0 */
#    undef  TTYS1_DEV                       /* No ttyS1 */
#    define SERIAL_CONSOLE  1
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  if defined(CONFIG_NR5_UART1)
#    define TTYS0_DEV       g_uart1port     /* UART1 is ttyS0 */
#    undef  TTYS1_DEV                       /* No ttyS1 */
#    define SERIAL_CONSOLE  1
#  else
#    undef  TTYS0_DEV
#    undef  TTYS1_DEV
#  endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of misoc_earlyserialinit(), misoc_serial_initialize(), and
 * misoc_putc().
 */

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct misoc_dev_s
{
  uintptr_t uartbase;
  uintptr_t rxtx_addr;
  uintptr_t txfull_addr;
  uintptr_t rxempty_addr;
  uintptr_t ev_status_addr;
  uintptr_t ev_pending_addr;
  uintptr_t ev_enable_addr;
  uint8_t irq;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers */

static void misoc_restoreuartint(struct uart_dev_s *dev, uint8_t im);
static void misoc_disableuartint(struct uart_dev_s *dev, uint8_t *im);

/* Serial driver methods */

static int  misoc_setup(struct uart_dev_s *dev);
static void misoc_shutdown(struct uart_dev_s *dev);
static int  misoc_attach(struct uart_dev_s *dev);
static void misoc_detach(struct uart_dev_s *dev);
static int  misoc_uart_interrupt(int irq, void *context, FAR void *arg);
static int  misoc_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  misoc_receive(struct uart_dev_s *dev, uint32_t *status);
static void misoc_rxint(struct uart_dev_s *dev, bool enable);
static bool misoc_rxavailable(struct uart_dev_s *dev);
static void misoc_send(struct uart_dev_s *dev, int ch);
static void misoc_txint(struct uart_dev_s *dev, bool enable);
static bool misoc_txready(struct uart_dev_s *dev);
static bool misoc_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = misoc_setup,
  .shutdown       = misoc_shutdown,
  .attach         = misoc_attach,
  .detach         = misoc_detach,
  .ioctl          = misoc_ioctl,
  .receive        = misoc_receive,
  .rxint          = misoc_rxint,
  .rxavailable    = misoc_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = misoc_send,
  .txint          = misoc_txint,
  .txready        = misoc_txready,
  .txempty        = misoc_txempty,
};

/* I/O buffers */

#ifdef CONFIG_MISOC_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the NR5 UART1 port. */

#ifdef CONFIG_MISOC_UART1
#ifndef CONFIG_MISOC_UART1PRIO
# define CONFIG_MISOC_UART1PRIO 4
#endif

static struct misoc_dev_s g_uart1priv =
{
  .uartbase        = CSR_UART_BASE,
  .irq             = UART_INTERRUPT,
  .rxtx_addr       = CSR_UART_RXTX_ADDR,
  .rxempty_addr    = CSR_UART_RXEMPTY_ADDR,
  .txfull_addr     = CSR_UART_TXFULL_ADDR,
  .ev_status_addr  = CSR_UART_EV_STATUS_ADDR,
  .ev_pending_addr = CSR_UART_EV_PENDING_ADDR,
  .ev_enable_addr  = CSR_UART_EV_ENABLE_ADDR,
};

static uart_dev_t g_uart1port =
{
#if SERIAL_CONSOLE == 1
  .isconsole = 1,
#endif
  .recv      =
  {
    .size    = CONFIG_UART1_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART1_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
   },
  .ops       = &g_uart_ops,
  .priv      = &g_uart1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: misoc_restoreuartint
 ****************************************************************************/

static void misoc_restoreuartint(struct uart_dev_s *dev, uint8_t im)
{
  /* Re-enable/re-disable interrupts corresponding to the state of bits in
   * im.
   */

  uart_ev_enable_write(im);
}

/****************************************************************************
 * Name: misoc_disableuartint
 ****************************************************************************/

static void misoc_disableuartint(struct uart_dev_s *dev, uint8_t *im)
{
  if (im)
   {
     *im = uart_ev_enable_read();
   }

  misoc_restoreuartint(dev, 0);
}

/****************************************************************************
 * Name: misoc_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int misoc_setup(struct uart_dev_s *dev)
{
  uart_ev_pending_write(uart_ev_pending_read());
  return OK;
}

/****************************************************************************
 * Name: misoc_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void misoc_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: misoc_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int misoc_attach(struct uart_dev_s *dev)
{
  struct misoc_dev_s *priv = (struct misoc_dev_s *)dev->priv;

  (void)irq_attach(priv->irq, misoc_uart_interrupt, dev);
  up_enable_irq(priv->irq);

  return OK;
}

/****************************************************************************
 * Name: misoc_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void misoc_detach(struct uart_dev_s *dev)
{
  struct misoc_dev_s *priv = (struct misoc_dev_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: misoc_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int misoc_uart_interrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint32_t stat;

  DEBUGASSERT(dev != NULL);

  /* Read as much as we can */

  stat = uart_ev_pending_read();
  if (stat & UART_EV_RX)
    {
      while (!uart_rxempty_read())
        {
          uart_recvchars(dev);
        }
    }

  /* Try to send all the buffer that were not sent.  Does uart_xmitchars
   * send only if there is something to send ???
   */

  if ((stat & UART_EV_TX) != 0)
    {
      uart_ev_pending_write(UART_EV_TX);
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: misoc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int misoc_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  return -ENOSYS;
#else
  return -ENOTTY;
#endif
}

/****************************************************************************
 * Name: misoc_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int misoc_receive(struct uart_dev_s *dev, uint32_t *status)
{
  int ret;

  if (status != NULL)
    {
      *status = 0;
    }

  ret = uart_rxtx_read();

  uart_ev_pending_write(UART_EV_RX);

  return ret;
}

/****************************************************************************
 * Name: misoc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void misoc_rxint(struct uart_dev_s *dev, bool enable)
{
  uint8_t im;

  im = uart_ev_enable_read();
  if (enable)
    {
      im |= UART_EV_RX;
    }
  else
    {
      im &= ~UART_EV_RX;
    }

  uart_ev_enable_write(im);
}

/****************************************************************************
 * Name: misoc_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool misoc_rxavailable(struct uart_dev_s *dev)
{
  return !uart_rxempty_read();
}

/****************************************************************************
 * Name: misoc_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void misoc_send(struct uart_dev_s *dev, int ch)
{
  uart_rxtx_write(ch);
}

/****************************************************************************
 * Name: misoc_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void misoc_txint(struct uart_dev_s *dev, bool enable)
{
  uint8_t im;

  im = uart_ev_enable_read();
  if (enable)
    {
      im |= UART_EV_TX;
      uart_ev_enable_write(im);

      /* Fake an uart INT */

      uart_xmitchars(dev);
    }
  else
    {
      im  &= ~UART_EV_TX;
      uart_ev_enable_write(im);
    }
}

/****************************************************************************
 * Name: misoc_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool misoc_txready(struct uart_dev_s *dev)
{
  return !uart_txfull_read();
}

/****************************************************************************
 * Name: misoc_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool misoc_txempty(struct uart_dev_s *dev)
{
  return !uart_txfull_read();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: misoc_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before misoc_serial_initialize.
 *
 ****************************************************************************/

void misoc_earlyserialinit(void)
{
}

/****************************************************************************
 * Name: misoc_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  struct uart_dev_s *dev = (struct uart_dev_s *)&CONSOLE_DEV;
  uint8_t imr;

   misoc_disableuartint(dev, &imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      misoc_lowputc('\r');
    }

  misoc_lowputc(ch);
  misoc_restoreuartint(dev, imr);
#endif
  return ch;
}

/****************************************************************************
 * Name: misoc_earlyserialinit, misoc_serial_initialize, and misoc_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/misoc_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

#else /* HAVE_UART_DEVICE */
void misoc_earlyserialinit(void)
{
}

void misoc_serial_initialize(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: misoc_putc
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

      misoc_lowputc('\r');
    }

  misoc_lowputc(ch);
#endif

  return ch;
}

#endif /* USE_SERIALDRIVER */

void misoc_lowputc(char ch)
{
  while (uart_txfull_read());
  uart_rxtx_write(ch);
  uart_ev_pending_write(UART_EV_TX);
}

/****************************************************************************
 * Name: misoc_serial_initialize
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that misoc_earlyserialinit was called previously.
 *
 ****************************************************************************/

void misoc_serial_initialize(void)
{
#ifdef USE_SERIALDRIVER
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
}
