/****************************************************************************
 * arch/arm/src/nrf52/nrf52_serial.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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

#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "nrf52_config.h"
#include "chip/nrf52_uarte.h"
#include "nrf52_clockconfig.h"
#include "nrf52_lowputc.h"
#include "nrf52_serial.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/
/* Is there at least one UART enabled and configured as a RS-232 device? */

#ifndef HAVE_UART_DEVICE
#  warning "No UARTs enabled"
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(HAVE_UART_DEVICE) && defined(USE_SERIALDRIVER)

/* Which UART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one UART device */

struct nrf52_dev_s
{
  uintptr_t uartbase;     /* Base address of UART registers */
  uint8_t   irq;          /* IRQ associated with this UART */
  bool      rx_available; /* rx byte available */

  /* UART configuration */

  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  nrf52_setup(struct uart_dev_s *dev);
static void nrf52_shutdown(struct uart_dev_s *dev);
static int  nrf52_attach(struct uart_dev_s *dev);
static void nrf52_detach(struct uart_dev_s *dev);
static int  nrf52_interrupt(int irq, void *context, FAR void *arg);
static int  nrf52_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  nrf52_receive(struct uart_dev_s *dev, uint32_t *status);
static void nrf52_rxint(struct uart_dev_s *dev, bool enable);
static bool nrf52_rxavailable(struct uart_dev_s *dev);
static void nrf52_send(struct uart_dev_s *dev, int ch);
static void nrf52_txint(struct uart_dev_s *dev, bool enable);
static bool nrf52_txready(struct uart_dev_s *dev);
static bool nrf52_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = nrf52_setup,
  .shutdown       = nrf52_shutdown,
  .attach         = nrf52_attach,
  .detach         = nrf52_detach,
  .ioctl          = nrf52_ioctl,
  .receive        = nrf52_receive,
  .rxint          = nrf52_rxint,
  .rxavailable    = nrf52_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = nrf52_send,
  .txint          = nrf52_txint,
  .txready        = nrf52_txready,
  .txempty        = nrf52_txempty,
};

/* I/O buffers */

#ifdef HAVE_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

/* This describes the state of the NRF52 UART0 port. */

#ifdef HAVE_UART0
static struct nrf52_dev_s g_uart0priv =
{
  .uartbase       = NRF52_UART0_BASE,
  .irq            = NRF52_IRQ_UART0,
  .rx_available   = false,
  .config         =
  {
    .baud         = CONFIG_UART0_BAUD,
    .parity       = CONFIG_UART0_PARITY,
    .bits         = CONFIG_UART0_BITS,
    .stopbits2    = CONFIG_UART0_2STOP,
#ifdef CONFIG_UART0_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_UART0_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

#define CONSOLE_DEV         g_uart0port /* UART0 is console */
#define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int nrf52_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv; */

  /* Configure the UART as an RS-232 UART */

  /* REVISIT: If nrf52_usart_configure() called 2nd time uart stops working.
   * Rx interrupt keeps firing.
   * configuring is done on __start
   */

  /* nrf52_usart_configure(priv->uartbase, &priv->config); */

#endif

  return OK;
}

/****************************************************************************
 * Name: nrf52_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void nrf52_shutdown(struct uart_dev_s *dev)
{
  struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv;

  /* Disable interrupts */
  /* Reset hardware and disable Rx and Tx */

  nrf52_usart_disable(priv->uartbase);
}

/****************************************************************************
 * Name: nrf52_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int nrf52_attach(struct uart_dev_s *dev)
{
  struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(priv->irq, nrf52_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void nrf52_detach(struct uart_dev_s *dev)
{
  struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv;

  /* Disable interrupts */

  putreg32(NRF52_UART_INTENSET_RXDRDY, priv->uartbase + NRF52_UART_INTENCLR_OFFSET);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: nrf52_interrupt
 *
 * Description:
 *   This is the UART status interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int nrf52_interrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct nrf52_dev_s *priv;
  uint32_t regval;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct nrf52_dev_s *)dev->priv;

  /* Check RX event */

  regval = getreg32(priv->uartbase + NRF52_UART_EVENTS_RXDRDY_OFFSET);

  if (regval != 0)
    {
      putreg32(0, priv->uartbase + NRF52_UART_EVENTS_RXDRDY_OFFSET);
      priv->rx_available = true;
      uart_recvchars(dev);
    }

  /* Clear errors */

  putreg32(0, priv->uartbase + NRF52_UART_ERRORSRC_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: nrf52_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int nrf52_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: nrf52_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int nrf52_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv;
  uint32_t data;

  /* Get input data along with receiver control information */

  data = getreg32(priv->uartbase + NRF52_UART_RXD_OFFSET);
  priv->rx_available = false;

  /* Return receiver control information */

  if (status)
    {
      *status = 0x00;
    }

  /* Then return the actual received data. */

  return data;
}

/****************************************************************************
 * Name: nrf52_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void nrf52_rxint(struct uart_dev_s *dev, bool enable)
{
  struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

      putreg32(NRF52_UART_INTENSET_RXDRDY,
               priv->uartbase + NRF52_UART_INTENSET_OFFSET);
      putreg32(1, priv->uartbase + NRF52_UART_TASKS_STARTRX_OFFSET);

#endif
    }
  else
    {
      putreg32(NRF52_UART_INTENSET_RXDRDY,
               priv->uartbase + NRF52_UART_INTENCLR_OFFSET);
      putreg32(1, priv->uartbase + NRF52_UART_TASKS_STOPRX_OFFSET);
    }
}

/****************************************************************************
 * Name: nrf52_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool nrf52_rxavailable(struct uart_dev_s *dev)
{
  struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv;

  /* Return true if the receive buffer/fifo is not "empty." */

  return priv->rx_available;
}

/****************************************************************************
 * Name: nrf52_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void nrf52_send(struct uart_dev_s *dev, int ch)
{
  struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv;

  putreg32(0, priv->uartbase + NRF52_UART_EVENTS_TXDRDY_OFFSET);
  putreg32(1, priv->uartbase + NRF52_UART_TASKS_STARTTX_OFFSET);

  putreg32(ch, priv->uartbase + NRF52_UART_TXD_OFFSET);
  while (getreg32(priv->uartbase + NRF52_UART_EVENTS_TXDRDY_OFFSET) == 0 )
    {
    }

  putreg32(1, priv->uartbase + NRF52_UART_TASKS_STOPTX_OFFSET);

}

/****************************************************************************
 * Name: nrf52_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void nrf52_txint(struct uart_dev_s *dev, bool enable)
{
  /* struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv; */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      irqstate_t flags;

      /* Enable the TX interrupt */

      flags = enter_critical_section();

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
      leave_critical_section(flags);
#endif
    }
  else
    {
      /* Disable the TX interrupt */
    }
}

/****************************************************************************
 * Name: nrf52_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool nrf52_txready(struct uart_dev_s *dev)
{
  /* struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv; */

  /* Return true if the transmit FIFO is "not full." */

  return true;
}

/****************************************************************************
 * Name: nrf52_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool nrf52_txempty(struct uart_dev_s *dev)
{
  /* struct nrf52_dev_s *priv = (struct nrf52_dev_s *)dev->priv; */

  /* Return true if the transmit FIFO is "empty." */

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before nrf52_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in nrf52_lowsetup() and main clock iniialization
 *   performed in nrf_clock_configure().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void nrf52_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  nrf52_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that nrf52_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   None
 *
 * Returns Value:
 *   None
 *
 ****************************************************************************/

void up_serialinit(void)
{
#ifdef HAVE_UART_CONSOLE
  /* Register the serial console */

  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
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
#ifdef HAVE_UART_CONSOLE
  /* struct nrf52_dev_s *priv = (struct nrf52_dev_s *)CONSOLE_DEV.priv; */

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

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_UART_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  return ch;
#endif
}

#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER */
