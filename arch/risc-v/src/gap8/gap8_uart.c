/****************************************************************************
 * arch/risc-v/src/gapuino/gap8_uart.c
 * UART driver on uDMA subsystem for GAP8
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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
 *  This UART IP has no flow control. So ioctl is limited.
 *  Note that here we don't use the uDMA to send multiple bytes, because
 *  NuttX serial drivers don't have abstraction for puts().
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
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include "gap8_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Software abstraction
 * inherit class _udma_peripheral
 */

struct gap8_uart_t
{
  struct gap8_udma_peripheral udma;

  /* Private */

  uint32_t tx_gpio;
  uint32_t rx_gpio;
  uint32_t coreclock;
  uint32_t baud;
  uint8_t  nr_bits;
  uint8_t  parity_enable;
  uint8_t  stop_bits;
  uint8_t  is_initialized;

  /* IO buffer for uDMA */

  uint8_t tx_buf[4];
  uint8_t rx_buf[4];
  int tx_cnt;
  int rx_cnt;
};

/****************************************************************************
 * Private Function prototype
 ****************************************************************************/

/* uart ISR routine */

static void uart_tx_isr(void *arg);
static void uart_rx_isr(void *arg);

/* Serial driver methods */

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uart_dev_t g_uart0port;

/* nuttx serial console operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* instantiate the UART */

static struct gap8_uart_t gap8_uarts[GAP8_NR_UART] =
{
  {
    .udma =
    {
      .regs  = (udma_reg_t *)UART,
      .id    = GAP8_UDMA_ID_UART,
      .on_tx = uart_tx_isr,
      .tx_arg = &g_uart0port,
      .on_rx = uart_rx_isr,
      .rx_arg = &g_uart0port,
      .is_tx_continous = 0,
      .is_rx_continous = 1,
    },

    .tx_gpio = GAP8_PIN_A7_UART_TX | GAP8_PIN_PULL_UP | GAP8_PIN_SPEED_HIGH,
    .rx_gpio = GAP8_PIN_B6_UART_RX | GAP8_PIN_PULL_UP | GAP8_PIN_SPEED_HIGH,
    .coreclock = CONFIG_CORE_CLOCK_FREQ, /* to be modified */
    .baud = CONFIG_UART_BAUD,
    .nr_bits = CONFIG_UART_BITS,
    .parity_enable = CONFIG_UART_PARITY,
    .stop_bits = CONFIG_UART_2STOP,
    .is_initialized = 0,
  }
};

/* IO buffers */

static char g_uart1rxbuffer[CONFIG_UART_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART_TXBUFSIZE];

/* Instantiate serial device */

static uart_dev_t g_uart0port =
{
  .isconsole = 1,
  .recv      =
  {
    .size    = CONFIG_UART_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &gap8_uarts[0],
};

/****************************************************************************
 * Private Function prototype
 ****************************************************************************/

/****************************************************************************
 * Name: uart_tx_isr & uart_rx_isr
 *
 * Description:
 *   These are the UART interrupt handler.  It is called on uDMA ISR. It
 *   should call uart_transmitchars or uart_receivechar to invoke the NuttX
 *   kernel.
 *
 ****************************************************************************/

static void uart_tx_isr(void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  the_uart->tx_cnt = 0;
  uart_xmitchars(dev);
}

static void uart_rx_isr(void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  the_uart->rx_cnt = 1;
  uart_recvchars(dev);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;
  uart_reg_t *uartreg = (uart_reg_t *)the_uart->udma.regs;
  uint32_t cfgreg = 0;

  if (the_uart->is_initialized == 0)
    {
      uint16_t div = the_uart->coreclock / the_uart->baud;

      gap8_udma_init(&the_uart->udma);

      /* Setup baudrate etc. */

      cfgreg = UART_SETUP_BIT_LENGTH(the_uart->nr_bits - 5) |
              UART_SETUP_PARITY_ENA(the_uart->parity_enable) |
              UART_SETUP_STOP_BITS(the_uart->stop_bits) |
              UART_SETUP_TX_ENA(1) |
              UART_SETUP_RX_ENA(1) |
              UART_SETUP_CLKDIV(div);
      uartreg->SETUP = cfgreg;

      gap8_configpin(the_uart->tx_gpio);
      gap8_configpin(the_uart->rx_gpio);

      the_uart->tx_cnt = 0;
      the_uart->rx_cnt = 0;

#if 0
      /* Start continuous rx */

      gap8_udma_rx_start(&the_uart->udma, the_uart->rx_buf, 1, 1);
#endif
    }

  the_uart->is_initialized = 1;
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  gap8_udma_deinit(&the_uart->udma);
  the_uart->is_initialized = 0;
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are
 *   called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  if (the_uart->is_initialized == 0)
    {
      up_setup(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  gap8_udma_tx_setirq(&the_uart->udma, 0);
  gap8_udma_rx_setirq(&the_uart->udma, 0);
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode      *inode;
  struct uart_dev_s *dev;
  struct gap8_uart_t   *priv;
  int                ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv);
  priv = (struct gap8_uart_t *)dev->priv;

  switch (cmd)
    {
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Other termios fields are not yet returned.
         * Note that only cfsetospeed is not necessary because we have
         * knowledge that only one speed is supported.
         */

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Handle other termios settings.
         * Note that only cfgetispeed is used besued we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);

#if 0
        gap8_uartconfigure(priv->uartbase, priv->baud, priv->parity,
                           priv->bits, priv->stopbits2);
#endif
      }
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
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;
  uint8_t ch = the_uart->rx_buf[0];

  the_uart->rx_cnt = 0;

  if (status)
    {
      *status = 0; /* We are not yet tracking serial errors */
    }

  /* Then trigger another reception */

  gap8_udma_rx_start(&the_uart->udma, the_uart->rx_buf, 1, 1);

  return (int)ch;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      gap8_udma_rx_setirq(&the_uart->udma, 1);

#if 0
      if (the_uart->rx_cnt == 0)
#endif
        {
          gap8_udma_rx_start(&the_uart->udma, the_uart->rx_buf, 1, 1);
        }
    }
#if 0
  else
    {
      gap8_udma_rx_setirq(&the_uart->udma, 0);
    }
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  return the_uart->rx_cnt > 0;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  the_uart->tx_buf[0] = (uint8_t)ch;
  the_uart->tx_buf[1] = 0;
  the_uart->tx_cnt = 1;
  gap8_udma_tx_start(&the_uart->udma, the_uart->tx_buf, 1, 1);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      gap8_udma_tx_setirq(&the_uart->udma, 1);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
    }
  else
    {
      gap8_udma_tx_setirq(&the_uart->udma, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  return (the_uart->tx_cnt == 0) ? true : false;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct gap8_uart_t *the_uart = (struct gap8_uart_t *)dev->priv;

  return (the_uart->tx_cnt == 0) ? true : false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock iniialization
 *   performed in up_clkinitialize().
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef CONFIG_UART_SERIAL_CONSOLE
  g_uart0port.isconsole = true;
  up_setup(&g_uart0port);
#endif
}
#endif

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
#ifdef CONFIG_UART_SERIAL_CONSOLE
  uart_register("/dev/console", &g_uart0port);
#endif

  uart_register("/dev/ttyS0", &g_uart0port);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONFIG_UART_SERIAL_CONSOLE
  struct uart_dev_s *dev = (struct uart_dev_s *)&g_uart0port;

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_putc('\r');
    }

  up_send(dev, ch);
#endif
  return ch;
}
