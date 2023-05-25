/****************************************************************************
 * arch/arm/src/nrf53/nrf53_serial.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "nrf53_config.h"
#include "hardware/nrf53_uarte.h"
#include "nrf53_clockconfig.h"
#include "nrf53_lowputc.h"
#include "nrf53_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many UARTs are supported */

#ifdef HAVE_UART1
#  define NRF53_NUART 2
#else
#  define NRF53_NUART 1
#endif

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

#ifdef CONFIG_UART0_SERIAL_CONSOLE
#  define CONSOLE_DEV         g_uart0port /* UART0 is console */
#  define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#elif CONFIG_UART1_SERIAL_CONSOLE
#  define CONSOLE_DEV         g_uart1port /* UART1 is console */
#  define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one UART device */

struct nrf53_dev_s
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

static int  nrf53_setup(struct uart_dev_s *dev);
static void nrf53_shutdown(struct uart_dev_s *dev);
static int  nrf53_attach(struct uart_dev_s *dev);
static void nrf53_detach(struct uart_dev_s *dev);
static int  nrf53_interrupt(int irq, void *context, void *arg);
static int  nrf53_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  nrf53_receive(struct uart_dev_s *dev, unsigned int *status);
static void nrf53_rxint(struct uart_dev_s *dev, bool enable);
static bool nrf53_rxavailable(struct uart_dev_s *dev);
static void nrf53_send(struct uart_dev_s *dev, int ch);
static void nrf53_txint(struct uart_dev_s *dev, bool enable);
static bool nrf53_txready(struct uart_dev_s *dev);
static bool nrf53_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = nrf53_setup,
  .shutdown       = nrf53_shutdown,
  .attach         = nrf53_attach,
  .detach         = nrf53_detach,
  .ioctl          = nrf53_ioctl,
  .receive        = nrf53_receive,
  .rxint          = nrf53_rxint,
  .rxavailable    = nrf53_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = nrf53_send,
  .txint          = nrf53_txint,
  .txready        = nrf53_txready,
  .txempty        = nrf53_txempty,
};

/* I/O buffers */

#ifdef HAVE_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef HAVE_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the NRF53 UART0 port. */

#ifdef HAVE_UART0
static struct nrf53_dev_s g_uart0priv =
{
  .uartbase       = NRF53_UART0_BASE,
  .irq            = NRF53_IRQ_SERIAL0,
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
    .txpin        = BOARD_UART0_TX_PIN,
    .rxpin        = BOARD_UART0_RX_PIN,
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

/* This describes the state of the NRF53 UART1 port. */

#ifdef HAVE_UART1
static struct nrf53_dev_s g_uart1priv =
{
  .uartbase       = NRF53_UART1_BASE,
  .irq            = NRF53_IRQ_SERIAL1,
  .rx_available   = false,
  .config         =
  {
    .baud         = CONFIG_UART1_BAUD,
    .parity       = CONFIG_UART1_PARITY,
    .bits         = CONFIG_UART1_BITS,
    .stopbits2    = CONFIG_UART1_2STOP,
#ifdef CONFIG_UART1_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_UART1_OFLOWCONTROL
    .oflow        = true,
#endif
    .txpin        = BOARD_UART1_TX_PIN,
    .rxpin        = BOARD_UART1_RX_PIN,
  }
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif

/* This table lets us iterate over the configured UARTs */

static struct uart_dev_s * const g_uart_devs[NRF53_NUART] =
{
#ifdef HAVE_UART0
  [0] = &g_uart0port,
#endif
#ifdef HAVE_UART1
  [1] = &g_uart1port
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int nrf53_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;

  /* TODO: Configure the UART as an RS-232 UART */

  /* REVISIT: If nrf53_usart_configure() called 2nd time uart stops working.
   * Rx interrupt keeps firing.
   * configuring is done on __start
   *
   * UPDATE 19.12.2019: No problems described above were observed,
   * but just in case we leave the above note for some time.
   */

  nrf53_usart_configure(priv->uartbase, &priv->config);
#endif

  /* TODO: configure UART if not selected as console */

  return OK;
}

/****************************************************************************
 * Name: nrf53_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void nrf53_shutdown(struct uart_dev_s *dev)
{
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;

  /* Disable interrupts */

  /* Reset hardware and disable Rx and Tx */

  nrf53_usart_disable(priv->uartbase, &priv->config);
}

/****************************************************************************
 * Name: nrf53_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).
 *   The RX and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int nrf53_attach(struct uart_dev_s *dev)
{
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(priv->irq, nrf53_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf53_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void nrf53_detach(struct uart_dev_s *dev)
{
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;

  /* Disable interrupts */

  putreg32(UART_INT_RXDRDY, priv->uartbase + NRF53_UART_INTENCLR_OFFSET);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: nrf53_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int nrf53_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct nrf53_dev_s *priv;
  uint32_t regval;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct nrf53_dev_s *)dev->priv;

  /* Check RX event */

  regval = getreg32(priv->uartbase + NRF53_UART_EVENTS_RXDRDY_OFFSET);

  if (regval != 0)
    {
      putreg32(0, priv->uartbase + NRF53_UART_EVENTS_RXDRDY_OFFSET);
      priv->rx_available = true;
      uart_recvchars(dev);
    }

  /* Clear errors */

  putreg32(0, priv->uartbase + NRF53_UART_ERRORSRC_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: nrf53_set_format
 *
 * Description:
 *   Set the serial line format and speed.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TERMIOS
void nrf53_set_format(struct uart_dev_s *dev)
{
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;

  nrf53_usart_setformat(priv->uartbase, &priv->config);
}
#endif

/****************************************************************************
 * Name: nrf53_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int nrf53_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode         *inode  = filep->f_inode;
  struct uart_dev_s    *dev    = inode->i_private;
  struct nrf53_dev_s   *priv   = (struct nrf53_dev_s *)dev->priv;
  struct uart_config_s *config = &priv->config;
#endif
  int                   ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          termiosp->c_cflag = ((config->parity != 0) ? PARENB : 0)
                              | ((config->parity == 1) ? PARODD : 0)
                              | ((config->stopbits2) ? CSTOPB : 0) |
#ifdef CONFIG_SERIAL_OFLOWCONTROL
                              ((config->oflow) ? CCTS_OFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
                              ((config->iflow) ? CRTS_IFLOW : 0) |
#endif
                              CS8;

          cfsetispeed(termiosp, config->baud);

          break;
        }

      case TCSETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Perform some sanity checks before accepting any changes */

          if ((termiosp->c_cflag & CSIZE) != CS8)
            {
              ret = -EINVAL;
              break;
            }

#ifndef HAVE_UART_STOPBITS
          if ((termiosp->c_cflag & CSTOPB) != 0)
            {
              ret = -EINVAL;
              break;
            }
#endif

          if (termiosp->c_cflag & PARODD)
            {
              ret = -EINVAL;
              break;
            }

          /* TODO: CCTS_OFLOW and CRTS_IFLOW */

          /* Parity */

          if (termiosp->c_cflag & PARENB)
            {
              config->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              config->parity = 0;
            }

#ifdef HAVE_UART_STOPBITS
          /* Stop bits */

          config->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
#endif

          /* Note that only cfgetispeed is used because we have knowledge
           * that only one speed is supported.
           */

          config->baud = cfgetispeed(termiosp);

          /* Effect the changes */

          nrf53_set_format(dev);

          break;
        }
#endif

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrf53_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int nrf53_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;
  uint32_t data;

  /* Get input data along with receiver control information */

  data = getreg32(priv->uartbase + NRF53_UART_RXD_OFFSET);
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
 * Name: nrf53_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void nrf53_rxint(struct uart_dev_s *dev, bool enable)
{
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

      putreg32(UART_INT_RXDRDY, priv->uartbase + NRF53_UART_INTENSET_OFFSET);
      putreg32(1, priv->uartbase + NRF53_UART_TASKS_STARTRX_OFFSET);

#endif
    }
  else
    {
      putreg32(UART_INT_RXDRDY, priv->uartbase + NRF53_UART_INTENCLR_OFFSET);
      putreg32(1, priv->uartbase + NRF53_UART_TASKS_STOPRX_OFFSET);
    }
}

/****************************************************************************
 * Name: nrf53_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool nrf53_rxavailable(struct uart_dev_s *dev)
{
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;

  /* Return true if the receive buffer/fifo is not "empty." */

  return priv->rx_available;
}

/****************************************************************************
 * Name: nrf53_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void nrf53_send(struct uart_dev_s *dev, int ch)
{
  struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv;

  putreg32(0, priv->uartbase + NRF53_UART_EVENTS_TXDRDY_OFFSET);
  putreg32(1, priv->uartbase + NRF53_UART_TASKS_STARTTX_OFFSET);

  putreg32(ch, priv->uartbase + NRF53_UART_TXD_OFFSET);
  while (getreg32(priv->uartbase + NRF53_UART_EVENTS_TXDRDY_OFFSET) == 0)
    {
    }

  putreg32(1, priv->uartbase + NRF53_UART_TASKS_STOPTX_OFFSET);
}

/****************************************************************************
 * Name: nrf53_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void nrf53_txint(struct uart_dev_s *dev, bool enable)
{
  /* struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv; */

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
 * Name: nrf53_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool nrf53_txready(struct uart_dev_s *dev)
{
  /* struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv; */

  /* Return true if the transmit FIFO is "not full." */

  return true;
}

/****************************************************************************
 * Name: nrf53_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool nrf53_txempty(struct uart_dev_s *dev)
{
  /* struct nrf53_dev_s *priv = (struct nrf53_dev_s *)dev->priv; */

  /* Return true if the transmit FIFO is "empty." */

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before nrf53_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in nrf53_lowsetup() and main clock
 *   initialization performed in nrf_clock_configure().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void nrf53_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  nrf53_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that nrf53_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   None
 *
 * Returns Value:
 *   None
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  unsigned minor = 0;
  unsigned i     = 0;
  char devname[16];

#ifdef HAVE_UART_CONSOLE
  /* Register the serial console */

  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  uart_register("/dev/ttyS0", &TTYS0_DEV);
  minor = 1;

  /* Register all remaining UARTs */

  strlcpy(devname, "/dev/ttySx", sizeof(devname));

  for (i = 0; i < NRF53_NUART; i++)
    {
      /* Don't create a device for non-configured ports. */

      if (g_uart_devs[i] == 0)
        {
          continue;
        }

      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->isconsole)
        {
          continue;
        }

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + minor++;
      uart_register(devname, g_uart_devs[i]);
    }
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
  /* struct nrf53_dev_s *priv = (struct nrf53_dev_s *)CONSOLE_DEV.priv; */

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
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

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  return ch;
#endif
}

#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER */
