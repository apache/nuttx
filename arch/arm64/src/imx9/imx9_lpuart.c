/****************************************************************************
 * arch/arm64/src/imx9/imx9_lpuart.c
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

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>

#include "chip.h"
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_gic.h"

#include "imx9_serial.h"
#include "imx9_boot.h"

#include "hardware/imx9_lpuart.h"
#include "hardware/imx9_memorymap.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which UART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART1-5 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV             g_uart1port /* UART1 is console */
#  define TTYS0_DEV               g_uart1port /* UART1 is ttyS0 */
#  define UART1_ASSIGNED          1
#endif

/* Rx DMA timeout in ms, which is used to calculate Rx ring buffer size */
#define DMA_RX_TIMEOUT              (10)
#define UART_AUTOSUSPEND_TIMEOUT    3000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_uart_port_s
{
  unsigned long iobase;
  unsigned int  baud;         /* Configured baud */
  unsigned int  irq_num;
  unsigned int  txfifo_size;
  unsigned int  rxfifo_size;
  int           is_console;
};

static inline uint32_t imx9_read(struct imx9_uart_port_s *port, uint32_t off)
{
  return getreg32(port->iobase + off);
}

static inline void imx9_write(struct imx9_uart_port_s *port, uint32_t val,
                              uint32_t off)
{
  putreg32(val, port->iobase + off);
}

static int imx9_lpuart_stop_tx(struct imx9_uart_port_s *sport)
{
  unsigned long temp;

  temp  = imx9_read(sport, UARTCTRL);
  temp  &= ~(UARTCTRL_TIE | UARTCTRL_TCIE);
  imx9_write(sport, temp, UARTCTRL);

  return 0;
}

static int imx9_lpuart_stop_rx(struct imx9_uart_port_s *sport)
{
  unsigned long temp;

  temp = imx9_read(sport, UARTCTRL);
  imx9_write(sport, temp & ~UARTCTRL_RE, UARTCTRL);

  return 0;
}

static int imx9_lpuart_start_tx(struct imx9_uart_port_s *sport)
{
  unsigned long temp;

  temp = imx9_read(sport, UARTCTRL);
  imx9_write(sport, temp | UARTCTRL_TIE, UARTCTRL);

  return 0;
}

static int imx9_lpuart_start_rx(struct imx9_uart_port_s *sport)
{
  unsigned long temp;

  temp = imx9_read(sport, UARTCTRL);
  imx9_write(sport, temp | UARTCTRL_RE, UARTCTRL);

  return 0;
}

/****************************************************************************
 * Name: imx9_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void imx9_rxint(struct uart_dev_s *dev, bool enable)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;

  if (enable)
    {
      imx9_lpuart_start_rx(sport);
    }
  else
    {
      imx9_lpuart_stop_rx(sport);
    }
}

/****************************************************************************
 * Name: imx9_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void imx9_txint(struct uart_dev_s *dev, bool enable)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;

  if (enable)
    {
      imx9_lpuart_start_tx(sport);
    }
  else
    {
      imx9_lpuart_stop_tx(sport);
    }
}

/****************************************************************************
 * Name: imx9_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void imx9_send(struct uart_dev_s *dev, int ch)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;

  imx9_write(sport, ch, UARTDATA);
}

/****************************************************************************
 * Name: imx9_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int imx9_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;
  unsigned int             rx;
  unsigned int             sr;

  /* to clear the FE, OR, NF, FE, PE flags,
   * read STAT then read DATA reg
   */

  sr = imx9_read(sport, UARTSTAT);
  rx = imx9_read(sport, UARTDATA);

  *status = sr;

  return rx;
}

/****************************************************************************
 * Name: imx9_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool imx9_rxavailable(struct uart_dev_s *dev)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;
  unsigned long            sfifo = imx9_read(sport, UARTFIFO);

  if (sfifo & UARTFIFO_RXEMPT)
    {
      return false;
    }
  else
    {
      return true;
    }
}

/****************************************************************************
 * Name: imx9_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool imx9_txready(struct uart_dev_s *dev)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;
  unsigned long            txcnt;

  /* When TXFULL is set, there is no space in the Tx FIFO  */

  txcnt = imx9_read(sport, UARTWATER);
  txcnt = txcnt >> UARTWATER_TXCNT_OFF;
  txcnt &= UARTWATER_COUNT_MASK;

  if (txcnt < sport->txfifo_size)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: imx9_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool imx9_txempty(struct uart_dev_s *dev)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;
  unsigned long            stat    = imx9_read(sport, UARTSTAT);
  unsigned long            sfifo   = imx9_read(sport, UARTFIFO);

  if (stat & UARTSTAT_TC && sfifo & UARTFIFO_TXEMPT)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: imx9_irq_handler (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It should cal
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ****************************************************************************/

static int imx9_uart_irq_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s       *dev = (struct uart_dev_s *)arg;
  struct imx9_uart_port_s *sport;
  unsigned long            sts;
  unsigned long            rxcount;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  sport = (struct imx9_uart_port_s *)dev->priv;

  sts     = imx9_read(sport, UARTSTAT);
  rxcount = imx9_read(sport, UARTWATER);
  rxcount = rxcount >> UARTWATER_RXCNT_OFF;

  if (sts & UARTSTAT_RDRF || rxcount > 0)
    {
      uart_recvchars(dev);
    }

  if (sts & UARTSTAT_TDRE)
    {
      uart_xmitchars(dev);
    }

  imx9_write(sport, sts, UARTSTAT);
  return OK;
}

static int imx9_lpuart_hw_reset(struct imx9_uart_port_s *sport)
{
  if (sport->is_console)
    {
      return 0;
    }

  /* Toggle the reset signal to reset and release the peripheral */

  putreg32(UARTGLOBAL_RST, sport->iobase + UARTGLOBAL);
  putreg32(0, sport->iobase + UARTGLOBAL);

  return 0;
}

static int imx9_setup(struct uart_dev_s *dev)
{
  irqstate_t               i_flags;
  int                      ret;
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;

  i_flags = up_irq_save();

  ret = imx9_lpuart_hw_reset(sport);
  if (ret < 0)
    {
      serr("failed to reset hw: %d\n", ret);
    }

  up_irq_restore(i_flags);

  return ret;
}

/****************************************************************************
 * Name: imx9_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void imx9_shutdown(struct uart_dev_s *dev)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;
  unsigned long            temp;
  irqstate_t               i_flags;

  i_flags = up_irq_save();

  /* clear statue */

  temp = imx9_read(sport, UARTSTAT);
  imx9_write(sport, temp, UARTSTAT);

  /* disable Rx/Tx DMA */

  temp = imx9_read(sport, UARTBAUD);
  temp &= ~(UARTBAUD_TDMAE | UARTBAUD_RDMAE | UARTBAUD_RIDMAE);
  imx9_write(sport, temp, UARTBAUD);

  /* disable Rx/Tx and interrupts */

  temp = imx9_read(sport, UARTCTRL);
  temp &= ~(UARTCTRL_TE | UARTCTRL_RE | UARTCTRL_TIE |
            UARTCTRL_TCIE | UARTCTRL_RIE | UARTCTRL_ILIE |
            UARTCTRL_LOOPS);
  imx9_write(sport, temp, UARTCTRL);
  imx9_write(sport, 0, UARTMODIR);

  up_irq_restore(i_flags);
}

static void imx9_setup_watermark(struct imx9_uart_port_s *sport)
{
  unsigned long val;
  unsigned long ctrl;
  unsigned long ctrl_saved;
  unsigned long rxiden_cnt;

  ctrl       = imx9_read(sport, UARTCTRL);
  ctrl_saved = ctrl;
  ctrl       &= ~(UARTCTRL_TIE | UARTCTRL_TCIE | UARTCTRL_TE |
                  UARTCTRL_RIE | UARTCTRL_RE);
  imx9_write(sport, ctrl, UARTCTRL);

  /* enable FIFO mode */

  val         = imx9_read(sport, UARTFIFO);
  val        |= UARTFIFO_TXFE | UARTFIFO_RXFE;
  val        |= UARTFIFO_TXFLUSH | UARTFIFO_RXFLUSH;
  val        &= ~(UARTFIFO_RXIDEN_MASK << UARTFIFO_RXIDEN_OFF);
  rxiden_cnt = 0;
  val        |= ((rxiden_cnt & UARTFIFO_RXIDEN_MASK) << UARTFIFO_RXIDEN_OFF);

  imx9_write(sport, val, UARTFIFO);

  /* set the watermark
   * rx_watermark = 1;
   */

  val = (1 << UARTWATER_RXWATER_OFF) | (0x0 << UARTWATER_TXWATER_OFF);
  imx9_write(sport, val, UARTWATER);

  /* Restore cr2 */

  imx9_write(sport, ctrl_saved, UARTCTRL);
}

static void imx9_setup_watermark_enable(struct imx9_uart_port_s *sport)
{
  uint32_t temp;

  imx9_setup_watermark(sport);

  temp = imx9_read(sport, UARTCTRL);
  temp |= UARTCTRL_RE | UARTCTRL_TE;
  temp |= UARTCTRL_IDLECFG << UARTCTRL_IDLECFG_OFF;
  imx9_write(sport, temp, UARTCTRL);
}

static void imx9_hw_disable(struct imx9_uart_port_s *sport)
{
  unsigned long temp;

  temp  = imx9_read(sport, UARTCTRL);
  temp  &= ~(UARTCTRL_RIE | UARTCTRL_ILIE | UARTCTRL_RE |
             UARTCTRL_TIE | UARTCTRL_TE);
  imx9_write(sport, temp, UARTCTRL);
}

static void imx9_configure(struct imx9_uart_port_s *sport)
{
  unsigned long temp;

  temp = imx9_read(sport, UARTCTRL);
  temp |= UARTCTRL_RIE | UARTCTRL_ILIE;
  temp |= UARTCTRL_TIE;
  imx9_write(sport, temp, UARTCTRL);
}

static void imx9_hw_setup(struct imx9_uart_port_s *sport)
{
  irqstate_t i_flags;

  i_flags = up_irq_save();

  imx9_hw_disable(sport);

  imx9_setup_watermark_enable(sport);
  imx9_configure(sport);
  up_enable_irq(sport->irq_num);

  up_irq_restore(i_flags);
}

/****************************************************************************
 * Name: imx9_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int imx9_attach(struct uart_dev_s *dev)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;
  int                      ret;
  unsigned long            temp;

  /* determine FIFO size */

  temp = imx9_read(sport, UARTFIFO);

  sport->txfifo_size = UARTFIFO_DEPTH(
    (temp >> UARTFIFO_TXSIZE_OFF) & UARTFIFO_FIFOSIZE_MASK);
  sport->rxfifo_size = UARTFIFO_DEPTH(
    (temp >> UARTFIFO_RXSIZE_OFF) & UARTFIFO_FIFOSIZE_MASK);

  ret = irq_attach(sport->irq_num, imx9_uart_irq_handler, dev);

  if (ret == OK)
    {
      imx9_hw_setup(sport);
    }
  else
    {
      sinfo("error ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: imx_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void imx9_detach(struct uart_dev_s *dev)
{
  struct imx9_uart_port_s *sport = (struct imx9_uart_port_s *)dev->priv;

  up_disable_irq(sport->irq_num);
  irq_detach(sport->irq_num);
}

/****************************************************************************
 * Name: imx9_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *   for current imx9 configure.
 *
 ****************************************************************************/

static int imx9_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || defined(CONFIG_SERIAL_TERMIOS)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  irqstate_t        flags;
#endif
  int               ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
    {
      struct imx_uart_s *user = (struct imx_uart_s *)arg;
      if (!user)
        {
          ret = -EINVAL;
        }
      else
        {
          memcpy(user, dev, sizeof(struct imx_uart_s));
        }
    }
    break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
    {
      struct termios    *termiosp = (struct termios *)arg;
      struct imx_uart_s *priv     = (struct imx_uart_s *)dev->priv;

      if (!termiosp)
        {
          ret = -EINVAL;
          break;
        }

      /* Return parity */

      termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                         ((priv->parity == 1) ? PARODD : 0);

      /* Return stop bits */

      termiosp->c_cflag |= (priv->stopbits2) ? CSTOPB : 0;

      /* Return flow control */

#ifdef CONFIG_SERIAL_OFLOWCONTROL
      termiosp->c_cflag |= ((priv->oflow) ? CCTS_OFLOW : 0);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
      termiosp->c_cflag |= ((priv->iflow) ? CRTS_IFLOW : 0);
#endif

      /* Return baud */

      cfsetispeed(termiosp, priv->baud);

      /* Return number of bits */

      switch (priv->bits)
        {
        case 5:
        {
          termiosp->c_cflag |= CS5;
          break;
        }

        case 6:
        {
          termiosp->c_cflag |= CS6;
          break;
        }

        case 7:
        {
          termiosp->c_cflag |= CS7;
          break;
        }

        default:
        case 8:
        {
          termiosp->c_cflag |= CS8;
          break;
        }

#if defined(CS9)
        case 9:
        {
          termiosp->c_cflag |= CS9;
          break;
        }
#endif
        }
    }
    break;

    case TCSETS:
    {
      struct termios    *termiosp = (struct termios *)arg;
      struct imx_uart_s *priv     = (struct imx_uart_s *)dev->priv;
      uint32_t           baud;
      uint32_t           ie;
      uint8_t            parity;
      uint8_t            nbits;
      bool               stop2;

      if ((!termiosp)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#endif
          )
        {
          ret = -EINVAL;
          break;
        }

      /* Decode baud. */

      ret  = OK;
      baud = cfgetispeed(termiosp);

      /* Decode number of bits */

      switch (termiosp->c_cflag & CSIZE)
        {
        case CS5:
        {
          nbits = 5;
          break;
        }

        case CS6:
        {
          nbits = 6;
          break;
        }

        case CS7:
        {
          nbits = 7;
          break;
        }

        case CS8:
        {
          nbits = 8;
          break;
        }

#if defined(CS9)
        case CS9:
        {
          nbits = 9;
          break;
        }

#endif
        default:
        {
          ret = -EINVAL;
          break;
        }
        }

      /* Decode parity */

      if ((termiosp->c_cflag & PARENB) != 0)
        {
          parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
        }
      else
        {
          parity = 0;
        }

      /* Decode stop bits */

      stop2 = (termiosp->c_cflag & CSTOPB) != 0;

      /* Verify that all settings are valid before committing */

      if (ret == OK)
        {
          /* Commit */

          priv->baud      = baud;
          priv->parity    = parity;
          priv->bits      = nbits;
          priv->stopbits2 = stop2;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

          /* effect the changes immediately - note that we do not
           * implement TCSADRAIN / TCSAFLUSH
           */

          flags = spin_lock_irqsave(NULL);
          imx9_disableuartint(priv, &ie);
          ret = imx9_setup(dev);

          /* Restore the interrupt state */

          imx_restoreuartint(priv, ie);
          priv->ie = ie;
          spin_unlock_irqrestore(NULL, flags);
        }
    }
    break;
#endif /* CONFIG_SERIAL_TERMIOS */

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
    default:
    {
      ret = -ENOTTY;
      break;
    }
    }

  return ret;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup       = imx9_setup,
  .shutdown    = imx9_shutdown,
  .attach      = imx9_attach,
  .detach      = imx9_detach,
  .ioctl       = imx9_ioctl,
  .receive     = imx9_receive,
  .rxint       = imx9_rxint,
  .rxavailable = imx9_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send        = imx9_send,
  .txint       = imx9_txint,
  .txready     = imx9_txready,
  .txempty     = imx9_txempty,
};

/* I/O buffers */

#ifdef CONFIG_IMX9_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

/* This describes the state of the IMX uart1 port. */

static struct imx9_uart_port_s g_uart1priv =
{
  .iobase     = IMX9_LPUART1_BASE,
  .baud       = CONFIG_UART1_BAUD,
  .irq_num    = IMX9_IRQ_LPUART1,
  .is_console = 1,
};

static struct uart_dev_s g_uart1port =
{
  .recv =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart1priv,
};

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm64_serialinit.
 *
 ****************************************************************************/

void arm64_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function imx9_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */
#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  imx9_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm64_lowputc('\r');
    }

  arm64_lowputc((uint8_t)ch);
  return ch;
}

/****************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that imx_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm64_serialinit(void)
{
  int ret;

  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      sinfo("error at register dev/console, ret =%d\n", ret);
    }

  ret = uart_register("/dev/ttyS0", &TTYS0_DEV);
  if (ret < 0)
    {
      sinfo("error at register dev/ttyS0, ret =%d\n", ret);
    }
}

#endif /* USE_SERIALDRIVER */
