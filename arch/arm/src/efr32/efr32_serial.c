/****************************************************************************
 * arch/arm/src/efr32/efr32_serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "efr32_config.h"
#include "efr32_lowputc.h"
#include "hardware/efr32_usart.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which USART is the console, and which is /dev/ttyS0 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart0port
#endif

#if defined(CONFIG_EFR32_USART0)
#  define TTYS0_DEV       g_usart0port
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct efr32_usart_s
{
  uintptr_t base;         /* Base address of the USART registers */
  uint32_t  baud;         /* Configured baud rate */
  uint32_t  ien;          /* Interrupts enabled by the upper half */
  uint8_t   rxirq;        /* RX IRQ number */
  uint8_t   txirq;        /* TX IRQ number */
  uint8_t   parity;       /* 0 = none, 1 = odd, 2 = even */
  uint8_t   bits;         /* Number of data bits */
  bool      stop2;        /* True: two stop bits */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  efr32_setup(struct uart_dev_s *dev);
static void efr32_shutdown(struct uart_dev_s *dev);
static int  efr32_attach(struct uart_dev_s *dev);
static void efr32_detach(struct uart_dev_s *dev);
static int  efr32_rxinterrupt(int irq, void *context, void *arg);
static int  efr32_txinterrupt(int irq, void *context, void *arg);
static int  efr32_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  efr32_receive(struct uart_dev_s *dev, unsigned int *status);
static void efr32_rxint(struct uart_dev_s *dev, bool enable);
static bool efr32_rxavailable(struct uart_dev_s *dev);
static void efr32_send(struct uart_dev_s *dev, int ch);
static void efr32_txint(struct uart_dev_s *dev, bool enable);
static bool efr32_txready(struct uart_dev_s *dev);
static bool efr32_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = efr32_setup,
  .shutdown       = efr32_shutdown,
  .attach         = efr32_attach,
  .detach         = efr32_detach,
  .ioctl          = efr32_ioctl,
  .receive        = efr32_receive,
  .rxint          = efr32_rxint,
  .rxavailable    = efr32_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = efr32_send,
  .txint          = efr32_txint,
  .txready        = efr32_txready,
  .txempty        = efr32_txempty,
};

#ifdef CONFIG_EFR32_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];

static struct efr32_usart_s g_usart0priv =
{
  .base   = EFR32_USART0_BASE,
  .baud   = CONFIG_USART0_BAUD,
  .rxirq  = EFR32_IRQ_USART0_RX,
  .txirq  = EFR32_IRQ_USART0_TX,
  .parity = CONFIG_USART0_PARITY,
  .bits   = CONFIG_USART0_BITS,
  .stop2  = CONFIG_USART0_2STOP != 0,
};

static struct uart_dev_s g_usart0port =
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
  .ops      = &g_uart_ops,
  .priv     = &g_usart0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efr32_serialin / efr32_serialout
 ****************************************************************************/

static inline uint32_t efr32_serialin(struct efr32_usart_s *priv,
                                      int offset)
{
  return getreg32(priv->base + offset);
}

static inline void efr32_serialout(struct efr32_usart_s *priv, int offset,
                                   uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: efr32_setien
 *
 * Description:
 *   Write the interrupt enables of the upper half to the hardware.
 *
 ****************************************************************************/

static void efr32_setien(struct efr32_usart_s *priv, uint32_t ien)
{
  priv->ien = ien;
  efr32_serialout(priv, EFR32_USART_IEN_OFFSET, ien);
}

/****************************************************************************
 * Name: efr32_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc.  This method is called
 *   the first time that the serial port is opened.
 *
 ****************************************************************************/

static int efr32_setup(struct uart_dev_s *dev)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  /* The console was already configured by efr32_lowsetup() */

  if (!dev->isconsole)
    {
      efr32_usart_configure(priv->base, priv->baud, priv->parity,
                            priv->bits, priv->stop2);
    }

  efr32_setien(priv, 0);
  return OK;
}

/****************************************************************************
 * Name: efr32_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial port is
 *   closed.  The console is left running.
 *
 ****************************************************************************/

static void efr32_shutdown(struct uart_dev_s *dev)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  efr32_setien(priv, 0);

  if (!dev->isconsole)
    {
      efr32_usart_reset(priv->base);
    }
}

/****************************************************************************
 * Name: efr32_attach
 *
 * Description:
 *   Configure the USART to operate in interrupt driven mode.  This method
 *   is called when the serial port is opened, after setup().
 *
 ****************************************************************************/

static int efr32_attach(struct uart_dev_s *dev)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->rxirq, efr32_rxinterrupt, dev);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(priv->txirq, efr32_txinterrupt, dev);
  if (ret < 0)
    {
      irq_detach(priv->rxirq);
      return ret;
    }

  up_enable_irq(priv->rxirq);
  up_enable_irq(priv->txirq);
  return OK;
}

/****************************************************************************
 * Name: efr32_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port
 *   is closed, before shutdown().
 *
 ****************************************************************************/

static void efr32_detach(struct uart_dev_s *dev)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  up_disable_irq(priv->rxirq);
  up_disable_irq(priv->txirq);
  irq_detach(priv->rxirq);
  irq_detach(priv->txirq);
}

/****************************************************************************
 * Name: efr32_rxinterrupt
 *
 * Description:
 *   USART RX interrupt handler.  RXDATAV clears itself when the receive
 *   buffer has been read; the error flags must be cleared here.
 *
 ****************************************************************************/

static int efr32_rxinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  efr32_serialout(priv, EFR32_USART_IF_OFFSET + EFR32_CLR_OFFSET,
                  USART_INT_RXFULL | USART_INT_RXOF | USART_INT_PERR |
                  USART_INT_FERR);

  if ((priv->ien & USART_INT_RXDATAV) != 0)
    {
      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: efr32_txinterrupt
 *
 * Description:
 *   USART TX interrupt handler.  TXBL clears itself when the transmit
 *   buffer is full.
 *
 ****************************************************************************/

static int efr32_txinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  if ((priv->ien & USART_INT_TXBL) != 0)
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: efr32_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int efr32_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;
  struct termios *termiosp = (struct termios *)arg;
  irqstate_t flags;
  uint32_t ien;
#endif
  int ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          if (termiosp == NULL)
            {
              ret = -EINVAL;
              break;
            }

          termiosp->c_cflag =
            (priv->parity != 0 ? PARENB : 0) |
            (priv->parity == 1 ? PARODD : 0) |
            (priv->stop2 ? CSTOPB : 0) |
            (priv->bits == 5 ? CS5 : priv->bits == 6 ? CS6 :
             priv->bits == 7 ? CS7 : CS8);

          cfsetispeed(termiosp, priv->baud);
        }
        break;

      case TCSETS:
        {
          if (termiosp == NULL)
            {
              ret = -EINVAL;
              break;
            }

          switch (termiosp->c_cflag & CSIZE)
            {
              case CS5:
                priv->bits = 5;
                break;

              case CS6:
                priv->bits = 6;
                break;

              case CS7:
                priv->bits = 7;
                break;

              default:
                priv->bits = 8;
                break;
            }

          if ((termiosp->c_cflag & PARENB) != 0)
            {
              priv->parity = (termiosp->c_cflag & PARODD) != 0 ? 1 : 2;
            }
          else
            {
              priv->parity = 0;
            }

          priv->stop2 = (termiosp->c_cflag & CSTOPB) != 0;
          priv->baud  = cfgetispeed(termiosp);

          /* Reconfigure with the interrupts held off */

          flags = enter_critical_section();
          ien   = priv->ien;
          efr32_usart_configure(priv->base, priv->baud, priv->parity,
                                priv->bits, priv->stop2);
          efr32_setien(priv, ien);
          leave_critical_section(flags);
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: efr32_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character
 *   from the USART.  Error bits associated with the receipt are provided
 *   in the return 'status'.
 *
 ****************************************************************************/

static int efr32_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;
  uint32_t rxdatax;

  rxdatax = efr32_serialin(priv, EFR32_USART_RXDATAX_OFFSET);
  *status = rxdatax & (USART_RXDATAX_PERR | USART_RXDATAX_FERR);

  return (int)(rxdatax & USART_RXDATAX_RXDATA_MASK);
}

/****************************************************************************
 * Name: efr32_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void efr32_rxint(struct uart_dev_s *dev, bool enable)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;
  irqstate_t flags;
  uint32_t ien;

  flags = enter_critical_section();

  ien = priv->ien;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      ien |= USART_INT_RXDATAV;
#endif
    }
  else
    {
      ien &= ~USART_INT_RXDATAV;
    }

  efr32_setien(priv, ien);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: efr32_rxavailable
 *
 * Description:
 *   Return true if the receive buffer is not empty
 *
 ****************************************************************************/

static bool efr32_rxavailable(struct uart_dev_s *dev)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  return (efr32_serialin(priv, EFR32_USART_STATUS_OFFSET) &
          USART_STATUS_RXDATAV) != 0;
}

/****************************************************************************
 * Name: efr32_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void efr32_send(struct uart_dev_s *dev, int ch)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  efr32_serialout(priv, EFR32_USART_TXDATA_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: efr32_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.  TXBL is set while the
 *   transmit buffer has room, so enabling it raises the interrupt at once.
 *
 ****************************************************************************/

static void efr32_txint(struct uart_dev_s *dev, bool enable)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;
  irqstate_t flags;
  uint32_t ien;

  flags = enter_critical_section();

  ien = priv->ien;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      ien |= USART_INT_TXBL;
#endif
    }
  else
    {
      ien &= ~USART_INT_TXBL;
    }

  efr32_setien(priv, ien);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: efr32_txready
 *
 * Description:
 *   Return true if the transmit buffer has room for another byte
 *
 ****************************************************************************/

static bool efr32_txready(struct uart_dev_s *dev)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  return (efr32_serialin(priv, EFR32_USART_STATUS_OFFSET) &
          USART_STATUS_TXBL) != 0;
}

/****************************************************************************
 * Name: efr32_txempty
 *
 * Description:
 *   Return true if the transmit buffer and shift register are empty
 *
 ****************************************************************************/

static bool efr32_txempty(struct uart_dev_s *dev)
{
  struct efr32_usart_s *priv = (struct efr32_usart_s *)dev->priv;

  return (efr32_serialin(priv, EFR32_USART_STATUS_OFFSET) &
          USART_STATUS_TXC) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that
 *   the serial console will be available during boot up.  This must be
 *   called before arm_serialinit.  The pins and the console format were
 *   set up by efr32_lowsetup().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  efr32_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
}

#endif /* USE_SERIALDRIVER */
