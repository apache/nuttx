/****************************************************************************
 * arch/arm/src/nrf91/nrf91_serial.c
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
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
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
#include "nrf91_config.h"
#include "hardware/nrf91_uarte.h"
#include "nrf91_clockconfig.h"
#include "nrf91_lowputc.h"
#include "nrf91_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many UARTs are supported */

#ifdef HAVE_UART1
#  define NRF91_NUART 2
#else
#  define NRF91_NUART 1
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
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_uart1port /* UART1 is console */
#  define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#endif

#define RX_INTERRUPTS (UART_INT_ENDRX | UART_INT_ERROR | UART_INT_RXTO)
#define TX_INTERRUPTS (UART_INT_ENDTX | UART_INT_TXSTOPPED)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one UART device */

struct nrf91_dev_s
{
  uintptr_t     uartbase;       /* Base address of UART registers */
  uint8_t       irq;            /* IRQ associated with this UART */
  uint8_t       rxdma;          /* Receive DMA buffer */
  uint8_t       txdma;          /* Transmit DMA buffer */
  uint8_t       rxchar;         /* Completed receive byte */
  uint32_t      rxstatus;       /* Receive error status */
  bool          rxvalid;        /* Receive byte available */
  bool          rxenabled;      /* Upper-half reception enabled */
  bool          rxactive;       /* Receiver started */
  bool          txenabled;      /* Upper-half transmission enabled */
  volatile bool txbusy;         /* DMA transmitter owns txdma */

  /* UART configuration */

  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  nrf91_setup(struct uart_dev_s *dev);
static void nrf91_shutdown(struct uart_dev_s *dev);
static int  nrf91_attach(struct uart_dev_s *dev);
static void nrf91_detach(struct uart_dev_s *dev);
static int  nrf91_interrupt(int irq, void *context, void *arg);
static int  nrf91_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  nrf91_receive(struct uart_dev_s *dev, unsigned int *status);
static void nrf91_rxint(struct uart_dev_s *dev, bool enable);
static bool nrf91_rxavailable(struct uart_dev_s *dev);
static void nrf91_send(struct uart_dev_s *dev, int ch);
static void nrf91_txint(struct uart_dev_s *dev, bool enable);
static bool nrf91_txready(struct uart_dev_s *dev);
static bool nrf91_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = nrf91_setup,
  .shutdown       = nrf91_shutdown,
  .attach         = nrf91_attach,
  .detach         = nrf91_detach,
  .ioctl          = nrf91_ioctl,
  .receive        = nrf91_receive,
  .rxint          = nrf91_rxint,
  .rxavailable    = nrf91_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = nrf91_send,
  .txint          = nrf91_txint,
  .txready        = nrf91_txready,
  .txempty        = nrf91_txempty,
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

/* This describes the state of the NRF91 UART0 port. */

#ifdef HAVE_UART0
static struct nrf91_dev_s g_uart0priv =
{
  .uartbase       = NRF91_UART0_BASE,
  .irq            = NRF91_IRQ_SERIAL0,
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

/* This describes the state of the NRF91 UART1 port. */

#ifdef HAVE_UART1
static struct nrf91_dev_s g_uart1priv =
{
  .uartbase       = NRF91_UART1_BASE,
  .irq            = NRF91_IRQ_SERIAL1,
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

static struct uart_dev_s * const g_uart_devs[NRF91_NUART] =
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
 * Name: nrf91_uart_putreg
 *
 * Description:
 *   Write a UART register.
 *
 ****************************************************************************/

static inline void nrf91_uart_putreg(struct nrf91_dev_s *priv,
                                     uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: nrf91_uart_getreg
 *
 * Description:
 *   Read a UART register.
 *
 ****************************************************************************/

static inline uint32_t nrf91_uart_getreg(struct nrf91_dev_s *priv,
                                         uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: nrf91_uart_clrevent
 *
 * Description:
 *   Clear a UARTE event and read it back before returning.
 *
 ****************************************************************************/

static void nrf91_uart_clrevent(struct nrf91_dev_s *priv, uint32_t event)
{
  nrf91_uart_putreg(priv, event, 0);
  (void)nrf91_uart_getreg(priv, event);
}

/****************************************************************************
 * Name: nrf91_rxstart
 *
 * Description:
 *   Start reception and queue the first one-byte DMA buffer.
 *
 ****************************************************************************/

static void nrf91_rxstart(struct nrf91_dev_s *priv)
{
  priv->rxactive = true;
  priv->rxvalid = false;
  nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_ENDRX_OFFSET);
  nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_RXTO_OFFSET);
  nrf91_uart_putreg(priv, NRF91_UARTE_RXD_PTR_OFFSET,
                    (uintptr_t)&priv->rxdma);
  nrf91_uart_putreg(priv, NRF91_UARTE_RXD_MAXCNT_OFFSET, 1);
  UP_DMB();
  nrf91_uart_putreg(priv, NRF91_UARTE_TASKS_STARTRX_OFFSET, 1);
}

/****************************************************************************
 * Name: nrf91_rxdeliver
 *
 * Description:
 *   Deliver the completed byte and restart the one-byte DMA. A byte
 *   received with the delivery disabled is kept, the UART FIFO holds the
 *   next bytes until rxint() enables the delivery.
 *
 ****************************************************************************/

static void nrf91_rxdeliver(struct uart_dev_s *dev)
{
  struct nrf91_dev_s *priv = dev->priv;

  if (priv->rxvalid && priv->rxenabled)
    {
      uart_recvchars(dev);
      if (!priv->rxvalid)
        {
          UP_DMB();
          nrf91_uart_putreg(priv, NRF91_UARTE_TASKS_STARTRX_OFFSET, 1);
        }
    }
}

/****************************************************************************
 * Name: nrf91_rxstop
 *
 * Description:
 *   Stop reception and deliver a completed byte. Interrupts must be
 *   disabled.
 *
 ****************************************************************************/

static void nrf91_rxstop(struct uart_dev_s *dev)
{
  struct nrf91_dev_s *priv = dev->priv;

  nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_RXTO_OFFSET);
  nrf91_uart_putreg(priv, NRF91_UARTE_TASKS_STOPRX_OFFSET, 1);
  while (!nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_RXTO_OFFSET))
    {
    }

  if (nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_ERROR_OFFSET))
    {
      nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_ERROR_OFFSET);
      priv->rxstatus |=
        nrf91_uart_getreg(priv, NRF91_UARTE_ERRORSRC_OFFSET);
      nrf91_uart_putreg(priv, NRF91_UARTE_ERRORSRC_OFFSET, priv->rxstatus);
    }

  /* STOP also generates ENDRX for an empty buffer. Deliver a completed
   * byte, but never deliver an empty buffer.
   */

  if (nrf91_uart_getreg(priv, NRF91_UARTE_RXD_AMOUNT_OFFSET) != 0)
    {
      UP_DMB();
      priv->rxchar = priv->rxdma;
      priv->rxvalid = true;
      if (priv->rxenabled)
        {
          uart_recvchars(dev);
        }

      priv->rxvalid = false;
    }

  nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_ENDRX_OFFSET);
}

/****************************************************************************
 * Name: nrf91_txstart
 *
 * Description:
 *   Start a one-byte DMA transfer. The caller must serialize access.
 *
 ****************************************************************************/

static void nrf91_txstart(struct nrf91_dev_s *priv, int ch)
{
  priv->txdma = ch;
  priv->txbusy = true;
  nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_ENDTX_OFFSET);
  nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_TXSTOPPED_OFFSET);
  nrf91_uart_putreg(priv, NRF91_UARTE_TXD_PTR_OFFSET,
                    (uintptr_t)&priv->txdma);
  nrf91_uart_putreg(priv, NRF91_UARTE_TXD_MAXCNT_OFFSET, 1);
  UP_DMB();
  nrf91_uart_putreg(priv, NRF91_UARTE_TASKS_STARTTX_OFFSET, 1);
}

/****************************************************************************
 * Name: nrf91_txstop
 *
 * Description:
 *   Wait for the transmitter to stop. Interrupts must be disabled.
 *
 ****************************************************************************/

static void nrf91_txstop(struct nrf91_dev_s *priv)
{
  if (priv->txbusy)
    {
      while (!nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_ENDTX_OFFSET) &&
             !nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_TXSTOPPED_OFFSET))
        {
        }

      if (!nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_TXSTOPPED_OFFSET))
        {
          nrf91_uart_putreg(priv, NRF91_UARTE_TASKS_STOPTX_OFFSET, 1);
          while (!nrf91_uart_getreg(priv,
                                    NRF91_UARTE_EVENTS_TXSTOPPED_OFFSET))
            {
            }
        }

      priv->txbusy = false;
    }
}

/****************************************************************************
 * Name: nrf91_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int nrf91_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;

  /* TODO: Configure the UART as an RS-232 UART */

  /* REVISIT: If nrf91_usart_configure() called 2nd time uart stops working.
   * Rx interrupt keeps firing.
   * configuring is done on __start
   *
   * UPDATE 19.12.2019: No problems described above were observed,
   * but just in case we leave the above note for some time.
   */

  nrf91_usart_configure(priv->uartbase, &priv->config);
#endif

  /* TODO: configure UART if not selected as console */

  return OK;
}

/****************************************************************************
 * Name: nrf91_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void nrf91_shutdown(struct uart_dev_s *dev)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;

  /* Disable interrupts */

  /* Reset hardware and disable Rx and Tx */

  nrf91_usart_disable(priv->uartbase, &priv->config);
}

/****************************************************************************
 * Name: nrf91_attach
 *
 * Description:
 *   Configure the UART to operate in interrupt driven mode. This method
 *   is called when the serial port is opened, normally just after setup.
 *   DMA completion interrupts release buffers even when delivery to the
 *   upper half is disabled by the rxint() and txint() methods.
 *
 ****************************************************************************/

static int nrf91_attach(struct uart_dev_s *dev)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, nrf91_interrupt, dev);
  if (ret == OK)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      nrf91_uart_putreg(priv, NRF91_UARTE_INTENSET_OFFSET,
                        RX_INTERRUPTS | TX_INTERRUPTS);
      up_enable_irq(priv->irq);
      nrf91_rxstart(priv);
#endif
    }

  return ret;
}

/****************************************************************************
 * Name: nrf91_detach
 *
 * Description:
 *   Detach UART interrupts. This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void nrf91_detach(struct uart_dev_s *dev)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;
  irqstate_t flags;

  /* Stop DMA before detaching the interrupt handler */

  flags = enter_critical_section();
  nrf91_uart_putreg(priv, NRF91_UARTE_INTENCLR_OFFSET, UINT32_MAX);

  if (priv->rxactive)
    {
      nrf91_rxstop(dev);
      priv->rxactive = false;
    }

  nrf91_txstop(priv);
  leave_critical_section(flags);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: nrf91_interrupt
 *
 * Description:
 *   Handle receive buffers and transfer completion.
 *
 ****************************************************************************/

static int nrf91_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = arg;
  struct nrf91_dev_s *priv = dev->priv;

  if (nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_ERROR_OFFSET))
    {
      nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_ERROR_OFFSET);
      priv->rxstatus |=
        nrf91_uart_getreg(priv, NRF91_UARTE_ERRORSRC_OFFSET);
      nrf91_uart_putreg(priv, NRF91_UARTE_ERRORSRC_OFFSET, priv->rxstatus);
    }

  /* Copy the completed byte before restarting DMA. Automatic ENDRX-to-
   * STARTRX would let DMA overwrite a buffer before the upper half
   * consumes it. The UART FIFO holds incoming bytes between transfers.
   */

  if (nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_ENDRX_OFFSET))
    {
      nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_ENDRX_OFFSET);
      UP_DMB();
      priv->rxchar = priv->rxdma;
      priv->rxvalid = true;
    }

  nrf91_rxdeliver(dev);
  nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_RXTO_OFFSET);

  /* There is no ENDTX-to-STOPTX shortcut */

  if (nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_ENDTX_OFFSET))
    {
      nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_ENDTX_OFFSET);
      nrf91_uart_putreg(priv, NRF91_UARTE_TASKS_STOPTX_OFFSET, 1);
    }

  if (nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_TXSTOPPED_OFFSET))
    {
      nrf91_uart_clrevent(priv, NRF91_UARTE_EVENTS_TXSTOPPED_OFFSET);
      priv->txbusy = false;
      if (priv->txenabled)
        {
          uart_xmitchars(dev);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf91_set_format
 *
 * Description:
 *   Set the serial line format and speed with DMA stopped.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TERMIOS
void nrf91_set_format(struct uart_dev_s *dev)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (priv->rxactive)
    {
      nrf91_rxstop(dev);
    }

  nrf91_txstop(priv);

  nrf91_uart_putreg(priv, NRF91_UARTE_ENABLE_OFFSET,
                    NRF91_UARTE_ENABLE_DISABLE);
  nrf91_usart_setformat(priv->uartbase, &priv->config);
  nrf91_uart_putreg(priv, NRF91_UARTE_ENABLE_OFFSET,
                    NRF91_UARTE_ENABLE_ENABLE);

  if (priv->rxactive)
    {
      nrf91_rxstart(priv);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: nrf91_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int nrf91_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode         *inode  = filep->f_inode;
  struct uart_dev_s    *dev    = inode->i_private;
  struct nrf91_dev_s   *priv   = (struct nrf91_dev_s *)dev->priv;
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

          nrf91_set_format(dev);

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
 * Name: nrf91_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int nrf91_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;

  if (status)
    {
      *status = priv->rxstatus;
    }

  priv->rxstatus = 0;
  priv->rxvalid = false;
  return priv->rxchar;
}

/****************************************************************************
 * Name: nrf91_rxint
 *
 * Description:
 *   Enable or disable delivery of received characters to the upper half.
 *
 ****************************************************************************/

static void nrf91_rxint(struct uart_dev_s *dev, bool enable)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_SUPPRESS_SERIAL_INTS
  enable = false;
#endif

  priv->rxenabled = enable;
  if (enable)
    {
      nrf91_rxdeliver(dev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf91_rxavailable
 *
 * Description:
 *   Report whether a received character is waiting for the upper half.
 *
 ****************************************************************************/

static bool nrf91_rxavailable(struct uart_dev_s *dev)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;

  return priv->rxvalid;
}

/****************************************************************************
 * Name: nrf91_send
 *
 * Description:
 *   Start transmitting the character supplied by the upper half.
 *
 ****************************************************************************/

static void nrf91_send(struct uart_dev_s *dev, int ch)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

  nrf91_txstart(priv, ch);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf91_txint
 *
 * Description:
 *   Enable or disable transmission from the upper-half transmit buffer.
 *
 ****************************************************************************/

static void nrf91_txint(struct uart_dev_s *dev, bool enable)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_SUPPRESS_SERIAL_INTS
  enable = false;
#endif

  priv->txenabled = enable;
  if (enable)
    {
      uart_xmitchars(dev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf91_txready
 *
 * Description:
 *   Report whether the transmitter can accept another character. The
 *   events are read, the upper half also polls with interrupts disabled.
 *   A transfer ended without STOPTX can be restarted.
 *
 ****************************************************************************/

static bool nrf91_txready(struct uart_dev_s *dev)
{
  struct nrf91_dev_s *priv = (struct nrf91_dev_s *)dev->priv;

  if (priv->txbusy)
    {
      return nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_ENDTX_OFFSET) ||
             nrf91_uart_getreg(priv, NRF91_UARTE_EVENTS_TXSTOPPED_OFFSET);
    }

  return true;
}

/****************************************************************************
 * Name: nrf91_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool nrf91_txempty(struct uart_dev_s *dev)
{
  return nrf91_txready(dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before nrf91_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in nrf91_lowsetup() and main clock
 *   initialization performed in nrf_clock_configure().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void nrf91_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  nrf91_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that nrf91_earlyserialinit was called previously.
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
  uart_register("/dev/ttyS0", &TTYS0_DEV);
  minor = 1;
#endif

  /* Register all remaining UARTs */

  strlcpy(devname, "/dev/ttySx", sizeof(devname));

  for (i = 0; i < NRF91_NUART; i++)
    {
      /* Don't create a device for non-configured ports. */

      if (g_uart_devs[i] == NULL)
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

void up_putc(int ch)
{
#ifdef HAVE_UART_CONSOLE
  struct nrf91_dev_s *priv = CONSOLE_DEV.priv;
  irqstate_t flags = enter_critical_section();

  /* Serialize polled debug output with the interrupt-driven transmitter */

  nrf91_txstop(priv);
  arm_lowputc(ch);
  leave_critical_section(flags);
#endif
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_UART_CONSOLE
  arm_lowputc(ch);
#endif
}

#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER */
