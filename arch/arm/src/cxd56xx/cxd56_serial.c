/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_serial.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 *   Copyright (C) 2012-2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "cxd56_config.h"
#include "cxd56_serial.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase; /* Base address of UART registers */
  uint32_t basefreq;  /* Base frequency of input clock */
  uint32_t baud;      /* Configured baud */
  uint32_t ier;       /* Saved IER value */
  uint8_t id;         /* ID=0,1,2,3 */
  uint8_t irq;        /* IRQ associated with this UART */
  uint8_t parity;     /* 0=none, 1=odd, 2=even */
  uint8_t bits;       /* Number of bits (7 or 8) */
  bool stopbits2;     /* true: Configure with 2 stop bits instead of 1 */
#ifdef HAVE_RS485
  bool dtrdir;        /* DTR pin is the direction bit */
#endif
  void *pmhandle;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int up_setup(FAR struct uart_dev_s *dev);
static void up_shutdown(FAR struct uart_dev_s *dev);
static int up_attach(FAR struct uart_dev_s *dev);
static void up_detach(FAR struct uart_dev_s *dev);
static int up_interrupt(int irq, FAR void *context, FAR void *arg);
static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifdef CONFIG_UART2_IFLOWCONTROL
static bool up_rxflowcontrol(FAR struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper);
#endif
static int up_receive(FAR struct uart_dev_s *dev, FAR uint32_t *status);
static void up_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(FAR struct uart_dev_s *dev);
static void up_send(FAR struct uart_dev_s *dev, int ch);
static void up_txint(FAR struct uart_dev_s *dev, bool enable);
static bool up_txready(FAR struct uart_dev_s *dev);
static bool up_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup         = up_setup,
  .shutdown      = up_shutdown,
  .attach        = up_attach,
  .detach        = up_detach,
  .ioctl         = up_ioctl,
  .receive       = up_receive,
  .rxint         = up_rxint,
  .rxavailable   = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send          = up_send,
  .txint         = up_txint,
  .txready       = up_txready,
  .txempty       = up_txempty,
};

#ifdef CONFIG_CXD56_UART2
static const struct uart_ops_s g_uart2_ops =
{
  .setup         = up_setup,
  .shutdown      = up_shutdown,
  .attach        = up_attach,
  .detach        = up_detach,
  .ioctl         = up_ioctl,
  .receive       = up_receive,
  .rxint         = up_rxint,
  .rxavailable   = up_rxavailable,
#ifdef CONFIG_UART2_IFLOWCONTROL
  .rxflowcontrol = up_rxflowcontrol,
#endif
  .send          = up_send,
  .txint         = up_txint,
  .txready       = up_txready,
  .txempty       = up_txempty,
};
#endif

/* I/O buffers */

#ifdef CONFIG_CXD56_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_CXD56_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

/* This describes the state of the CXD56xx uart1 port. */

#ifdef CONFIG_CXD56_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase  = CXD56_UART1_BASE,
  .basefreq  = BOARD_UART1_BASEFREQ,
  .baud      = CONFIG_UART1_BAUD,
  .id        = 1,
  .irq       = CXD56_IRQ_UART1,
  .parity    = CONFIG_UART1_PARITY,
  .bits      = CONFIG_UART1_BITS,
  .stopbits2 = CONFIG_UART1_2STOP,
};

static uart_dev_t g_uart1port =
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
#  define TTYS0_DEV g_uart1port /* UART1=ttyS0 */
#endif

/* This describes the state of the CXD56xx uart1 port. */

#ifdef CONFIG_CXD56_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase  = CXD56_UART2_BASE,
  .basefreq  = BOARD_UART2_BASEFREQ,
  .baud      = CONFIG_UART2_BAUD,
  .id        = 2,
  .irq       = CXD56_IRQ_APP_UART,
  .parity    = CONFIG_UART2_PARITY,
  .bits      = CONFIG_UART2_BITS,
  .stopbits2 = CONFIG_UART2_2STOP,
};

static uart_dev_t g_uart2port =
{
  .recv =
    {
      .size   = CONFIG_UART2_RXBUFSIZE,
      .buffer = g_uart2rxbuffer,
    },
  .xmit =
    {
      .size   = CONFIG_UART2_TXBUFSIZE,
      .buffer = g_uart2txbuffer,
    },
  .ops  = &g_uart2_ops,
  .priv = &g_uart2priv,
};

#  define TTYS2_DEV g_uart2port /* UART2=ttyS2 */
#endif

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port /* UART1=console */
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2port /* UART2=console */
#  endif
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(FAR struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(FAR struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(FAR struct up_dev_s *priv,
                                     FAR uint32_t *ier)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (ier)
    {
      *ier = priv->ier & UART_INTR_ALL;
    }

  priv->ier &= ~UART_INTR_ALL;
  up_serialout(priv, CXD56_UART_IMSC, priv->ier);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(FAR struct up_dev_s *priv, uint32_t ier)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->ier |= ier & UART_INTR_ALL;
  up_serialout(priv, CXD56_UART_IMSC, priv->ier);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(FAR struct up_dev_s *priv, bool enable)
{
  uint32_t lcr = up_serialin(priv, CXD56_UART_LCR_H);
  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }

  up_serialout(priv, CXD56_UART_LCR_H, lcr);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(FAR struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  uint32_t lcr;
  uint32_t cr;

  cxd56_uart_setup(priv->id);

  /* Init HW */

  up_serialout(priv, CXD56_UART_CR, 0);
  up_serialout(priv, CXD56_UART_LCR_H, 0);
  up_serialout(priv, CXD56_UART_DMACR, 0);
  up_serialout(priv, CXD56_UART_RSR_ECR, 0xf);

  /* Set up the IER */

  priv->ier = up_serialin(priv, CXD56_UART_IMSC);

  /* Set the BAUD divisor */

  cxd56_setbaud(priv->uartbase, priv->basefreq, priv->baud);

  /* Set up the LCR */

  lcr = 0;
  if (priv->bits == 7)
    {
      lcr |= UART_LCR_WLEN(7);
    }
  else
    {
      lcr |= UART_LCR_WLEN(8);
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STP2;
    }

  if (priv->parity == 1)
    {
      lcr |= (UART_LCR_PEN);
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

  /* Save the LCR */

  up_serialout(priv, CXD56_UART_LCR_H, lcr);

  up_serialout(priv, CXD56_UART_IFLS, 0);
  up_serialout(priv, CXD56_UART_ICR, 0x7ff);

  cr = UART_CR_RXE | UART_CR_TXE;

  /* Enable Auto-RTS and Auto-CS Flow Control in the Modem Control Register */

#  ifdef CONFIG_UART1_FLOWCONTROL
  if (priv->uartbase == CXD56_UART1_BASE)
    {
      cr |= UART_CR_CTSEN | UART_CR_RTSEN;
    }
#  endif

  /* Enable Auto-RTS and Auto-CS Flow Control in UART2 */

#  ifdef CONFIG_UART2_IFLOWCONTROL
  if (priv->uartbase == CXD56_UART2_BASE)
    {
      cr |= UART_CR_RTSEN;
    }
#  endif
#  ifdef CONFIG_UART2_OFLOWCONTROL
  if (priv->uartbase == CXD56_UART2_BASE)
    {
      cr |= UART_CR_CTSEN;
    }
#  endif
  up_serialout(priv, CXD56_UART_CR, cr);

  /* Enable FIFO and UART in the last */

  lcr |= UART_LCR_FEN;
  up_serialout(priv, CXD56_UART_LCR_H, lcr);
  cr |= UART_CR_EN;
  up_serialout(priv, CXD56_UART_CR, cr);
#endif

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;

  /* Disable further interrupts from the UART */

  up_disableuartint(priv, NULL);

  /* Put the UART hardware back its reset state */

  switch (priv->id)
    {
      case 0:
      case 1:
      case 2:
        cxd56_uart_reset(priv->id);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX and
 *   TX interrupts are not enabled until the txint() and rxint() methods are
 *   called.
 *
 ****************************************************************************/

static int up_attach(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(FAR struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
  (void)nbuffered;

  up_rxint(dev, !upper);

  return true;
}
#endif /* CONFIG_SERIAL_IFLOWCONTROL */

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;
  FAR struct up_dev_s *priv;
  uint32_t status;
  int passes;

  priv = (FAR struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

      status = up_serialin(priv, CXD56_UART_MIS);
      if (status == 0)
        {
          return OK;
        }

      up_serialout(priv, CXD56_UART_ICR, status);
      if (status & UART_INTR_RI)
        {
        }

      if (status & UART_INTR_CTS)
        {
        }

      if (status & UART_INTR_DCD)
        {
        }

      if (status & UART_INTR_DSR)
        {
        }

      if (status & (UART_INTR_RX | UART_INTR_RT))
        {
          uart_recvchars(dev);
        }

      if (status & UART_INTR_TX)
        {
          uart_xmitchars(dev);
        }

      if (status & UART_INTR_FE)
        {
        }

      if (status & UART_INTR_PE)
        {
        }

      if (status & UART_INTR_BE)
        {
        }

      if (status & UART_INTR_OE)
        {
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode    = filep->f_inode;
  FAR struct uart_dev_s *dev = inode->i_private;
  FAR struct up_dev_s *priv  = (FAR struct up_dev_s *)dev->priv;
  int ret                = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
      case TIOCSERGSTRUCT:
        {
          FAR struct up_dev_s *user = (FAR struct up_dev_s *)arg;
          if (!user)
            {
              ret = -EINVAL;
            }
          else
            {
              memcpy(user, dev, sizeof(struct up_dev_s));
            }
        }
        break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          FAR struct termios *termiosp = (FAR struct termios *)arg;

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
          FAR struct termios *termiosp = (FAR struct termios *)arg;

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
          cxd56_setbaud(priv->uartbase, priv->basefreq, priv->baud);
        }
        break;
#endif

      case TIOCSBRK: /* BSD compatibility: Turn break on, unconditionally */
        {
          irqstate_t flags = enter_critical_section();
          up_enablebreaks(priv, true);
          leave_critical_section(flags);
        }
        break;

      case TIOCCBRK: /* BSD compatibility: Turn break off, unconditionally */
        {
          irqstate_t flags;
          flags = enter_critical_section();
          up_enablebreaks(priv, false);
          leave_critical_section(flags);
        }
        break;

      case TCFLSH: /* Flush TX fifo etc. */
        {
          while (!up_txempty(dev));
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
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

static int up_receive(FAR struct uart_dev_s *dev, FAR uint32_t *status)
{
  FAR struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rbr;

  rbr     = up_serialin(priv, CXD56_UART_DR);
  *status = rbr & 0xf00;
  return rbr & 0xff;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= (UART_INTR_RX | UART_INTR_RT);
#endif
    }
  else
    {
      priv->ier &= ~(UART_INTR_RX | UART_INTR_RT);
    }

  up_serialout(priv, CXD56_UART_IMSC, priv->ier);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, CXD56_UART_FR) & UART_FLAG_RXFE) == 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  up_serialout(priv, CXD56_UART_DR, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_INTR_TX;
      up_serialout(priv, CXD56_UART_IMSC, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->ier &= ~UART_INTR_TX;
      up_serialout(priv, CXD56_UART_IMSC, priv->ier);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, CXD56_UART_FR) & UART_FLAG_TXFF) == 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  uint32_t rbr = 0;
  rbr = up_serialin(priv, CXD56_UART_FR);
  return (((rbr & UART_FLAG_TXFE) != 0) && ((rbr & UART_FLAG_BUSY) == 0));
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by up_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#  ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#  endif
}
#endif

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
#ifdef CONSOLE_DEV
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS2_DEV
  (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
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
#ifdef HAVE_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t ier;
  up_disableuartint(priv, &ier);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#ifdef HAVE_CONSOLE
  up_restoreuartint(priv, ier);
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
#ifdef HAVE_UART
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

#endif /* USE_SERIALDRIVER */
