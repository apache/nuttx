/****************************************************************************
 * arch/arm/src/rm57/rm57_serial.c
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

/* Adapted from tms570_serial.c, using RM57's SCI register layout. Only
 * SCI1/LIN1 is currently supported.
 */

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

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_sci.h"
#include "rm57_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/* Which SCI will be tty0/console? */

#if defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI1)
#  define CONSOLE_DEV           g_sci1port  /* SCI1 is console */
#  define TTYS0_DEV             g_sci1port  /* SCI1 is ttyS0 */
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_RM57_SCI1)
#    define TTYS0_DEV           g_sci1port  /* SCI1 is ttyS0 */
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rm57_dev_s
{
  const uint32_t scibase;       /* Base address of SCI registers */
  struct sci_config_s config;   /* SCI configuration */
  uint8_t irq;                  /* IRQ associated with this SCI */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  rm57_setup(struct uart_dev_s *dev);
static void rm57_shutdown(struct uart_dev_s *dev);
static int  rm57_attach(struct uart_dev_s *dev);
static void rm57_detach(struct uart_dev_s *dev);
static int  rm57_interrupt(int irq, void *context, void *arg);
static int  rm57_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  rm57_receive(struct uart_dev_s *dev, unsigned int *status);
static void rm57_rxint(struct uart_dev_s *dev, bool enable);
static bool rm57_rxavailable(struct uart_dev_s *dev);
static void rm57_send(struct uart_dev_s *dev, int ch);
static void rm57_txint(struct uart_dev_s *dev, bool enable);
static bool rm57_txready(struct uart_dev_s *dev);
static bool rm57_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_sci_ops =
{
  .setup          = rm57_setup,
  .shutdown       = rm57_shutdown,
  .attach         = rm57_attach,
  .detach         = rm57_detach,
  .ioctl          = rm57_ioctl,
  .receive        = rm57_receive,
  .rxint          = rm57_rxint,
  .rxavailable    = rm57_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = rm57_send,
  .txint          = rm57_txint,
  .txready        = rm57_txready,
  .txempty        = rm57_txempty,
};

/* I/O buffers */

#ifdef CONFIG_RM57_SCI1
static char g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif

/* This describes the state of the SCI1 port. */

#ifdef CONFIG_RM57_SCI1
static struct rm57_dev_s g_sci1priv =
{
  .scibase        = RM57_SCI1_BASE,
  .config         =
  {
    .baud         = CONFIG_SCI1_BAUD,
    .parity       = 0,
    .bits         = 8,
    .stopbits2    = CONFIG_SCI1_2STOP,
  },
  .irq            = RM57_REQ_LIN1HIGH,
};

static uart_dev_t g_sci1port =
{
  .recv     =
  {
    .size   = CONFIG_SCI1_RXBUFSIZE,
    .buffer = g_sci1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_SCI1_TXBUFSIZE,
    .buffer = g_sci1txbuffer,
  },
  .ops      = &g_sci_ops,
  .priv     = &g_sci1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_serialin
 ****************************************************************************/

static inline uint32_t rm57_serialin(struct rm57_dev_s *priv, int offset)
{
  return getreg32(priv->scibase + offset);
}

/****************************************************************************
 * Name: rm57_serialout
 ****************************************************************************/

static inline void rm57_serialout(struct rm57_dev_s *priv, int offset,
                                   uint32_t value)
{
  putreg32(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: rm57_restoresciint
 ****************************************************************************/

static inline void rm57_restoresciint(struct rm57_dev_s *priv,
                                       uint32_t ints)
{
  rm57_serialout(priv, RM57_SCI_SETINT_OFFSET, ints);
}

/****************************************************************************
 * Name: rm57_disableallints
 ****************************************************************************/

static void rm57_disableallints(struct rm57_dev_s *priv, uint32_t *ints)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = enter_critical_section();
  if (ints)
    {
      *ints = rm57_serialin(priv, RM57_SCI_SETINT_OFFSET);
    }

  rm57_serialout(priv, RM57_SCI_CLEARINT_OFFSET, SCI_INT_ALLINTS);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rm57_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int rm57_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_SCI_CONFIG
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;

  return rm57_sci_configure(priv->scibase, &priv->config);
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: rm57_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void rm57_shutdown(struct uart_dev_s *dev)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;

  rm57_serialout(priv, RM57_SCI_GCR1_OFFSET, 0);
  rm57_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: rm57_attach
 *
 * Description:
 *   Configure the SCI to operate in interrupt driven mode.  This method
 *   is called when the serial port is opened, normally just after the
 *   setup() method is called.
 *
 ****************************************************************************/

static int rm57_attach(struct uart_dev_s *dev)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->irq, rm57_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: rm57_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port
 *   is closed normally, just before the shutdown method is called.
 *
 ****************************************************************************/

static void rm57_detach(struct uart_dev_s *dev)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: rm57_interrupt
 *
 * Description:
 *   This is the common SCI interrupt handler.
 *
 ****************************************************************************/

static int rm57_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct rm57_dev_s *priv;
  uint32_t intvec;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct rm57_dev_s *)dev->priv;

  for (; ; )
    {
      /* Reading INTVECT0 clears the corresponding INTFLAG bit for most
       * interrupt sources.
       */

      intvec = rm57_serialin(priv, RM57_SCI_INTVECT0_OFFSET) &
               SCI_INTVECT_MASK;

      switch (intvec)
        {
          case SCI_INTVECT_NONE:    /* No interrupt */
            return OK;

          case SCI_INTVECT_WAKEUP:  /* Wake-up interrupt (ignored) */
            break;

          /* SCI errors: ignored for now, since break-detect interrupt
           * is never enabled
           */

          case SCI_INTVECT_PE:
          case SCI_INTVECT_FE:
          case SCI_INTVECT_OE:
          case SCI_INTVECT_BRKDT:
          case SCI_INTVECT_BE:
            break;

          case SCI_INTVECT_RX:      /* Receive interrupt */
            uart_recvchars(dev);
            break;

          case SCI_INTVECT_TX:      /* Transmit interrupt */
            uart_xmitchars(dev);
            break;

          /* LIN mode only.  These should never occur in SCI mode */

          case SCI_INTVECT_ISFE:
          case SCI_INTVECT_ID:
          case SCI_INTVECT_PBE:
          case SCI_INTVECT_CE:
          case SCI_INTVECT_NRE:
          case SCI_INTVECT_TOAWUS:
          case SCI_INTVECT_TOA3WUS:
          case SCI_INTVECT_TIMEOUT:
          default:
            DEBUGPANIC();
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rm57_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int rm57_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct rm57_dev_s *user = (struct rm57_dev_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev->priv, sizeof(struct rm57_dev_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        termiosp->c_cflag = ((priv->config.parity != 0) ? PARENB : 0) |
                            ((priv->config.parity == 1) ? PARODD : 0);
        termiosp->c_cflag |= (priv->config.stopbits2) ? CSTOPB : 0;
        cfsetispeed(termiosp, priv->config.baud);

        switch (priv->config.bits)
          {
          case 5:
            termiosp->c_cflag |= CS5;
            break;

          case 6:
            termiosp->c_cflag |= CS6;
            break;

          case 7:
            termiosp->c_cflag |= CS7;
            break;

          default:
          case 8:
            termiosp->c_cflag |= CS8;
            break;
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;
        uint32_t baud;
        uint32_t ints;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        ret = OK;
        baud = cfgetispeed(termiosp);

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            nbits = 5;
            break;

          case CS6:
            nbits = 6;
            break;

          case CS7:
            nbits = 7;
            break;

          case CS8:
            nbits = 8;
            break;

          default:
            ret = -EINVAL;
            break;
          }

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        stop2 = (termiosp->c_cflag & CSTOPB) != 0;

        if (ret == OK)
          {
            priv->config.baud      = baud;
            priv->config.parity    = parity;
            priv->config.bits      = nbits;
            priv->config.stopbits2 = stop2;

            rm57_disableallints(priv, &ints);
            ret = rm57_sci_configure(priv->scibase, &priv->config);
            rm57_restoresciint(priv, ints);
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: rm57_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character
 *   from the SCI.  Error bits associated with the receipt are provided
 *   in the return 'status'.
 *
 ****************************************************************************/

static int rm57_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;

  *status = rm57_serialin(priv, RM57_SCI_FLR_OFFSET);
  return (int)(rm57_serialin(priv, RM57_SCI_RD_OFFSET) & 0xff);
}

/****************************************************************************
 * Name: rm57_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void rm57_rxint(struct uart_dev_s *dev, bool enable)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      rm57_serialout(priv, RM57_SCI_SETINT_OFFSET, SCI_INT_RX);
#endif
    }
  else
    {
      rm57_serialout(priv, RM57_SCI_CLEARINT_OFFSET, SCI_INT_RX);
    }
}

/****************************************************************************
 * Name: rm57_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool rm57_rxavailable(struct uart_dev_s *dev)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;
  return ((rm57_serialin(priv, RM57_SCI_FLR_OFFSET) & SCI_FLR_RXRDY) != 0);
}

/****************************************************************************
 * Name: rm57_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/

static void rm57_send(struct uart_dev_s *dev, int ch)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;
  rm57_serialout(priv, RM57_SCI_TD_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: rm57_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void rm57_txint(struct uart_dev_s *dev, bool enable)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      rm57_serialout(priv, RM57_SCI_SETINT_OFFSET, SCI_INT_TX);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      rm57_serialout(priv, RM57_SCI_CLEARINT_OFFSET, SCI_INT_TX);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rm57_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool rm57_txready(struct uart_dev_s *dev)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;
  return ((rm57_serialin(priv, RM57_SCI_FLR_OFFSET) & SCI_FLR_TXRDY) != 0);
}

/****************************************************************************
 * Name: rm57_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool rm57_txempty(struct uart_dev_s *dev)
{
  struct rm57_dev_s *priv = (struct rm57_dev_s *)dev->priv;
  return ((rm57_serialin(priv, RM57_SCI_FLR_OFFSET) &
          SCI_FLR_TXEMPTY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  /* Disable all SCIs */

  rm57_disableallints(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  rm57_disableallints(TTYS1_DEV.priv, NULL);
#endif

#ifdef CONSOLE_DEV
  /* Configure whichever one is the console.  NOTE: this was already done
   * in rm57_lowsetup().
   */

  CONSOLE_DEV.isconsole = true;
  rm57_setup(&CONSOLE_DEV);

  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

#endif /* USE_SERIALDRIVER */
