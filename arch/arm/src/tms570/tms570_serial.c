/****************************************************************************
 * arch/arm/src/tms570/tms570_serial.c
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
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/tms570_sci.h"
#include "tms570_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/* Which SCI with be tty0/console and which tty1? */

/* First pick the console and ttys0.  This could be any of SCI1-1 */

#if defined(CONFIG_SCI1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_sci1port  /* SCI1 is console */
#    define TTYS0_DEV           g_sci1port  /* SCI1 is ttyS0 */
#    define SCI1_ASSIGNED       1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_sci2port  /* SCI2 is console */
#    define TTYS0_DEV           g_sci2port  /* SCI2 is ttyS0 */
#    define SCI2_ASSIGNED       1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_TMS570_SCI1)
#    define TTYS0_DEV           g_sci1port  /* SCI1 is ttyS0 */
#    define SCI1_ASSIGNED       1
#  elif defined(CONFIG_TMS570_SCI2)
#    define TTYS0_DEV           g_sci2port  /* SCI2 is ttyS0 */
#    define SCI2_ASSIGNED       1
#  endif
#endif

/* Pick ttys1.  This could be any of SCI1-1, excluding the console
 * SCI.
 */

#if defined(CONFIG_TMS570_SCI1) && !defined(SCI1_ASSIGNED)
#  define TTYS1_DEV             g_sci1port  /* SCI1 is ttyS1 */
#  define SCI1_ASSIGNED         1
#elif defined(CONFIG_TMS570_SCI2) && !defined(SCI2_ASSIGNED)
#  define TTYS1_DEV             g_sci2port  /* SCI2 is ttyS1 */
#  define SCI2_ASSIGNED         1
#endif

/* BAUD definitions
 *
 * The source clock is selectable and could be one of:
 *
 *   - The peripheral clock
 *   - A division of the peripheral clock, where the divider is product-
 *     dependent, but generally set to 8
 *   - A processor/peripheral independent clock source fully programmable
 *      provided by PMC (PCK)
 *   - The external clock, available on the SCK pin
 *
 * Only the first two options are supported by this driver.  The divided
 * peripheral clock is only used for very low BAUD selections.
 */

#define FAST_SCI_CLOCK         BOARD_MCK_FREQUENCY
#define SLOW_SCI_CLOCK         (BOARD_MCK_FREQUENCY >> 3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tms570_dev_s
{
  const uint32_t scibase;       /* Base address of SCI registers */
  struct sci_config_s config;   /* SCI configuration */
  uint8_t irq;                  /* IRQ associated with this SCI */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tms570_setup(struct uart_dev_s *dev);
static void tms570_shutdown(struct uart_dev_s *dev);
static int  tms570_attach(struct uart_dev_s *dev);
static void tms570_detach(struct uart_dev_s *dev);
static int  tms570_interrupt(int irq, void *context, void *arg);
static int  tms570_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  tms570_receive(struct uart_dev_s *dev, unsigned int *status);
static void tms570_rxint(struct uart_dev_s *dev, bool enable);
static bool tms570_rxavailable(struct uart_dev_s *dev);
static void tms570_send(struct uart_dev_s *dev, int ch);
static void tms570_txint(struct uart_dev_s *dev, bool enable);
static bool tms570_txready(struct uart_dev_s *dev);
static bool tms570_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_sci_ops =
{
  .setup          = tms570_setup,
  .shutdown       = tms570_shutdown,
  .attach         = tms570_attach,
  .detach         = tms570_detach,
  .ioctl          = tms570_ioctl,
  .receive        = tms570_receive,
  .rxint          = tms570_rxint,
  .rxavailable    = tms570_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = tms570_send,
  .txint          = tms570_txint,
  .txready        = tms570_txready,
  .txempty        = tms570_txempty,
};

/* I/O buffers */

#ifdef CONFIG_TMS570_SCI1
static char g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif
#ifdef CONFIG_TMS570_SCI2
static char g_sci2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_sci2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#endif

/* This describes the state of the SCI1 port. */

#ifdef CONFIG_TMS570_SCI1
static struct tms570_dev_s g_sci1priv =
{
  .scibase        = TMS570_SCI1_BASE,
  .config         =
  {
    .baud         = CONFIG_SCI1_BAUD,
    .parity       = CONFIG_SCI1_PARITY,
    .bits         = CONFIG_SCI1_BITS,
    .stopbits2    = CONFIG_SCI1_2STOP,
  },
  .irq            = TMS570_REQ_SCI1_0,
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

/* This describes the state of the SCI2 port. */

#ifdef CONFIG_TMS570_SCI2
static struct tms570_dev_s g_sci2priv =
{
  .scibase        = TMS570_SCI2_BASE,
  .config         =
  {
    .baud         = CONFIG_SCI2_BAUD,
    .parity       = CONFIG_SCI2_PARITY,
    .bits         = CONFIG_SCI2_BITS,
    .stopbits2    = CONFIG_SCI2_2STOP,
  },
  .irq            = TMS570_REQ_SCI2_0,
};

static uart_dev_t g_sci2port =
{
  .recv     =
  {
    .size   = CONFIG_SCI2_RXBUFSIZE,
    .buffer = g_sci2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_SCI2_TXBUFSIZE,
    .buffer = g_sci2txbuffer,
  },
  .ops      = &g_sci_ops,
  .priv     = &g_sci2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_serialin
 ****************************************************************************/

static inline uint32_t tms570_serialin(struct tms570_dev_s *priv, int offset)
{
  return getreg32(priv->scibase + offset);
}

/****************************************************************************
 * Name: tms570_serialout
 ****************************************************************************/

static inline void tms570_serialout(struct tms570_dev_s *priv, int offset,
                                    uint32_t value)
{
  putreg32(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: tms570_restoresciint
 ****************************************************************************/

static inline void tms570_restoresciint(struct tms570_dev_s *priv,
                                        uint32_t ints)
{
  /* Restore the previous interrupt state (assuming all interrupts
   * disabled)
   */

  tms570_serialout(priv, TMS570_SCI_SETINT_OFFSET, ints);
}

/****************************************************************************
 * Name: tms570_disableallints
 ****************************************************************************/

static void tms570_disableallints(struct tms570_dev_s *priv, uint32_t *ints)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = enter_critical_section();
  if (ints)
    {
      /* Return the current enable bitsopop9 */

      *ints = tms570_serialin(priv, TMS570_SCI_SETINT_OFFSET);
    }

  /* Disable all interrupts */

  tms570_serialout(priv, TMS570_SCI_CLEARINT_OFFSET, SCI_INT_ALLINTS);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tms570_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int tms570_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_SCI_CONFIG
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;

  /* Configure baud, number of bits, stop bits, and parity */

  return tms570_sci_configure(priv->scibase, &priv->config);
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: tms570_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void tms570_shutdown(struct uart_dev_s *dev)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;

  /* Reset and disable receiver and transmitter */

  tms570_serialout(priv, TMS570_SCI_GCR1_OFFSET, 0);

  /* Disable all interrupts */

  tms570_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: tms570_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int tms570_attach(struct uart_dev_s *dev)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, tms570_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the SCI
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: tms570_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void tms570_detach(struct uart_dev_s *dev)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: tms570_interrupt
 *
 * Description:
 *   This is the common SCI interrupt handler.  It will be invoked
 *   when an interrupt received on the device.  It should call
 *   sci_transmitchars or sci_receivechar to perform the appropriate data
 *   transfers.
 *
 ****************************************************************************/

static int tms570_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct tms570_dev_s *priv;
  uint32_t intvec;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct tms570_dev_s *)dev->priv;

  /* Loop until there are no further pending interrupts */

  for (; ; )
    {
      /* Get the next pending interrupt.  For most interrupts, reading the
       * INVECT0 register clears the corresponding INTFLAG.
       */

      intvec = tms570_serialin(priv, TMS570_SCI_INTVECT0_OFFSET) & \
               SCI_INTVECT_MASK;

      /* Handle the pending interrupt */

      switch (intvec)
        {
          case SCI_INTVECT_NONE:    /* No interrupt */
            return OK;

          case SCI_INTVECT_WAKEUP:  /* Wake-up interrupt */

            /* SCI sets the WAKEUP flag if bus activity on the RX line
             * either prevents power-down mode from being entered, or RX
             * line activity causes an exit from power-down mode. If
             * enabled wakeup interrupt is triggered once WAKEUP flag is
             * set.
             *
             * REVISIT: This interrupt is ignored because for now the
             * break detect interrupt is never enabled.
             */

            break;

          /* SCI Errors
           *
           * REVISIT: These error interrupta are ignored because for now the
           * break detect interrupt is never enabled.
           */

          case SCI_INTVECT_PE:      /* Parity error interrupt */
          case SCI_INTVECT_FE:      /* Framing error interrupt */
          case SCI_INTVECT_OE:      /* Overrun error interrupt */
          case SCI_INTVECT_BRKDT:   /* Break detect interrupt */
          case SCI_INTVECT_BE:      /* Bit error interrupt */
            break;

          case SCI_INTVECT_RX:      /* Receive interrupt */
            {
              /* Receive data ready... process incoming bytes */

              uart_recvchars(dev);
            }
            break;

          case SCI_INTVECT_TX:      /* Tranmit interrupt */
            {
              /* Transmit data register available ...
               * process outgoing bytes
               */

              uart_xmitchars(dev);
            }
            break;

          /* LIN mode only.  These should never occur in SCI mode */

          case SCI_INTVECT_ISFE:    /* Inconsistent synch field error interrupt */
          case SCI_INTVECT_ID:      /* Identification interrupt */
          case SCI_INTVECT_PBE:     /* Physical bus error interrupt */
          case SCI_INTVECT_CE:      /* Checksum error interrupt */
          case SCI_INTVECT_NRE:     /* No response error interrupt */
          case SCI_INTVECT_TOAWUS:  /* Timeout after wakeup signal interrupt */
          case SCI_INTVECT_TOA3WUS: /* Timeout after 2 Wakeup signls interrupt */
          case SCI_INTVECT_TIMEOUT: /* Timeout interrupt */

          default:
            DEBUGPANIC();
        }
    }

  return OK;
}

/****************************************************************************
 * Name: tms570_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int tms570_ioctl(struct file *filep, int cmd, unsigned long arg)
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
         struct tms570_dev_s *user = (struct tms570_dev_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct tms570_dev_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return parity */

        termiosp->c_cflag = ((priv->config.parity != 0) ? PARENB : 0) |
                            ((priv->config.parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= (priv->config.stopbits2) ? CSTOPB : 0;

        /* Return baud */

        cfsetispeed(termiosp, priv->config.baud);

        /* Return number of bits */

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

          case 9:
            termiosp->c_cflag |= CS8 /* CS9 */;
            break;
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;
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

        /* Decode baud. */

        ret = OK;
        baud = cfgetispeed(termiosp);

        /* Decode number of bits */

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
#if 0
          case CS9:
            nbits = 9;
            break;
#endif
          default:
            ret = -EINVAL;
            break;
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

            priv->config.baud      = baud;
            priv->config.parity    = parity;
            priv->config.bits      = nbits;
            priv->config.stopbits2 = stop2;

            /* effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            tms570_disableallints(priv, &ints);
            ret = tms570_sci_configure(priv->scibase, &priv->config);

            /* Restore the interrupt state */

            tms570_restoresciint(priv, ints);
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
 * Name: tms570_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int tms570_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;

  /* Return the error information in the saved status. */

  *status = tms570_serialin(priv, TMS570_SCI_FLR_OFFSET);

  /* Then return the actual received byte */

  return (int)(tms570_serialin(priv, TMS570_SCI_RD_OFFSET) & 0xff);
}

/****************************************************************************
 * Name: tms570_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void tms570_rxint(struct uart_dev_s *dev, bool enable)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an RX timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      tms570_serialout(priv, TMS570_SCI_SETINT_OFFSET, SCI_INT_RX);
#endif
    }
  else
    {
      tms570_serialout(priv, TMS570_SCI_CLEARINT_OFFSET, SCI_INT_RX);
    }
}

/****************************************************************************
 * Name: tms570_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool tms570_rxavailable(struct uart_dev_s *dev)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;
  return ((tms570_serialin(priv, TMS570_SCI_FLR_OFFSET) & \
          SCI_FLR_RXRDY) != 0);
}

/****************************************************************************
 * Name: tms570_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *-
 ****************************************************************************/

static void tms570_send(struct uart_dev_s *dev, int ch)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;
  tms570_serialout(priv, TMS570_SCI_TD_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: tms570_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void tms570_txint(struct uart_dev_s *dev, bool enable)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      tms570_serialout(priv, TMS570_SCI_SETINT_OFFSET, SCI_INT_TX);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      tms570_serialout(priv, TMS570_SCI_CLEARINT_OFFSET, SCI_INT_TX);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tms570_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool tms570_txready(struct uart_dev_s *dev)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;
  return ((tms570_serialin(priv, TMS570_SCI_FLR_OFFSET) & \
          SCI_FLR_TXRDY) != 0);
}

/****************************************************************************
 * Name: tms570_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool tms570_txempty(struct uart_dev_s *dev)
{
  struct tms570_dev_s *priv = (struct tms570_dev_s *)dev->priv;
  return ((tms570_serialin(priv, TMS570_SCI_FLR_OFFSET) & \
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
  /* Disable all SCIS */

  tms570_disableallints(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  tms570_disableallints(TTYS1_DEV.priv, NULL);
#endif

#ifdef CONSOLE_DEV
  /* Configure whichever one is the console.  NOTE: This was already done
   * in tms570_lowsetup().
   */

  CONSOLE_DEV.isconsole = true;
  tms570_setup(&CONSOLE_DEV);

  /* Register the console */

  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all SCIs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

#endif /* USE_SERIALDRIVER */
