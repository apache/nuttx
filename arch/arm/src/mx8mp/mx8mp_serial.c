/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_serial.c
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
#include <nuttx/serial/serial.h>

#include "chip.h"
#include "mx8mp_lowputc.h"
#include "mx8mp_ccm.h"
#include "mx8mp_serial.h"
#include "hardware/mx8mp_uart.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which UART with be tty0/console and which tty1-3?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART1-4 */

#if   defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port /* UART1 is console */
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart2port /* UART2 is console */
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart3port /* UART3 is console */
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart4port /* UART4 is console */
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_MX8MP_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_MX8MP_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(CONFIG_MX8MP_UART3)
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  elif defined(CONFIG_MX8MP_UART4)
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART1-4 excluding the console UART.
 * One of UART1-4 could be the console; one of UART1-4 has already been
 * assigned to ttys0.
 */

#if defined(CONFIG_MX8MP_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_MX8MP_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_MX8MP_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_MX8MP_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART2-4. It can't be UART1 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-4 could be the
 * console.  One of UART2-4 has already been assigned to ttys0 or ttyS1.
 */

#if defined(CONFIG_MX8MP_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_MX8MP_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_MX8MP_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART3-4. It can't be UART1-2 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART3-4 could also be the console.  One of UART3-4 has already
 * been assigned to ttys0, 1, or 3.
 */

#if defined(CONFIG_MX8MP_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_MX8MP_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#endif

/* UART, if available, should have been assigned to ttyS0-4. */

#if defined(CONFIG_MX8MP_UART4) && !defined(UART4_ASSIGNED)
#  errnor UART4 was not assigned to a TTY.
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mx8mp_uart_s
{
  int      clock;       /* Peripheral clock as described in hardware/mx8mp_ccm.h */
  uint32_t uartbase;    /* Base address of UART registers */
  uint32_t baud;        /* Configured baud */
  uint32_t ie;          /* Saved enabled interrupts */
  uint32_t ucr1;        /* Saved UCR1 value */
  uint8_t  irq;         /* IRQ associated with this UART */
  uint8_t  parity;      /* 0=none, 1=odd, 2=even */
  uint8_t  bits;        /* Number of bits (7 or 8) */
  uint8_t  stopbits2:1; /* 1: Configure with 2 stop bits vs 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t  iflow:1;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t  oflow:1;     /* output flow control (CTS) enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t mx8mp_serialin(struct mx8mp_uart_s *priv,
                                      uint32_t offset);
static inline void mx8mp_serialout(struct mx8mp_uart_s *priv,
                                   uint32_t offset,
                                   uint32_t value);
static inline void mx8mp_disableuartint(struct mx8mp_uart_s *priv,
                                        uint32_t *ucr1);
static inline void mx8mp_restoreuartint(struct mx8mp_uart_s *priv,
                                        uint32_t ucr1);
static inline void mx8mp_waittxready(struct mx8mp_uart_s *priv);

static int  mx8mp_setup(struct uart_dev_s *dev);
static void mx8mp_shutdown(struct uart_dev_s *dev);
static int  mx8mp_attach(struct uart_dev_s *dev);
static void mx8mp_detach(struct uart_dev_s *dev);
static int  mx8mp_interrupt(int irq, void *context, void *arg);
static int  mx8mp_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  mx8mp_receive(struct uart_dev_s *dev, unsigned int *status);
static void mx8mp_rxint(struct uart_dev_s *dev, bool enable);
static bool mx8mp_rxavailable(struct uart_dev_s *dev);
static void mx8mp_send(struct uart_dev_s *dev, int ch);
static void mx8mp_txint(struct uart_dev_s *dev, bool enable);
static bool mx8mp_txready(struct uart_dev_s *dev);
static bool mx8mp_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup          = mx8mp_setup,
  .shutdown       = mx8mp_shutdown,
  .attach         = mx8mp_attach,
  .detach         = mx8mp_detach,
  .ioctl          = mx8mp_ioctl,
  .receive        = mx8mp_receive,
  .rxint          = mx8mp_rxint,
  .rxavailable    = mx8mp_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = mx8mp_send,
  .txint          = mx8mp_txint,
  .txready        = mx8mp_txready,
  .txempty        = mx8mp_txempty,
};

/* I/O buffers */

#ifdef CONFIG_MX8MP_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_MX8MP_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_MX8MP_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

#ifdef CONFIG_MX8MP_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

#ifdef CONFIG_MX8MP_UART1
static struct mx8mp_uart_s g_uart1priv =
{
  .clock          = UART1_CLK_ROOT,
  .uartbase       = MX8M_UART1,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = MX8MP_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
};

static struct uart_dev_s g_uart1port =
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

#ifdef CONFIG_MX8MP_UART2
static struct mx8mp_uart_s g_uart2priv =
{
  .clock          = UART2_CLK_ROOT,
  .uartbase       = MX8M_UART2,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = MX8MP_IRQ_UART2,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
  .stopbits2      = CONFIG_UART2_2STOP,
};

static struct uart_dev_s g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

#ifdef CONFIG_MX8MP_UART3
static struct mx8mp_uart_s g_uart3priv =
{
  .clock          = UART3_CLK_ROOT,
  .uartbase       = MX8M_UART3,
  .baud           = CONFIG_UART3_BAUD,
  .irq            = MX8MP_IRQ_UART3,
  .parity         = CONFIG_UART3_PARITY,
  .bits           = CONFIG_UART3_BITS,
  .stopbits2      = CONFIG_UART3_2STOP,
};

static struct uart_dev_s g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_UART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};
#endif

#ifdef CONFIG_MX8MP_UART4
static struct mx8mp_uart_s g_uart4priv =
{
  .clock          = UART4_CLK_ROOT,
  .uartbase       = MX8M_UART4,
  .baud           = CONFIG_UART4_BAUD,
  .irq            = MX8MP_IRQ_UART4,
  .parity         = CONFIG_UART4_PARITY,
  .bits           = CONFIG_UART4_BITS,
  .stopbits2      = CONFIG_UART4_2STOP,
};

static struct uart_dev_s g_uart4port =
{
  .recv     =
  {
    .size   = CONFIG_UART4_RXBUFSIZE,
    .buffer = g_uart4rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART4_TXBUFSIZE,
    .buffer = g_uart4txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart4priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_serialin
 ****************************************************************************/

static inline uint32_t mx8mp_serialin(struct mx8mp_uart_s *priv,
                                      uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: mx8mp_serialout
 ****************************************************************************/

static inline void mx8mp_serialout(struct mx8mp_uart_s *priv,
                                   uint32_t offset,
                                   uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: mx8mp_disableuartint
 ****************************************************************************/

static inline void mx8mp_disableuartint(struct mx8mp_uart_s *priv,
                                        uint32_t *ucr1)
{
  /* Return the current Rx and Tx interrupt state */

  if (ucr1 != NULL)
    {
      *ucr1 = priv->ucr1 & (UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
    }

  /* Then disable both Rx and Tx interrupts */

  priv->ucr1 &= ~(UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  mx8mp_serialout(priv, UART_UCR1_OFFSET, priv->ucr1);
}

/****************************************************************************
 * Name: mx8mp_restoreuartint
 ****************************************************************************/

static inline void mx8mp_restoreuartint(struct mx8mp_uart_s *priv,
                                        uint32_t ucr1)
{
  /* Enable/disable any interrupts that are currently disabled but should be
   * enabled/disabled.
   */

  priv->ucr1 &= ~(UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  priv->ucr1 |= ucr1 & (UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  mx8mp_serialout(priv, UART_UCR1_OFFSET, priv->ucr1);
}

/****************************************************************************
 * Name: mx8mp_waittxready
 ****************************************************************************/

static inline void mx8mp_waittxready(struct mx8mp_uart_s *priv)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if ((mx8mp_serialin(priv, UART_UTS_OFFSET) & UART_UTS_TXFULL) == 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: mx8mp_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int mx8mp_setup(struct uart_dev_s *dev)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct uart_config_s config;
  int ret;

  /* Configure the UART */

  config.baud       = priv->baud;          /* Configured baud */
  config.parity     = priv->parity;        /* 0=none, 1=odd, 2=even */
  config.bits       = priv->bits;          /* Number of bits (5-9) */
  config.stopbits2  = priv->stopbits2;     /* true: Configure with 2 stop bits instead of 1 */

  uint32_t clk = mx8mp_ccm_get_clock(priv->clock);
  ret = mx8mp_uart_configure(priv->uartbase, clk, &config);

  /* Initialize the UCR1 shadow register */

  priv->ucr1 = mx8mp_serialin(priv, UART_UCR1_OFFSET);
  return ret;

#else
  /* Initialize the UCR1 shadow register */

  priv->ucr1 = mx8mp_serialin(priv, UART_UCR1_OFFSET);
  return OK;
#endif
}

/****************************************************************************
 * Name: mx8mp_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void mx8mp_shutdown(struct uart_dev_s *dev)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;

  /* Disable the UART */

  mx8mp_serialout(priv, UART_UCR1_OFFSET, 0);
  mx8mp_serialout(priv, UART_UCR2_OFFSET, 0);
  mx8mp_serialout(priv, UART_UCR3_OFFSET, 0);
  mx8mp_serialout(priv, UART_UCR4_OFFSET, 0);
}

/****************************************************************************
 * Name: mx8mp_attach
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

static int mx8mp_attach(struct uart_dev_s *dev)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, mx8mp_interrupt, dev);
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
 * Name: mx8mp_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void mx8mp_detach(struct uart_dev_s *dev)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: mx8mp_interrupt (and front-ends)
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int mx8mp_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct mx8mp_uart_s *priv;
  uint32_t usr1;
  int passes = 0;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct mx8mp_uart_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (; ; )
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

      usr1 = mx8mp_serialin(priv, UART_USR1_OFFSET);
      usr1 &= (UART_USR1_RRDY | UART_USR1_TRDY);

      if (usr1 == 0 || passes > 256)
        {
          return OK;
        }

      /* Handle incoming, receive bytes */

      if (usr1 & UART_USR1_RRDY)
        {
          uart_recvchars(dev);
        }

      /* Handle outgoing, transmit bytes */

      if (usr1 & UART_USR1_TRDY &&
         (mx8mp_serialin(priv, UART_UCR1_OFFSET) & UART_UCR1_TXEMPTYEN) != 0)
        {
          uart_xmitchars(dev);
        }

      /* Keep track of how many times we do this in case there
       * is some hardware failure condition.
       */

      passes++;
    }
}

/****************************************************************************
 * Name: mx8mp_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int mx8mp_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || defined(CONFIG_SERIAL_TERMIOS)
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  irqstate_t flags;
#endif
  int ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct mx8mp_uart_s *user = (struct mx8mp_uart_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct mx8mp_uart_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;

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

#if defined(CS9)
          case 9:
            termiosp->c_cflag |= CS9;
            break;
#endif
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;
        uint32_t baud;
        uint32_t ie;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;

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

#if defined(CS9)
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

            priv->baud      = baud;
            priv->parity    = parity;
            priv->bits      = nbits;
            priv->stopbits2 = stop2;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow     = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow     = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
            /* effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            flags  = spin_lock_irqsave(NULL);
            mx8mp_disableuartint(priv, &ie);
            ret = mx8mp_setup(dev);

            /* Restore the interrupt state */

            mx8mp_restoreuartint(priv, ie);
            priv->ie = ie;
            spin_unlock_irqrestore(NULL, flags);
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: mx8mp_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int mx8mp_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;
  uint32_t rxd0;

  rxd0    = mx8mp_serialin(priv, UART_RXD_OFFSET);
  *status = rxd0;
  return (rxd0 & UART_RXD_DATA_MASK) >> UART_RXD_DATA_SHIFT;
}

/****************************************************************************
 * Name: mx8mp_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void mx8mp_rxint(struct uart_dev_s *dev, bool enable)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;

  /* Enable interrupts for data availab at Rx FIFO */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ucr1 |= UART_UCR1_RRDYEN;
#endif
    }
  else
    {
      priv->ucr1 &= ~UART_UCR1_RRDYEN;
    }

  mx8mp_serialout(priv, UART_UCR1_OFFSET, priv->ucr1);
}

/****************************************************************************
 * Name: mx8mp_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool mx8mp_rxavailable(struct uart_dev_s *dev)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;

  /* Return true is data is ready in the Rx FIFO */

  return ((mx8mp_serialin(priv, UART_USR2_OFFSET) & UART_USR2_RDR) != 0);
}

/****************************************************************************
 * Name: mx8mp_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void mx8mp_send(struct uart_dev_s *dev, int ch)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;
  mx8mp_serialout(priv, UART_TXD_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: mx8mp_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void mx8mp_txint(struct uart_dev_s *dev, bool enable)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;

  /* We won't take an interrupt until the FIFO is completely empty (although
   * there may still be a transmission in progress).
   */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ucr1 |= UART_UCR1_TXEMPTYEN;
#endif
    }
  else
    {
      priv->ucr1 &= ~UART_UCR1_TXEMPTYEN;
    }

  mx8mp_serialout(priv, UART_UCR1_OFFSET, priv->ucr1);
}

/****************************************************************************
 * Name: mx8mp_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool mx8mp_txready(struct uart_dev_s *dev)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;

  /* When TXFULL is set, there is no space in the Tx FIFO  */

  return ((mx8mp_serialin(priv, UART_UTS_OFFSET) & UART_UTS_TXFULL) == 0);
}

/****************************************************************************
 * Name: mx8mp_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool mx8mp_txempty(struct uart_dev_s *dev)
{
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)dev->priv;

  /* When TXDC is set, the FIFO is empty and the transmission is complete */

  return ((mx8mp_serialin(priv, UART_USR2_OFFSET) & UART_USR2_TXDC) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm_serialinit.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function mx8mp_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */
#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  mx8mp_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that mx8mp_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef HAVE_UART_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#  ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#    ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#      ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS2_DEV);
#        ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS2_DEV);
#        endif
#      endif
#    endif
#  endif
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
#ifdef HAVE_UART_CONSOLE
  struct mx8mp_uart_s *priv = (struct mx8mp_uart_s *)CONSOLE_DEV.priv;
  uint32_t ier;

  /* Disable UART interrupts and wait until the hardware is ready to send
   * a byte.
   */

  mx8mp_disableuartint(priv, &ier);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  mx8mp_restoreuartint(priv, ier);
#endif

  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#  undef MX8MP_CONSOLE_VBASE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define MX8MP_CONSOLE MX8M_UART1
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define MX8MP_CONSOLE MX8M_UART2
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define MX8MP_CONSOLE MX8M_UART3
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define MX8MP_CONSOLE MX8M_UART4
#  endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef MX8MP_CONSOLE
static inline void mx8mp_waittxready(void)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Loop until TXFULL is zero -- meaning that there is space available
       * in the TX FIFO.
       */

      if ((getreg32(MX8MP_CONSOLE + UART_UTS_OFFSET) & UART_UTS_TXFULL) == 0)
        {
          break;
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef MX8MP_CONSOLE
  mx8mp_waittxready();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      putreg32((uint16_t)'\r', MX8MP_CONSOLE + UART_TXD_OFFSET);
      mx8mp_waittxready();
    }

  putreg32((uint16_t)ch, MX8MP_CONSOLE + UART_TXD_OFFSET);
#endif

  return ch;
}

#endif /* USE_SERIALDRIVER */
