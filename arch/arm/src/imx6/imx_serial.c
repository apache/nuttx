/****************************************************************************
 * arch/arm/src/imx6/imx_serial.c
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
#include "arm_arch.h"
#include "arm_internal.h"

#include "gic.h"
#include "hardware/imx_uart.h"
#include "imx_config.h"
#include "imx_lowputc.h"

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
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart5port /* UART5 is console */
#    define TTYS5_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_IMX6_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_IMX6_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(CONFIG_IMX6_UART3)
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  elif defined(CONFIG_IMX6_UART4)
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  elif defined(CONFIG_IMX6_UART5)
#    define TTYS0_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART1-5 excluding the console UART.
 * One of UART1-5 could be the console; one of UART1-5 has already been
 * assigned to ttys0.
 */

#if defined(CONFIG_IMX6_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS1_DEV           g_uart5port /* UART5 is ttyS1 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART2-5. It can't be UART1 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-5 could be the
 * console.  One of UART2-5 has already been assigned to ttys0 or ttyS1.
 */

#if defined(CONFIG_IMX6_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS2_DEV           g_uart5port /* UART5 is ttyS2 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART3-5. It can't be UART1-2 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART32-5 could also be the console.  One of UART3-5 has already
 * been assigned to ttys0, 1, or 3.
 */

#if defined(CONFIG_IMX6_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS3_DEV           g_uart5port /* UART5 is ttyS3 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys4. This could be one of UART4-5. It can't be UART1-3 because
 * those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * UART 4-5 could be the console.  One of UART4-5 has already been
 * assigned to ttys0, 1, 3, or 4.
 */

#if defined(CONFIG_IMX6_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_IMX6_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS4_DEV           g_uart5port /* UART5 is ttyS4 */
#  define UART5_ASSIGNED      1
#endif

/* UART, if available, should have been assigned to ttyS0-4. */

#if defined(CONFIG_IMX6_UART5) && !defined(UART5_ASSIGNED)
#  errnor UART5 was not assigned to a TTY.
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx_uart_s
{
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

static inline uint32_t imx_serialin(struct imx_uart_s *priv,
              uint32_t offset);
static inline void imx_serialout(struct imx_uart_s *priv, uint32_t offset,
              uint32_t value);
static inline void imx_disableuartint(struct imx_uart_s *priv,
              uint32_t *ucr1);
static inline void imx_restoreuartint(struct imx_uart_s *priv,
              uint32_t ucr1);
static inline void imx_waittxready(struct imx_uart_s *priv);

static int  imx_setup(struct uart_dev_s *dev);
static void imx_shutdown(struct uart_dev_s *dev);
static int  imx_attach(struct uart_dev_s *dev);
static void imx_detach(struct uart_dev_s *dev);
static int  imx_interrupt(int irq, void *context, FAR void *arg);
static int  imx_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  imx_receive(struct uart_dev_s *dev, unsigned int *status);
static void imx_rxint(struct uart_dev_s *dev, bool enable);
static bool imx_rxavailable(struct uart_dev_s *dev);
static void imx_send(struct uart_dev_s *dev, int ch);
static void imx_txint(struct uart_dev_s *dev, bool enable);
static bool imx_txready(struct uart_dev_s *dev);
static bool imx_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup          = imx_setup,
  .shutdown       = imx_shutdown,
  .attach         = imx_attach,
  .detach         = imx_detach,
  .ioctl          = imx_ioctl,
  .receive        = imx_receive,
  .rxint          = imx_rxint,
  .rxavailable    = imx_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = imx_send,
  .txint          = imx_txint,
  .txready        = imx_txready,
  .txempty        = imx_txempty,
};

/* I/O buffers */

#ifdef CONFIG_IMX6_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_IMX6_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_IMX6_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

#ifdef CONFIG_IMX6_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

#ifdef CONFIG_IMX6_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
#endif

/* This describes the state of the IMX uart1 port. */

#ifdef CONFIG_IMX6_UART1
static struct imx_uart_s g_uart1priv =
{
  .uartbase       = IMX_UART1_VBASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = IMX_IRQ_UART1,
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

/* This describes the state of the IMX uart2 port. */

#ifdef CONFIG_IMX6_UART2
static struct imx_uart_s g_uart2priv =
{
  .uartbase       = IMX_UART2_VBASE,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = IMX_IRQ_UART2,
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

#ifdef CONFIG_IMX6_UART3
static struct imx_uart_s g_uart3priv =
{
  .uartbase       = IMX_UART3_REGISTER_BASE,
  .baud           = IMX_UART3_VBASE,
  .irq            = IMX_IRQ_UART3,
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

#ifdef CONFIG_IMX6_UART4
static struct imx_uart_s g_uart4priv =
{
  .uartbase       = IMX_UART4_REGISTER_BASE,
  .baud           = IMX_UART4_VBASE,
  .irq            = IMX_IRQ_UART4,
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

#ifdef CONFIG_IMX6_UART5
static struct imx_uart_s g_uart5priv =
{
  .uartbase       = IMX_UART5_REGISTER_BASE,
  .baud           = IMX_UART5_VBASE,
  .irq            = IMX_IRQ_UART5,
  .parity         = CONFIG_UART5_PARITY,
  .bits           = CONFIG_UART5_BITS,
  .stopbits2      = CONFIG_UART5_2STOP,
};

static struct uart_dev_s g_uart5port =
{
  .recv     =
  {
    .size   = CONFIG_UART5_RXBUFSIZE,
    .buffer = g_uart5rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART5_TXBUFSIZE,
    .buffer = g_uart5txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart5priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_serialin
 ****************************************************************************/

static inline uint32_t imx_serialin(struct imx_uart_s *priv, uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: imx_serialout
 ****************************************************************************/

static inline void imx_serialout(struct imx_uart_s *priv, uint32_t offset,
                                 uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: imx_disableuartint
 ****************************************************************************/

static inline void imx_disableuartint(struct imx_uart_s *priv,
                                      uint32_t *ucr1)
{
  /* Return the current Rx and Tx interrupt state */

  if (ucr1 != NULL)
    {
      *ucr1 = priv->ucr1 & (UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
    }

  /* Then disable both Rx and Tx interrupts */

  priv->ucr1 &= ~(UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  imx_serialout(priv, UART_UCR1_OFFSET, priv->ucr1);
}

/****************************************************************************
 * Name: imx_restoreuartint
 ****************************************************************************/

static inline void imx_restoreuartint(struct imx_uart_s *priv, uint32_t ucr1)
{
  /* Enable/disable any interrupts that are currently disabled but should be
   * enabled/disabled.
   */

  priv->ucr1 &= ~(UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  priv->ucr1 |= ucr1 & (UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  imx_serialout(priv, UART_UCR1_OFFSET, priv->ucr1);
}

/****************************************************************************
 * Name: imx_waittxready
 ****************************************************************************/

static inline void imx_waittxready(struct imx_uart_s *priv)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if ((imx_serialin(priv, UART_UTS_OFFSET) & UART_UTS_TXFULL) == 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: imx_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int imx_setup(struct uart_dev_s *dev)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct uart_config_s config;
  int ret;

  /* Configure the UART */

  config.baud       = priv->baud;          /* Configured baud */
  config.parity     = priv->parity;        /* 0=none, 1=odd, 2=even */
  config.bits       = priv->bits;          /* Number of bits (5-9) */
  config.stopbits2  = priv->stopbits2;     /* true: Configure with 2 stop bits instead of 1 */

  ret = imx_uart_configure(priv->uartbase, &config);

  /* Initialize the UCR1 shadow register */

  priv->ucr1 = imx_serialin(priv, UART_UCR1_OFFSET);
  return ret;

#else
  /* Initialize the UCR1 shadow register */

  priv->ucr1 = imx_serialin(priv, UART_UCR1_OFFSET);
  return OK;
#endif
}

/****************************************************************************
 * Name: imx_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void imx_shutdown(struct uart_dev_s *dev)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;

  /* Disable the UART */

  imx_serialout(priv, UART_UCR1_OFFSET, 0);
  imx_serialout(priv, UART_UCR2_OFFSET, 0);
  imx_serialout(priv, UART_UCR3_OFFSET, 0);
  imx_serialout(priv, UART_UCR4_OFFSET, 0);
}

/****************************************************************************
 * Name: imx_attach
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

static int imx_attach(struct uart_dev_s *dev)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, imx_interrupt, dev);
  if (ret == OK)
    {
      /* Configure as a (high) level interrupt */

      arm_gic_irq_trigger(priv->irq, false);

      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
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

static void imx_detach(struct uart_dev_s *dev)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: imx_interrupt (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It should cal
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ****************************************************************************/

static int imx_interrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct imx_uart_s *priv;
  uint32_t usr1;
  int passes = 0;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct imx_uart_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (; ; )
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

      usr1 = imx_serialin(priv, UART_USR1_OFFSET);
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
          (imx_serialin(priv, UART_UCR1_OFFSET) & UART_UCR1_TXEMPTYEN) != 0)
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
 * Name: imx_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int imx_ioctl(struct file *filep, int cmd, unsigned long arg)
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
        struct termios  *termiosp = (struct termios *)arg;
        struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;

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
        struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;
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
            imx_disableuartint(priv, &ie);
            ret = imx_setup(dev);

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
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: imx_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int imx_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;
  uint32_t rxd0;

  rxd0    = imx_serialin(priv, UART_RXD_OFFSET);
  *status = rxd0;
  return (rxd0 & UART_RXD_DATA_MASK) >> UART_RXD_DATA_SHIFT;
}

/****************************************************************************
 * Name: imx_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void imx_rxint(struct uart_dev_s *dev, bool enable)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;

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

  imx_serialout(priv, UART_UCR1_OFFSET, priv->ucr1);
}

/****************************************************************************
 * Name: imx_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool imx_rxavailable(struct uart_dev_s *dev)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;

  /* Return true is data is ready in the Rx FIFO */

  return ((imx_serialin(priv, UART_USR2_OFFSET) & UART_USR2_RDR) != 0);
}

/****************************************************************************
 * Name: imx_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void imx_send(struct uart_dev_s *dev, int ch)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;
  imx_serialout(priv, UART_TXD_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: imx_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void imx_txint(struct uart_dev_s *dev, bool enable)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;

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

  imx_serialout(priv, UART_UCR1_OFFSET, priv->ucr1);
}

/****************************************************************************
 * Name: imx_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool imx_txready(struct uart_dev_s *dev)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;

  /* When TXFULL is set, there is no space in the Tx FIFO  */

  return ((imx_serialin(priv, UART_UTS_OFFSET) & UART_UTS_TXFULL) == 0);
}

/****************************************************************************
 * Name: imx_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool imx_txempty(struct uart_dev_s *dev)
{
  struct imx_uart_s *priv = (struct imx_uart_s *)dev->priv;

  /* When TXDC is set, the FIFO is empty and the transmission is complete */

  return ((imx_serialin(priv, UART_USR2_OFFSET) & UART_USR2_TXDC) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm_serialinit.
 *
 ****************************************************************************/

void imx_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function imx_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  imx_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that imx_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
# ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#  ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#    ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS2_DEV);
#      ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS2_DEV);
#      endif
#    endif
#  endif
# endif
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
  struct imx_uart_s *priv = (struct imx_uart_s *)CONSOLE_DEV.priv;
  uint32_t ier;

  /* Disable UART interrupts and wait until the hardware is ready to send
   * a byte.
   */

  imx_disableuartint(priv, &ier);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      imx_lowputc('\r');
    }

  imx_lowputc(ch);
  imx_restoreuartint(priv, ier);

  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#  undef IMX_CONSOLE_VBASE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE IMX_UART1_VBASE
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE IMX_UART2_VBASE
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE IMX_UART3_VBASE
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE IMX_UART4_VBASE
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE IMX_UART5_VBASE
#  endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef IMX_CONSOLE_VBASE
static inline void imx_waittxready(void)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Loop until TXFULL is zero -- meaning that there is space available
       * in the TX FIFO.
       */

      if ((getreg32(IMX_CONSOLE_VBASE + UART_UTS) & UART_UTS_TXFULL) == 0)
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
#ifdef IMX_CONSOLE_VBASE
  imx_waittxready();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      putreg32((uint16_t)'\r', IMX_CONSOLE_VBASE + UART_TXD_OFFSET);
      imx_waittxready();
    }

  putreg32((uint16_t)ch, IMX_CONSOLE_VBASE + UART_TXD_OFFSET);
#endif

  return ch;
}

#endif /* USE_SERIALDRIVER */
