/****************************************************************************
 * arch/sparc/src/s698pm/s698pm-serial.c
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
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include "sparc_internal.h"
#include "s698pm-config.h"
#include "chip.h"
#include "s698pm-uart.h"
#include "s698pm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART1-4 */

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
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_S698PM_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_S698PM_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(CONFIG_S698PM_UART3)
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  elif defined(CONFIG_S698PM_UART4)
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART1-4 excluding the console UART. */

#if defined(CONFIG_S698PM_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_S698PM_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_S698PM_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_S698PM_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART2-4. It can't be UART1 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-4 could also be the
 * console.
 */

#if defined(CONFIG_S698PM_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_S698PM_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_S698PM_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART3-4. It can't be UART1-2 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART 2-4 could also be the console.
 */

#if defined(CONFIG_S698PM_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_S698PM_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of sparc_earlyserialinit(), sparc_serialinit(), and
 * up_putc().
 */

#ifdef HAVE_UART_DEVICE

/* These values describe the set of enabled interrupts */

#define RX_ENABLED(im)    (((im) & MSK_UART_ENABLE_RX) != 0)
#define TX_ENABLED(im)    (((im) & MSK_UART_ENABLE_TX) != 0)

#define ENABLE_RX(im)     do { (im) |= MSK_UART_ENABLE_RX; } while (0)
#define ENABLE_TX(im)     do { (im) |= MSK_UART_ENABLE_TX; } while (0)

#define DISABLE_RX(im)    do { (im) &= ~MSK_UART_ENABLE_RX; } while (0)
#define DISABLE_TX(im)    do { (im) &= ~MSK_UART_ENABLE_TX; } while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase;  /* Base address of UART registers */
  uint32_t  baud;      /* Configured baud */
  uint8_t   irq;       /* IRQ associated with this UART (for attachment) */
  uint8_t   im;        /* Interrupt mask state */
  uint8_t   parity;    /* 0=none, 1=odd, 2=even */
  uint8_t   bits;      /* Number of bits (5, 6, 7 or 8) */
  bool      stopbits2; /* true: Configure with 2 stop bits instead of 1 */
  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers */

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset);
static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value);
static void up_restoreuartint(struct uart_dev_s *dev, uint8_t im);
static void up_disableuartint(struct uart_dev_s *dev, uint8_t *im);

/* Serial driver methods */

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
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

/* I/O buffers */

#ifdef CONFIG_S698PM_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_S698PM_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif
#ifdef CONFIG_S698PM_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif
#ifdef CONFIG_S698PM_UART4
static char g_uart3rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

/* This describes the state of the S698PM UART1 port. */

#ifdef CONFIG_S698PM_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase  = S698PM_UART1_BASE,
  .baud      = CONFIG_UART1_BAUD,
  .irq       = S698PM_IRQ_UART_1_RX_TX,
  .parity    = CONFIG_UART1_PARITY,
  .bits      = CONFIG_UART1_BITS,
  .stopbits2 = CONFIG_UART1_2STOP,
};

static uart_dev_t g_uart1port =
{
  .recv      =
  {
    .size    = CONFIG_UART1_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART1_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart1priv,
};
#endif

/* This describes the state of the S698PM UART2 port. */

#ifdef CONFIG_S698PM_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase  = S698PM_UART2_BASE,
  .baud      = CONFIG_UART2_BAUD,
  .irq       = S698PM_IRQ_UART_2_RX_TX,
  .parity    = CONFIG_UART2_PARITY,
  .bits      = CONFIG_UART2_BITS,
  .stopbits2 = CONFIG_UART2_2STOP,
};

static uart_dev_t g_uart2port =
{
  .recv      =
  {
    .size    = CONFIG_UART2_RXBUFSIZE,
    .buffer  = g_uart2rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART2_TXBUFSIZE,
    .buffer  = g_uart2txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart2priv,
};
#endif

/* This describes the state of the S698PM UART3 port. */

#ifdef CONFIG_S698PM_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase  = S698PM_UART3_BASE,
  .baud      = CONFIG_UART3_BAUD,
  .irq       = S698PM_IRQ_UART_3_RX_TX,
  .parity    = CONFIG_UART3_PARITY,
  .bits      = CONFIG_UART3_BITS,
  .stopbits2 = CONFIG_UART3_2STOP,
};

static uart_dev_t g_uart3port =
{
  .recv      =
  {
    .size    = CONFIG_UART3_RXBUFSIZE,
    .buffer  = g_uart3rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART3_TXBUFSIZE,
    .buffer  = g_uart3txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart3priv,
};
#endif

/* This describes the state of the S698PM UART4 port. */

#ifdef CONFIG_S698PM_UART4
static struct up_dev_s g_uart4priv =
{
  .uartbase  = S698PM_UART4_BASE,
  .baud      = CONFIG_UART4_BAUD,
  .irq       = S698PM_IRQ_UART_4_RX_TX,
  .parity    = CONFIG_UART4_PARITY,
  .bits      = CONFIG_UART4_BITS,
  .stopbits2 = CONFIG_UART4_2STOP,
};

static uart_dev_t g_uart4port =
{
  .recv      =
  {
    .size    = CONFIG_UART4_RXBUFSIZE,
    .buffer  = g_uart4rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART4_TXBUFSIZE,
    .buffer  = g_uart4txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart4priv,
};
#endif
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_setuartint
 ****************************************************************************/

static inline void up_setuartint(struct up_dev_s *priv)
{
  uint8_t regval;

  regval   = up_serialin(priv, S698PM_UART_CTRLREG_OFFSET);
  regval  &= ~MSK_UART_ALLINTS;
  regval  |= priv->im;
  up_serialout(priv, S698PM_UART_CTRLREG_OFFSET, regval);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static void up_restoreuartint(struct uart_dev_s *dev, uint8_t im)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  /* Re-enable/re-disable interrupts corresponding to the state of bits
   * in im
   */

  flags = spin_lock_irqsave(&priv->lock);
  priv->im  &= ~MSK_UART_ALLINTS;
  priv->im  |= im;
  up_setuartint(priv);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static void up_disableuartint(struct uart_dev_s *dev, uint8_t *im)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (im)
    {
      *im = priv->im;
    }

  /* Disable all interrupts */

  DISABLE_RX(priv->im);
  DISABLE_TX(priv->im);
  up_setuartint(priv);
  spin_unlock_irqrestore(&priv->lock, flags);
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
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Configure the UART as an RS-232 UART */

  s698pm_uartconfigure(priv->uartbase, priv->baud, priv->parity,
                        priv->bits, priv->stopbits2);
#endif

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set up the interrupt priority */

  up_prioritize_irq(priv->irq, priv->irqprio);
#endif

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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disableuartint(dev, NULL);

  /* Reset hardware and disable Rx and Tx */

  s698pm_uartreset(priv->uartbase);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      /* Set up to uart interrupts on the current CPU */

       (void)s698pm_setup_irq(0, priv->irq, 0);

      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the USART
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
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disableuartint(dev, NULL);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t           mis;
  int                passes;
  bool               handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the masked UART status and clear the pending interrupts. */

       mis = up_serialin(priv, S698PM_UART_STATREG_OFFSET);

      /* Handle incoming, receive bytes */

      if ((mis & UART_STA_DR) != 0)
        {
          /* Rx buffer not empty ... process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((mis & UART_STA_TF) != UART_STA_TF)
        {
          /* Tx FIFO not full ... process outgoing bytes */

          uart_xmitchars(dev);
          handled = true;
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

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode      *inode;
  struct uart_dev_s *dev;
  struct up_dev_s   *priv;
  int                ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv);
  priv = (struct up_dev_s *)dev->priv;

  switch (cmd)
    {
    case xxx: /* Add commands here */
      break;

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
        s698pm_uartconfigure(priv->uartbase, priv->baud, priv->parity,
                              priv->bits, priv->stopbits2);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return status information */

  if (status)
    {
      *status = 0; /* We are not yet tracking serial errors */
    }

  /* Then return the actual received byte */

  return  (int)(up_serialin(priv, S698PM_UART_RXREG_OFFSET) &
                UART_RXREG_MASK);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx FIFO (or an Rx
       * timeout occurs.
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= MSK_UART_ENABLE_RXIT;
#endif
    }
  else
    {
      priv->im &= ~MSK_UART_ENABLE_RXIT;
    }

  up_setuartint(priv);
  spin_unlock_irqrestore(&priv->lock, flags);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return true is data is available in the receive data buffer */

  return (up_serialin(priv, S698PM_UART_STATREG_OFFSET) &
          UART_STA_DR) != 0;
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, S698PM_UART_TXREG_OFFSET, (uint32_t)ch);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= MSK_UART_ENABLE_TXIT;
      up_setuartint(priv);

      /* Fake a TX interrupt */

#  ifdef CONFIG_SMP
      spin_unlock_irqrestore(&priv->lock, flags);
#  endif
      uart_xmitchars(dev);
#  ifdef CONFIG_SMP
      flags = spin_lock_irqsave(&priv->lock);
#  endif
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~MSK_UART_ENABLE_TXIT;
      up_setuartint(priv);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the Transmit buffer register is not full */

  return (up_serialin(priv, S698PM_UART_STATREG_OFFSET) &
          UART_STA_TF) != UART_STA_TF;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the Transmit shift register is empty */

  return (up_serialin(priv, S698PM_UART_STATREG_OFFSET) &
          UART_STA_TS) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sparc_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before sparc_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in sparc_consoleinit() and main clock
 *   iniialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void sparc_earlyserialinit(void)
{
  /* Disable interrupts from all UARTS.  The console is enabled in
   * s698pm_consoleinit().
   */

  up_disableuartint(&TTYS0_DEV, NULL);
#ifdef TTYS1_DEV
  up_disableuartint(&TTYS1_DEV, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableuartint(&TTYS2_DEV, NULL);
#endif
#ifdef TTYS3_DEV
  up_disableuartint(&TTYS3_DEV, NULL);
#endif
  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: sparc_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that sparc_earlyserialinit was called previously.
 *
 ****************************************************************************/

void sparc_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV);
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
#ifdef HAVE_SERIAL_CONSOLE
  struct uart_dev_s *dev = (struct uart_dev_s *)&CONSOLE_DEV;
  uint8_t imr = 0;

  up_disableuartint(dev, &imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      sparc_lowputc('\r');
    }

  sparc_lowputc(ch);
  up_restoreuartint(dev, imr);
#endif
  return ch;
}

/****************************************************************************
 * Name: sparc_earlyserialinit, sparc_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/sparc_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/
#else /* HAVE_UART_DEVICE */
void sparc_earlyserialinit(void)
{
}

void sparc_serialinit(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
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
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      sparc_lowputc('\r');
    }

  sparc_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */

