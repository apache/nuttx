/****************************************************************************
 * arch/arm/src/eoss3/eoss3_serial.c
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
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/power/pm.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/wqueue.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/eoss3_uart.h"
#include "hardware/eoss3_intr.h"
#include "eoss3_lowputc.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct eoss3_uart_s
{
  uint32_t  ie;       /* Saved IE value */
  struct work_s work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void eoss3_disableuartint(struct eoss3_uart_s *priv,
                                        uint32_t *ie);
static inline void eoss3_restoreuartint(struct eoss3_uart_s *priv,
                                        uint32_t ie);
static void eoss3_tx_work(void *arg);
static int  eoss3_setup(struct uart_dev_s *dev);
static void eoss3_shutdown(struct uart_dev_s *dev);
static int  eoss3_attach(struct uart_dev_s *dev);
static void eoss3_detach(struct uart_dev_s *dev);
static int  eoss3_interrupt(int irq, void *context, FAR void *arg);
static int  eoss3_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  eoss3_receive(struct uart_dev_s *dev, unsigned int *status);
static void eoss3_rxint(struct uart_dev_s *dev, bool enable);
static bool eoss3_rxavailable(struct uart_dev_s *dev);
static void eoss3_send(struct uart_dev_s *dev, int ch);
static void eoss3_txint(struct uart_dev_s *dev, bool enable);
static bool eoss3_txready(struct uart_dev_s *dev);
static bool eoss3_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup          = eoss3_setup,
  .shutdown       = eoss3_shutdown,
  .attach         = eoss3_attach,
  .detach         = eoss3_detach,
  .ioctl          = eoss3_ioctl,
  .receive        = eoss3_receive,
  .rxint          = eoss3_rxint,
  .rxavailable    = eoss3_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = eoss3_send,
  .txint          = eoss3_txint,
  .txready        = eoss3_txready,
  .txempty        = eoss3_txempty,
};

/* I/O buffers */

#ifdef CONFIG_EOSS3_UART
static char g_uartrxbuffer[CONFIG_UART_RXBUFSIZE];
static char g_uarttxbuffer[CONFIG_UART_TXBUFSIZE];
#endif

/* This describes the state of the EOSS3 uart port. */

#ifdef CONFIG_EOSS3_UART
static struct eoss3_uart_s g_uartpriv;

static struct uart_dev_s g_uartport =
{
  .recv         =
  {
    .size       = CONFIG_UART_RXBUFSIZE,
    .buffer     = g_uartrxbuffer,
  },
  .xmit         =
  {
    .size       = CONFIG_UART_TXBUFSIZE,
    .buffer     = g_uarttxbuffer,
  },
  .ops          = &g_uart_ops,
  .priv         = &g_uartpriv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eoss3_disableuartint
 ****************************************************************************/

static inline void eoss3_disableuartint(struct eoss3_uart_s *priv,
                                        uint32_t *ie)
{
  if (ie)
    {
      *ie = priv->ie & UART_IMSC_ALLINTS;
    }

  priv->ie &= ~UART_IMSC_ALLINTS;
  putreg32(priv->ie, EOSS3_UART_IMSC);
}

/****************************************************************************
 * Name: eoss3_restoreuartint
 ****************************************************************************/

static inline void eoss3_restoreuartint(struct eoss3_uart_s *priv,
                                        uint32_t ie)
{
  priv->ie |= ie & UART_IMSC_ALLINTS;
  putreg32(priv->ie, EOSS3_UART_IMSC);
}

/****************************************************************************
 * Name: eoss3_tx_work
 *
 * Description:
 *   We expect that the TX FIFO should be ready for data again
 *
 ****************************************************************************/

static void eoss3_tx_work(void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct eoss3_uart_s *priv = (struct eoss3_uart_s *)dev->priv;
  irqstate_t flags;

  uart_xmitchars(dev);

  /* Check if there is still data to send, if so reschedule */

  flags = enter_critical_section();
  if (dev->xmit.head != dev->xmit.tail)
    {
      work_queue(HPWORK, &priv->work, eoss3_tx_work,
                 (FAR void *)arg, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: eoss3_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int eoss3_setup(struct uart_dev_s *dev)
{
  struct eoss3_uart_s *priv = (struct eoss3_uart_s *)dev->priv;
  int ret;
  uint32_t lcrh;

  /* Grab starting state of interrupts */

  priv->ie = getreg32(EOSS3_UART_IMSC);

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Enable the FIFO
   * This is especially important since we are faking tx empty interrupt.
   */

  lcrh = getreg32(EOSS3_UART_LCR_H);
  lcrh |= UART_LCR_H_FEN;
  putreg32(lcrh, EOSS3_UART_LCR_H);
  ret = OK;
  return ret;
#else
  UNUSED(ret);
  UNUSED(lcrh);
  return OK;
#endif
}

/****************************************************************************
 * Name: eoss3_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void eoss3_shutdown(struct uart_dev_s *dev)
{
  struct eoss3_uart_s *priv = (struct eoss3_uart_s *)dev->priv;
  eoss3_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: eoss3_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate
 *   in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int eoss3_attach(struct uart_dev_s *dev)
{
  int ret;
  uint32_t m4_intrc;

  /* Attach and enable the IRQ */

  ret = irq_attach(EOSS3_IRQ_UART, eoss3_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART. This also turns on the top level routing.  This may
       * later move into the up_enable_irq logic.
       */

      putreg32(INTR_UART_DET, EOSS3_INTR_OTHER);
      putreg32(UART_IMSC_ALLINTS, EOSS3_UART_ICR);
      m4_intrc = getreg32(EOSS3_INTR_OTHER_EN_M4);
      m4_intrc |= INTR_UART_EN_M4;
      putreg32(m4_intrc, EOSS3_INTR_OTHER_EN_M4);
      up_enable_irq(EOSS3_IRQ_UART);
    }

  return ret;
}

/****************************************************************************
 * Name: eoss3_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void eoss3_detach(struct uart_dev_s *dev)
{
  up_disable_irq(EOSS3_IRQ_UART);
  irq_detach(EOSS3_IRQ_UART);
}

/****************************************************************************
 * Name: eoss3_interrupt
 *
 * Description:
 *   This is the common UART interrupt handler.  It should call
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ****************************************************************************/

static int eoss3_interrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint32_t status;
  int passes;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

      status  = getreg32(EOSS3_UART_MIS);

      /* If no interrupts remaining break */

      if (status == 0)
        {
          putreg32(UART_IMSC_ALLINTS, EOSS3_UART_ICR);
          break;
        }

      /* Handle incoming, receive bytes */

      if (status & (UART_MIS_RXMIS | UART_MIS_RTMIS))
        {
          uart_recvchars(dev);
        }
    }

  /* This will clear the pending interrupt flag on the top level */

  putreg32(INTR_UART_DET, EOSS3_INTR_OTHER);

  return OK;
}

/****************************************************************************
 * Name: eoss3_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int eoss3_ioctl(struct file *filep, int cmd, unsigned long arg)
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
         struct eoss3_uart_s *user = (struct eoss3_uart_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct eoss3_uart_s));
           }
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
 * Name: eoss3_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int eoss3_receive(struct uart_dev_s *dev, unsigned int *status)
{
  *status = getreg32(EOSS3_UART_RSR_ECR);
  return (getreg32(EOSS3_UART_DR) & UART_DR_DATA_MASK);
}

/****************************************************************************
 * Name: eoss3_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void eoss3_rxint(struct uart_dev_s *dev, bool enable)
{
  struct eoss3_uart_s *priv = (struct eoss3_uart_s *)dev->priv;

  /* Enable interrupts for data available at Rx */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= (UART_IMSC_RXIM | UART_IMSC_RTIM);
#endif
    }
  else
    {
      priv->ie &= ~(UART_IMSC_RXIM | UART_IMSC_RTIM);
    }

  putreg32(priv->ie, EOSS3_UART_IMSC);
}

/****************************************************************************
 * Name: eoss3_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool eoss3_rxavailable(struct uart_dev_s *dev)
{
  return ((getreg32(EOSS3_UART_TFR) & UART_TFR_RXFE) == 0);
}

/****************************************************************************
 * Name: eoss3_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void eoss3_send(struct uart_dev_s *dev, int ch)
{
  putreg32(ch & UART_DR_DATA_MASK, EOSS3_UART_DR);
}

/****************************************************************************
 * Name: eoss3_txint
 *
 * Description:
 *   Normally would turn on and off the tx empty interrupt; instead, we are
 *   enabling a kernel worker because there is no interrupt. This worker
 *   will requeue and dequeue itself as needed.
 *
 ****************************************************************************/

static void eoss3_txint(struct uart_dev_s *dev, bool enable)
{
  struct eoss3_uart_s *priv = (struct eoss3_uart_s *)dev->priv;

  if (enable)
    {
      if (work_available(&priv->work))
        {
          work_queue(HPWORK, &priv->work, eoss3_tx_work,
                     (FAR void *)dev, 0);
        }
    }
}

/****************************************************************************
 * Name: eoss3_txready
 *
 * Description:
 *   Return true if the transmit fifo is not full
 *
 ****************************************************************************/

static bool eoss3_txready(struct uart_dev_s *dev)
{
  return ((getreg32(EOSS3_UART_TFR) & UART_TFR_TXFF) == 0);
}

/****************************************************************************
 * Name: eoss3_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool eoss3_txempty(struct uart_dev_s *dev)
{
  return ((getreg32(EOSS3_UART_TFR) & UART_TFR_TXFE) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function eoss3_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

#ifdef CONFIG_UART_SERIAL_CONSOLE
  g_uartport.isconsole = true;
  eoss3_setup(&g_uartport);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that eoss3_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef CONFIG_UART_SERIAL_CONSOLE
  uart_register("/dev/console", &g_uartport);
#endif
#ifdef CONFIG_EOSS3_UART
  uart_register("/dev/ttyS0", &g_uartport);
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
#ifdef CONFIG_UART_SERIAL_CONSOLE
  struct eoss3_uart_s *priv = (struct eoss3_uart_s *)g_uartport.priv;
  uint32_t ie;

  eoss3_disableuartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  eoss3_restoreuartint(priv, ie);
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
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  /* Output the character */

  arm_lowputc(ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
