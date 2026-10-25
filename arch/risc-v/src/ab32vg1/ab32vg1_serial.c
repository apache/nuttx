/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_serial.c
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
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "ab32vg1.h"
#include "ab32vg1_lowputc.h"
#include "hardware/ab32vg1_uart.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_AB32VG1_UART0
#  error "The serial driver requires CONFIG_AB32VG1_UART0"
#endif

#ifdef CONFIG_UART0_SERIAL_CONSOLE
#  define CONSOLE_DEV     g_uart0port   /* UART0 is console */
#  define HAVE_CONSOLE    1
#endif

#define TTYS0_DEV         g_uart0port   /* UART0 is ttyS0 */

/* Key sent by host tools to reach the mask ROM loader: the "reset" token
 * followed by the sync token (see scripts/ab32vg1_dl.py).
 */

#define ROMKEY_LEN        6

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ab32vg1_uart_s
{
  uintptr_t base;         /* Base address of the UART registers */
  uint32_t  baud;         /* Configured baud */
  int       irq;          /* PIC interrupt (shared by UART0/1/2) */
  bool      stop2;        /* Two stop bits */
  bool      txbusy;       /* A byte was written and TXPND is pending */
#ifdef CONFIG_AB32VG1_UART0_ROMKEY
  uint8_t   keymatch;     /* Number of key bytes matched so far */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  ab32vg1_setup(struct uart_dev_s *dev);
static void ab32vg1_shutdown(struct uart_dev_s *dev);
static int  ab32vg1_attach(struct uart_dev_s *dev);
static void ab32vg1_detach(struct uart_dev_s *dev);
static int  ab32vg1_interrupt(int irq, void *context, void *arg);
static int  ab32vg1_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  ab32vg1_receive(struct uart_dev_s *dev, unsigned int *status);
static void ab32vg1_rxint(struct uart_dev_s *dev, bool enable);
static bool ab32vg1_rxavailable(struct uart_dev_s *dev);
static void ab32vg1_send(struct uart_dev_s *dev, int ch);
static void ab32vg1_txint(struct uart_dev_s *dev, bool enable);
static bool ab32vg1_txready(struct uart_dev_s *dev);
static bool ab32vg1_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = ab32vg1_setup,
  .shutdown       = ab32vg1_shutdown,
  .attach         = ab32vg1_attach,
  .detach         = ab32vg1_detach,
  .ioctl          = ab32vg1_ioctl,
  .receive        = ab32vg1_receive,
  .rxint          = ab32vg1_rxint,
  .rxavailable    = ab32vg1_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = ab32vg1_send,
  .txint          = ab32vg1_txint,
  .txready        = ab32vg1_txready,
  .txempty        = ab32vg1_txempty,
};

#ifdef CONFIG_AB32VG1_UART0_ROMKEY
static const uint8_t g_romkey[ROMKEY_LEN] =
{
  0xf5, 0xa0, 0xa5, 0x96, 0x87, 0x5a
};
#endif

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct ab32vg1_uart_s g_uart0priv =
{
  .base   = AB32VG1_UART0_BASE,
  .baud   = CONFIG_UART0_BAUD,
  .irq    = AB32VG1_IRQ_UART,
  .stop2  = CONFIG_UART0_2STOP != 0,
};

static uart_dev_t g_uart0port =
{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  .isconsole = true,
#endif
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t ab32vg1_serialin(struct ab32vg1_uart_s *priv,
                                        int offset)
{
  return getreg32(priv->base + offset);
}

static inline void ab32vg1_serialout(struct ab32vg1_uart_s *priv,
                                     int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static inline void ab32vg1_modifycon(struct ab32vg1_uart_s *priv,
                                     uint32_t clrbits, uint32_t setbits)
{
  irqstate_t flags = up_irq_save();

  modifyreg32(priv->base + AB32VG1_UART_CON_OFFSET, clrbits, setbits);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: ab32vg1_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc.  This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int ab32vg1_setup(struct uart_dev_s *dev)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  ab32vg1_uart0_configure(priv->baud, priv->stop2);
  priv->txbusy = false;
#endif

  return OK;
}

/****************************************************************************
 * Name: ab32vg1_shutdown
 *
 * Description:
 *   Disable the UART interrupts.  This method is called when the serial
 *   port is closed.  The console keeps working for polled output.
 *
 ****************************************************************************/

static void ab32vg1_shutdown(struct uart_dev_s *dev)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;

  ab32vg1_modifycon(priv, UART_CON_RXIE | UART_CON_TXIE, 0);
}

/****************************************************************************
 * Name: ab32vg1_attach
 *
 * Description:
 *   Configure the UART to operate in interrupt driven mode.  This method is
 *   called when the serial port is opened.
 *
 ****************************************************************************/

static int ab32vg1_attach(struct uart_dev_s *dev)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->irq, ab32vg1_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: ab32vg1_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *
 ****************************************************************************/

static void ab32vg1_detach(struct uart_dev_s *dev)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: ab32vg1_interrupt
 *
 * Description:
 *   UART interrupt handler.  The PIC source is shared by UART0/1/2 and is
 *   level triggered on RXPND/TXPND of every UART with the interrupt
 *   enabled.
 *
 ****************************************************************************/

static int ab32vg1_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;
  uint32_t con;
  int passes;

  for (passes = 0; passes < 256; passes++)
    {
      con = ab32vg1_serialin(priv, AB32VG1_UART_CON_OFFSET);

      if ((con & (UART_CON_RXIE | UART_CON_RXPND)) ==
          (UART_CON_RXIE | UART_CON_RXPND))
        {
          uart_recvchars(dev);
        }
      else if ((con & (UART_CON_TXIE | UART_CON_TXPND)) ==
               (UART_CON_TXIE | UART_CON_TXPND))
        {
          /* uart_xmitchars() either writes a new byte, which clears
           * TXPND, or disables the TX interrupt.
           */

          priv->txbusy = false;
          uart_xmitchars(dev);
        }
      else
        {
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ab32vg1_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int ab32vg1_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;
  struct termios *termiosp = (struct termios *)arg;
  int ret = OK;

  switch (cmd)
    {
      case TCGETS:
        if (termiosp == NULL)
          {
            return -EINVAL;
          }

        termiosp->c_cflag = CS8 | (priv->stop2 ? CSTOPB : 0);
        cfsetispeed(termiosp, priv->baud);
        break;

      case TCSETS:
        if (termiosp == NULL)
          {
            return -EINVAL;
          }

        /* Only 8N1/8N2 framing is supported */

        if ((termiosp->c_cflag & (CSIZE | PARENB)) != CS8)
          {
            return -EINVAL;
          }

        priv->stop2 = (termiosp->c_cflag & CSTOPB) != 0;
        priv->baud  = cfgetispeed(termiosp);
        ab32vg1_uart0_configure(priv->baud, priv->stop2);
        ab32vg1_modifycon(priv, 0, UART_CON_RXIE);
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
 * Name: ab32vg1_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character from
 *   the UART.  Error bits associated with the receipt are provided in the
 *   return 'status'.
 *
 ****************************************************************************/

static int ab32vg1_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;
  uint8_t ch;

  ch = ab32vg1_serialin(priv, AB32VG1_UART_DATA_OFFSET) & 0xff;
  ab32vg1_serialout(priv, AB32VG1_UART_CPND_OFFSET, UART_CPND_CRXPND);
  *status = 0;

#ifdef CONFIG_AB32VG1_UART0_ROMKEY
  if (ch == g_romkey[priv->keymatch])
    {
      if (++priv->keymatch == ROMKEY_LEN)
        {
          up_systemreset();
        }
    }
  else
    {
      priv->keymatch = (ch == g_romkey[0]) ? 1 : 0;
    }
#endif

  return ch;
}

/****************************************************************************
 * Name: ab32vg1_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void ab32vg1_rxint(struct uart_dev_s *dev, bool enable)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
  if (enable)
    {
      ab32vg1_modifycon(priv, 0, UART_CON_RXIE);
    }
  else
#endif
    {
      ab32vg1_modifycon(priv, UART_CON_RXIE, 0);
    }
}

/****************************************************************************
 * Name: ab32vg1_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool ab32vg1_rxavailable(struct uart_dev_s *dev)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;

  return (ab32vg1_serialin(priv, AB32VG1_UART_CON_OFFSET) &
          UART_CON_RXPND) != 0;
}

/****************************************************************************
 * Name: ab32vg1_send
 *
 * Description:
 *   This method will send one byte on the UART.  Writing the data register
 *   clears TXPND.
 *
 ****************************************************************************/

static void ab32vg1_send(struct uart_dev_s *dev, int ch)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;

  priv->txbusy = true;
  ab32vg1_serialout(priv, AB32VG1_UART_DATA_OFFSET, (uint8_t)ch);
}

/****************************************************************************
 * Name: ab32vg1_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void ab32vg1_txint(struct uart_dev_s *dev, bool enable)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      ab32vg1_modifycon(priv, 0, UART_CON_TXIE);

      /* If the transmitter is idle no TX interrupt will come: fake one by
       * calling uart_xmitchars() with interrupts disabled.
       */

      if (!priv->txbusy)
        {
          uart_xmitchars(dev);
        }
#endif
    }
  else
    {
      ab32vg1_modifycon(priv, UART_CON_TXIE, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ab32vg1_txready
 *
 * Description:
 *   Return true if a new byte can be written: no byte in flight, or the
 *   last one has been shifted out (TXPND).  The UART has no FIFO.
 *
 ****************************************************************************/

static bool ab32vg1_txready(struct uart_dev_s *dev)
{
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)dev->priv;

  return !priv->txbusy ||
         (ab32vg1_serialin(priv, AB32VG1_UART_CON_OFFSET) &
          UART_CON_TXPND) != 0;
}

/****************************************************************************
 * Name: ab32vg1_txempty
 *
 * Description:
 *   Return true if all bytes have been sent
 *
 ****************************************************************************/

static bool ab32vg1_txempty(struct uart_dev_s *dev)
{
  return ab32vg1_txready(dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before riscv_serialinit.
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
#ifdef HAVE_CONSOLE
  ab32vg1_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
#ifdef HAVE_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  uart_register("/dev/ttyS0", &TTYS0_DEV);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  struct ab32vg1_uart_s *priv = (struct ab32vg1_uart_s *)CONSOLE_DEV.priv;
  irqstate_t flags;

  /* Let a byte sent by the interrupt driven path finish first.  If TX
   * interrupts are enabled, the TXPND left by our byte resumes them.
   */

  flags = up_irq_save();
  if (priv->txbusy)
    {
      while ((ab32vg1_serialin(priv, AB32VG1_UART_CON_OFFSET) &
              UART_CON_TXPND) == 0)
        {
        }
    }

  riscv_lowputc(ch);
  priv->txbusy = true;
  up_irq_restore(flags);
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
  irqstate_t flags = up_irq_save();

  riscv_lowputc(ch);
  up_irq_restore(flags);
}

#endif /* USE_SERIALDRIVER */
