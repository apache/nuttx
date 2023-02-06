/****************************************************************************
 * arch/arm/src/c5471/c5471_serial.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BASE_BAUD     115200

#if defined(CONFIG_UART_IRDA_HWFLOWCONTROL) || defined(CONFIG_UART_MODEM_HWFLOWCONTROL)
# define CONFIG_UART_HWFLOWCONTROL
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_regs_s
{
  uint32_t  ier;
  uint32_t  lcr;
  uint32_t  fcr;
#ifdef CONFIG_UART_HWFLOWCONTROL
  uint32_t  efr;
  uint32_t  tcr;
#endif
};

struct up_dev_s
{
  unsigned int         uartbase;    /* Base address of UART registers */
  unsigned int         baud_base;   /* Base baud for conversions */
  unsigned int         baud;        /* Configured baud */

  uint8_t              xmit_fifo_size; /* Size of transmit FIFO */

  uint8_t              irq;         /* IRQ associated with this UART */
  uint8_t              parity;      /* 0=none, 1=odd, 2=even */
  uint8_t              bits;        /* Number of bits (7 or 8) */
#ifdef CONFIG_UART_HWFLOWCONTROL
  bool                 flowcontrol; /* true: Hardware flow control
                                     * is enabled. */
#endif
  bool                 stopbits2;   /* true: Configure with 2
                                     * stop bits instead of 1 */
  struct uart_regs_s   regs;        /* Shadow copy of readonly regs */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
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

static char g_irdarxbuffer[CONFIG_UART_IRDA_RXBUFSIZE];
static char g_irdatxbuffer[CONFIG_UART_IRDA_TXBUFSIZE];
static char g_modemrxbuffer[CONFIG_UART_MODEM_RXBUFSIZE];
static char g_modemtxbuffer[CONFIG_UART_MODEM_TXBUFSIZE];

/* This describes the state of the C5471 serial IRDA port. */

static struct up_dev_s g_irdapriv =
{
  .xmit_fifo_size = UART_IRDA_XMIT_FIFO_SIZE,
  .baud_base      = BASE_BAUD,
  .uartbase       = UART_IRDA_BASE,
  .baud           = CONFIG_UART_IRDA_BAUD,
  .irq            = C5471_IRQ_UART_IRDA,
  .parity         = CONFIG_UART_IRDA_PARITY,
  .bits           = CONFIG_UART_IRDA_BITS,
#ifdef CONFIG_UART_IRDA_HWFLOWCONTROL
  .flowcontrol    = true,
#endif
  .stopbits2      = CONFIG_UART_IRDA_2STOP,
};

static uart_dev_t g_irdaport =
{
  .recv     =
  {
    .size   = CONFIG_UART_IRDA_RXBUFSIZE,
    .buffer = g_irdarxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART_IRDA_TXBUFSIZE,
    .buffer = g_irdatxbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_irdapriv,
};

/* This describes the state of the C5471 serial Modem port. */

static struct up_dev_s g_modempriv =
{
  .xmit_fifo_size = UART_XMIT_FIFO_SIZE,
  .baud_base      = BASE_BAUD,
  .uartbase       = UART_MODEM_BASE,
  .baud           = CONFIG_UART_MODEM_BAUD,
  .irq            = C5471_IRQ_UART,
  .parity         = CONFIG_UART_MODEM_PARITY,
  .bits           = CONFIG_UART_MODEM_BITS,
#ifdef CONFIG_UART_MODEM_HWFLOWCONTROL
  .flowcontrol    = true,
#endif
  .stopbits2      = CONFIG_UART_MODEM_2STOP,
};

static uart_dev_t g_modemport =
{
  .recv     =
  {
    .size   = CONFIG_UART_MODEM_RXBUFSIZE,
    .buffer = g_modemrxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART_MODEM_TXBUFSIZE,
    .buffer = g_modemtxbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_modempriv,
};

/* Now, which one with be tty0/console and which tty1? */

#ifdef CONFIG_SERIAL_IRDA_CONSOLE
# define CONSOLE_DEV     g_irdaport
# define TTYS0_DEV       g_irdaport
# define TTYS1_DEV       g_modemport
#else
# define CONSOLE_DEV     g_modemport
# define TTYS0_DEV       g_modemport
# define TTYS1_DEV       g_irdaport
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_inserial
 ****************************************************************************/

static inline uint32_t up_inserial(struct up_dev_s *priv, uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv,
                                uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint16_t *ier)
{
  if (ier)
    {
      *ier = priv->regs.ier & UART_IER_INTMASK;
    }

  priv->regs.ier &= ~UART_IER_INTMASK;
  up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint16_t ier)
{
  priv->regs.ier |= ier & (UART_IER_RECVINT | UART_IER_XMITINT);
  up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

static inline void up_waittxready(struct up_dev_s *priv)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if ((up_inserial(priv, UART_SSR_OFFS) & UART_SSR_TXFULL) == 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: up_disablebreaks
 ****************************************************************************/

static inline void up_disablebreaks(struct up_dev_s *priv)
{
  priv->regs.lcr &= ~UART_LCR_BOC;
  up_serialout(priv, UART_LCR_OFFS, priv->regs.lcr);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv)
{
  priv->regs.lcr |= UART_LCR_BOC;
  up_serialout(priv, UART_LCR_OFFS, priv->regs.lcr);
}

/****************************************************************************
 * Name: up_setrate
 ****************************************************************************/

static inline void up_setrate(struct up_dev_s *priv, unsigned int rate)
{
  uint32_t div_bit_rate;

  switch (rate)
    {
    case 115200:
      div_bit_rate = BAUD_115200;
      break;
    case 57600:
      div_bit_rate = BAUD_57600;
      break;
    case 38400:
      div_bit_rate = BAUD_38400;
      break;
    case 19200:
      div_bit_rate = BAUD_19200;
      break;
    case 4800:
      div_bit_rate = BAUD_4800;
      break;
    case 2400:
      div_bit_rate = BAUD_2400;
      break;
    case 1200:
      div_bit_rate = BAUD_1200;
      break;
    case 9600:
    default:
      div_bit_rate = BAUD_9600;
      break;
    }

  up_serialout(priv, UART_DIV_BIT_RATE_OFFS, div_bit_rate);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = dev->priv;
  unsigned int cval;

  if (priv->bits == 7)
    {
      cval = UART_LCR_7BITS;
    }
  else
    {
      cval = UART_LCR_8BITS;
    }

  if (priv->stopbits2)
    {
      cval |= UART_LCR_2STOP;
    }

  if (priv->parity == 1)   /* Odd parity */
    {
      cval |= (UART_LCR_PAREN | UART_LCR_PARODD);
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      cval |= (UART_LCR_PAREN | UART_LCR_PAREVEN);
    }

  /* Both the IrDA and MODEM UARTs support RESET and UART mode. */

  up_serialout(priv, UART_MDR_OFFS, MDR_RESET_MODE);
  up_mdelay(5);
  up_serialout(priv, UART_MDR_OFFS, MDR_UART_MODE);
  up_mdelay(5);

  priv->regs.ier = up_inserial(priv, UART_IER_OFFS);
  priv->regs.lcr = up_inserial(priv, UART_LCR_OFFS);
#ifdef CONFIG_UART_HWFLOWCONTROL
  if (priv->flowcontrol)
    {
      priv->regs.efr = up_inserial(priv, UART_EFR_OFFS);
      priv->regs.tcr = up_inserial(priv, UART_TCR_OFFS);
    }
#endif

  up_disableuartint(priv, NULL);

  up_serialout(priv, UART_EFR_OFFS,  0x0010);           /* Enable fifo control */
  up_serialout(priv, UART_TFCR_OFFS, 0);                /* Reset to 0 */
  up_serialout(priv, UART_RFCR_OFFS, UART_FCR_RX_CLR);  /* Clear RX fifo */
  up_serialout(priv, UART_TFCR_OFFS, UART_FCR_TX_CLR);  /* Clear TX fifo */
  up_serialout(priv, UART_TFCR_OFFS, UART_FCR_FIFO_EN); /* Enable RX/TX fifos */

  up_disablebreaks(priv);

  /* Set the RX and TX trigger levels to the minimum */

  priv->regs.fcr = (priv->regs.fcr & 0xffffffcf) | UART_FCR_FTL;
  up_serialout(priv, UART_RFCR_OFFS, priv->regs.fcr);

  priv->regs.fcr = (priv->regs.fcr & 0xffffff3f) | UART_FCR_FTL;
  up_serialout(priv, UART_RFCR_OFFS, priv->regs.fcr);

  up_setrate(priv, priv->baud);

  priv->regs.lcr &= 0xffffffe0;        /* clear original field, and... */
  priv->regs.lcr |= (uint32_t)cval;    /* Set new bits in that field. */
  up_serialout(priv, UART_LCR_OFFS, priv->regs.lcr);

#ifdef CONFIG_UART_HWFLOWCONTROL
  if (priv->flowcontrol)
    {
      /* Set the FIFO level triggers for flow control
       * Halt = 48 bytes, resume = 12 bytes
       */

      priv->regs.tcr = (priv->regs.tcr & 0xffffff00) | 0x0000003c;
      up_serialout(priv, UART_TCR_OFFS, priv->regs.tcr);

      /* Enable RTS/CTS flow control */

      priv->regs.efr |= 0x000000c0;
      up_serialout(priv, UART_EFR_OFFS, priv->regs.efr);
    }
  else
    {
      /* Disable RTS/CTS flow control */

      priv->regs.efr &= 0xffffff3f;
      up_serialout(priv, UART_EFR_OFFS, priv->regs.efr);
    }
#endif
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

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  up_disableuartint(priv, NULL);
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
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling). The RX and
 *   TX interrupts are not enabled until the txint() and rxint() methods are
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
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  volatile uint32_t  cause;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  cause = up_inserial(priv, UART_ISR_OFFS) & 0x0000003f;

  if ((cause & 0x0000000c) == 0x0000000c)
    {
      uint32_t ier_val = 0;

      /* Is this an interrupt from the IrDA UART? */

      if (irq == C5471_IRQ_UART_IRDA)
        {
          /* Save the currently enabled IrDA UART interrupts
           * so that we can restore the IrDA interrupt state
           * below.
           */

          ier_val = up_inserial(priv, UART_IER_OFFS);

          /* Then disable all IrDA UART interrupts */

          up_serialout(priv, UART_IER_OFFS, 0);
        }

      /* Receive characters from the RX fifo */

      uart_recvchars(dev);

      /* read UART_RHR to clear int condition
       * toss = up_inserialchar(priv,&status);
       */

      /* Is this an interrupt from the IrDA UART? */

      if (irq == C5471_IRQ_UART_IRDA)
        {
          /* Restore the IrDA UART interrupt enables */

          up_serialout(priv, UART_IER_OFFS, ier_val);
        }
    }
  else if ((cause & 0x0000000c) == 0x00000004)
    {
      uart_recvchars(dev);
    }

  if ((cause & 0x00000002) != 0)
    {
      uart_xmitchars(dev);
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
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s *)arg;
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

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = enter_critical_section();
        up_enablebreaks(priv);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = enter_critical_section();
        up_disablebreaks(priv);
        leave_critical_section(flags);
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
 * Called (usually) from the interrupt level to receive one character from
 * the UART.  Error bits associated with the receipt are provided in the
 * the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rhr;
  uint32_t lsr;

  /* Construct a 16bit status word that uses the high byte to
   * hold the status bits associated with framing,parity,break
   * and a low byte that holds error bits of LSR for
   * conditions such as overflow, etc.
   */

  rhr = up_inserial(priv, UART_RHR_OFFS);
  lsr = up_inserial(priv, UART_LSR_OFFS);

  *status = (unsigned int)((rhr & 0x0000ff00) | (lsr & 0x000000ff));

  return rhr & 0x000000ff;
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
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->regs.ier |= UART_IER_RECVINT;
      up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
#endif
    }
  else
    {
      priv->regs.ier &= ~UART_IER_RECVINT;
      up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
    }
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return up_inserial(priv, UART_LSR_OFFS) & UART_RX_FIFO_NOEMPTY;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, UART_THR_OFFS, (uint8_t)ch);
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
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->regs.ier |= UART_IER_XMITINT;
      up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
#endif
    }
  else
    {
      priv->regs.ier &= ~UART_IER_XMITINT;
      up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
    }
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return (up_inserial(priv, UART_SSR_OFFS) & UART_SSR_TXFULL) == 0;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return (up_inserial(priv, UART_LSR_OFFS) & UART_LSR_TREF) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm_serialinit.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  up_disableuartint(TTYS0_DEV.priv, NULL);
  up_disableuartint(TTYS1_DEV.priv, NULL);

  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  uart_register("/dev/console", &CONSOLE_DEV);
  uart_register("/dev/ttyS0", &TTYS0_DEV);
  uart_register("/dev/ttyS1", &TTYS1_DEV);
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
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint16_t  ier;

  up_disableuartint(priv, &ier);
  up_waittxready(priv);
  up_serialout(priv, UART_THR_OFFS, (uint8_t)ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxready(priv);
      up_serialout(priv, UART_THR_OFFS, '\r');
    }

  up_waittxready(priv);
  up_restoreuartint(priv, ier);
  return ch;
}
