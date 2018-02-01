/****************************************************************************
 * arch/arm/src/bcm2708/bcm_miniuart.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/serial.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/bcm2708_aux.h"
#include "bcm_lowputc.h"
#include "bcm_aux.h"

#ifdef CONFIG_BCM2708_MINI_UART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcm_dev_s
{
  uint32_t      baud;      /* Configured baud */
  uint8_t       parity;    /* 0=none, 1=odd, 2=even */
  uint8_t       bits;      /* Number of bits (8 or 9) */
  uint8_t       stop2;     /* Use 2 stop bits */
  uint8_t       ier;       /* Interrupt enable register shadow */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool          iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool          oflow;     /* output flow control (CTS) enabled */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  gpio_pinset_t rts_gpio;  /* UART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  gpio_pinset_t cts_gpio;  /* UART CTS GPIO pin configuration */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  bcm_setup(struct uart_dev_s *dev);
static void bcm_shutdown(struct uart_dev_s *dev);
static int  bcm_attach(struct uart_dev_s *dev);
static void bcm_detach(struct uart_dev_s *dev);
static int  bcm_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  bcm_receive(struct uart_dev_s *dev, uint32_t *status);
static void bcm_rxint(struct uart_dev_s *dev, bool enable);
static bool bcm_rxavailable(struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool bcm_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
                             bool upper);
#endif
static void bcm_send(struct uart_dev_s *dev, int ch);
static void bcm_txint(struct uart_dev_s *dev, bool enable);
static bool bcm_txready(struct uart_dev_s *dev);
static bool bcm_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_miniuart_ops =
{
  .setup          = bcm_setup,
  .shutdown       = bcm_shutdown,
  .attach         = bcm_attach,
  .detach         = bcm_detach,
  .ioctl          = bcm_ioctl,
  .receive        = bcm_receive,
  .rxint          = bcm_rxint,
  .rxavailable    = bcm_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = bcm_rxflowcontrol,
#endif
  .send           = bcm_send,
  .txint          = bcm_txint,
  .txready        = bcm_txready,
  .txempty        = bcm_txempty,
};

/* I/O buffers */

static char g_miniuart_rxbuffer[CONFIG_BCM2708_MINI_UART_RXBUFSIZE];
static char g_miniuart_txbuffer[CONFIG_BCM2708_MINI_UART_TXBUFSIZE];

/* This describes the state of the Kinetis Mini-UART0 port. */

static struct bcm_dev_s g_miniuart_priv =
{
  .baud           = CONFIG_BCM2708_MINI_UART_BAUD,
  .parity         = CONFIG_BCM2708_MINI_UART_BITS,
  .bits           = CONFIG_BCM2708_MINI_UART_PARITY,
  .stop2          = CONFIG_BCM2708_MINI_UART_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(BCM2708_MINI_UART_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = xxxx,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(BCM2708_MINI_UART_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = xxxx,
#endif
};

static uart_dev_t g_miniuart_port =
{
  .recv     =
  {
    .size   = CONFIG_BCM2708_MINI_UART_RXBUFSIZE,
    .buffer = g_miniuart_rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_BCM2708_MINI_UART_TXBUFSIZE,
    .buffer = g_miniuart_txbuffer,
   },
  .ops      = &g_miniuart_ops,
  .priv     = &g_miniuart_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_setuartint
 ****************************************************************************/

static void bcm_setuartint(struct bcm_dev_s *priv)
{
  modifyreg8(BCM_AUX_MU_IER, BCM_AUX_MU_IO_TXD | BCM_AUX_MU_IO_RXD, priv->ier);
}

/****************************************************************************
 * Name: bcm_restoreuartint
 ****************************************************************************/

static void bcm_restoreuartint(struct bcm_dev_s *priv, uint8_t ier)
{
  irqstate_t flags;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in ie */

  flags     = enter_critical_section();
  priv->ier = ier & (BCM_AUX_MU_IO_TXD | BCM_AUX_MU_IO_RXD);
  bcm_setuartint(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bcm_disableuartint
 ****************************************************************************/

static void bcm_disableuartint(struct bcm_dev_s *priv, uint8_t *ier)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (ier != NULL)
    {
      *ier = priv->ier;
    }

  bcm_restoreuartint(priv, 0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bcm_setup
 *
 * Description:
 *   Configure the Mini-UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int bcm_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct bcm_dev_s *priv = (struct bcm_dev_s *)dev->priv;
  struct uart_config_s config;
#endif

  /* Enable Mini-UART register access */

  bcm_aux_disable(BCM_AUX_MINI_UART);

  /* Make sure that all interrupts are disabled */

  bcm_restoreuartint(priv, 0);

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the Mini-UART */

  config.baud      = priv->baud;
  config.parity    = priv->parity;
  config.bits      = priv->bits;
  config.stopbits2 = priv->stop2;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  config.iflow     = priv->iflow;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  config.oflow     = priv->oflow;
#endif

  bcm_miniuart_configure(&config);
#endif

  return OK;
}

/****************************************************************************
 * Name: bcm_shutdown
 *
 * Description:
 *   Disable the Mini-UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void bcm_shutdown(struct uart_dev_s *dev)
{
  struct bcm_dev_s *priv = (struct bcm_dev_s *)dev->priv;

  /* Disable interrupts */

  bcm_restoreuartint(priv, 0);

  /* Disable receiver and tranmsmitter */

  putreg8(0, BCM_AUX_MU_CNTL);

  /* Disable the Mini-UART */

  bcm_aux_disable(BCM_AUX_MINI_UART);
}

/****************************************************************************
 * Name: bcm_attach
 *
 * Description:
 *   Configure the Mini-UART to operation in interrupt driven mode.  This
 *   method is called when the serial port is opened.  Normally, this is
 *   just after the setup() method is called, however, the serial
 *   console may operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int bcm_attach(struct uart_dev_s *dev)
{
  /* Nothing to do here... this is all handled in bcm_aux.c */

  return OK;
}

/****************************************************************************
 * Name: bcm_detach
 *
 * Description:
 *   Detach Mini-UART interrupts.  This method is called when the serial port
 *   is closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void bcm_detach(struct uart_dev_s *dev)
{
  struct bcm_dev_s *priv = (struct bcm_dev_s *)dev->priv;

  /* There is not much to do here.  Interrupt related logic is handled in
   * handled in bcm_aux.c.
   */

  /* Disable Mini-UART interrupts */

  bcm_restoreuartint(priv, 0);
}

/****************************************************************************
 * Name: bcm_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int bcm_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || \
    defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  struct inode      *inode;
  struct uart_dev_s *dev;
  uint8_t regval;
#endif
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  struct bcm_dev_s  *priv;
  bool              iflow = false;
  bool              oflow = false;
#endif
  int               ret = OK;

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || \
    defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->priv != NULL);
#endif

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  priv  = (struct bcm_dev_s *)dev->priv;
#endif

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct bcm_dev_s *user = (struct bcm_dev_s *)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct bcm_dev_s));
          }
      }
      break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        cfsetispeed(termiosp, priv->baud);

        /* Note: CSIZE only supports 5-8 bits. The driver only support 8/9 bit
         * modes and therefore is no way to report 9-bit mode, we always claim
         * 8 bit mode.
         */

        termiosp->c_cflag =
          ((priv->parity != 0) ? PARENB : 0) |
          ((priv->parity == 1) ? PARODD : 0) |
          ((priv->stop2) ? CSTOPB : 0) |
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
          ((priv->oflow) ? CCTS_OFLOW : 0) |
#  endif
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
          ((priv->iflow) ? CRTS_IFLOW : 0) |
#  endif
          CS8;

        /* TODO: CCTS_IFLOW, CCTS_OFLOW */
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        struct uart_config_s config;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Perform some sanity checks before accepting any changes */

        if (((termiosp->c_cflag & CSIZE) != CS8)
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#  endif
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#  endif
           )
          {
            ret = -EINVAL;
            break;
          }

        if (termiosp->c_cflag & PARENB)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        priv->stop2 = (termiosp->c_cflag & CSTOPB) != 0;
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
        priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
        oflow = priv->oflow;
#  endif
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
        priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
        iflow = priv->iflow;
#  endif

        /* Note that since there is no way to request 9-bit mode
         * and no way to support 5/6/7-bit modes, we ignore them
         * all here.
         */

        /* Note that only cfgetispeed is used because we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);

        /* Effect the changes immediately - note that we do not implement
         * TCSADRAIN / TCSAFLUSH
         */

        config.baud      = priv->baud;
        config.parity    = priv->parity;
        config.bits      = priv->bits;
        config.stopbits2 = priv->stop2;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        config.iflow     = priv->iflow;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
        config.oflow     = priv->oflow;
#endif
        bcm_miniuart_configure(&config);
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_BCM2708_MINI_UART_BREAKS
    case TIOCSBRK:
      {
        irqstate_t flags;
        uint8_t lcr;

        /* Send a break signal */

        flags = enter_critical_section();
        lcr   = getreg8(BCM_AUX_MU_LCR);
        lcr  |= BCM_AUX_MU_LCR_BREAK;
        putreg8(lcd, BCM_AUX_MU_LCR);

        /* Disable TX activity */

        bcm_txint(dev, false);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:
      {
        irqstate_t flags;
        uint8_t lcr;

        /* Configure TX back to UART */

        flags = enter_critical_section();
        lcr   = getreg8(BCM_AUX_MU_LCR);
        lcr  &= ~BCM_AUX_MU_LCR_BREAK;
        putreg8(lcd, BCM_AUX_MU_LCR);

        /* Enable further TX activity */

        bcm_txint(dev, true);
        leave_critical_section(flags);
      }
      break;
#endif /* CONFIG_BCM2708_MINI_UART_BREAKS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: bcm_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the Mini-UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int bcm_receive(struct uart_dev_s *dev, uint32_t *status)
{
  /* Revisit... RX status not returned */

  *status = 0;

  /* RX data is available when reading from the I/O register */

  return getreg8(BCM_AUX_MU_IO);
}

/****************************************************************************
 * Name: bcm_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void bcm_rxint(struct uart_dev_s *dev, bool enable)
{
  struct bcm_dev_s *priv = (struct bcm_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx related error occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= BCM_AUX_MU_IER_RXD;
      bcm_setuartint(priv);
#endif
    }
  else
    {
      priv->ier &= ~BCM_AUX_MU_IER_RXD;
      bcm_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bcm_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool bcm_rxavailable(struct uart_dev_s *dev)
{
  /* Return true if there is at least one more by in the RX FIFO.
   * NOTE: This has the side effect of clearing any RX overrun status.
   */

  return (getreg8(BCM_AUX_MU_LSR) & BCM_AUX_MU_LSR_DTREADY) != 0;
}

/****************************************************************************
 * Name: bcm_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input Parameters:
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
static bool bcm_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
#warning Missing logic
  return false;
}
#endif

/****************************************************************************
 * Name: bcm_send
 *
 * Description:
 *   This method will send one byte on the Mini-UART.
 *
 ****************************************************************************/

static void bcm_send(struct uart_dev_s *dev, int ch)
{
  /* Data is sent be writing to the IO register */

  putreg8((uint8_t)ch, BCM_AUX_MU_IO);
}

/****************************************************************************
 * Name: bcm_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void bcm_txint(struct uart_dev_s *dev, bool enable)
{
  struct bcm_dev_s *priv = (struct bcm_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= BCM_AUX_MU_IER_TXD;
      bcm_setuartint(priv);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->ier &= ~BCM_AUX_MU_IER_TXD;
      bcm_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bcm_txready
 *
 * Description:
 *   Return true if the hardware can accept another by for transfer.
 *
 ****************************************************************************/

static bool bcm_txready(struct uart_dev_s *dev)
{
  /* Return true if the TX FIFO can accept at least one more byte.
   * NOTE: This has the side effect of clearing any RX overrun status.
   */

  return (getreg8(BCM_AUX_MU_LSR) & BCM_AUX_MU_LSR_TXEMPTY) != 0;
}

/****************************************************************************
 * Name: bcm_txempty
 *
 * Description:
 *   Return true if the transmit FIFO is empty
 *
 ****************************************************************************/

static bool bcm_txempty(struct uart_dev_s *dev)
{
  /* Return true if the TX FIFO is empty.
   * NOTE: This has the side effect of clearing any RX overrun status.
   */

  return (getreg8(BCM_AUX_MU_LSR) & BCM_AUX_MU_LSR_TXIDLE) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_mu_interrupt
 *
 * Description:
 *   This is the Mini-UART status interrupt handler.  It will be invoked by
 *   logic in bcm_aux.c when the common AUX interrupt interupt is received
 *   and a pending Mini-UART interrupt is pending.
 *
 *   Then interrupt handler should call uart_transmitchars() or
 *   uart_receivechar() to perform the appropriate data transfers.\
 *
 ****************************************************************************/

int bcm_mu_interrupt(int irq, void *context, void *arg)
{
  uint8_t iir;

  /* Loop while there are pending interrupts */

  while (((iir = getreg8(BCM_AUX_MU_IIR)) & BCM_AUX_MU_IIR_PEND) != 0)
    {
      switch (iir & BCM_AUX_MU_IIR_MASK)
        {
          case BCM_AUX_MU_IIR_NONE:     /* No interrupts */
          default:                      /* Shouldn't happen */
            return OK;

          case BCM_AUX_MU_IIR_TXEMPTY:  /* TX FIFO empty (read) */
            uart_xmitchars(&g_miniuart_port);
            break;

          case BCM_AUX_MU_IIR_RXDATA:   /* Data in RX FIFO (read) */
            uart_recvchars(&g_miniuart_port);
            break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bcm_miniuart_earlyserialinit
 *
 * Description:
 *   Performs the low level Mini-UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in bcm_lowsetup() and main clock initialization
 *   performed in up_clkinitialize().
 *
 ****************************************************************************/

void bcm_miniuart_earlyserialinit(void)
{
  /* Disable interrupts from all Mini-UARTS.  The console is enabled in
   * bcm_setup()
   */

  bcm_restoreuartint(g_miniuart_port.priv, 0);

#ifdef CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE
  /* Configure the Mini-USAR console */

  g_miniuart_port.isconsole = true;
  bcm_setup(&g_miniuart_port);
#endif
}

/****************************************************************************
 * Name: bcm_miniuart_serialinit
 *
 * Description:
 *   Register the Mini-UART serial console and serial port.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bcm_miniuart_serialinit(void)
{
/* Register Mini-UART the console */

#ifdef CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE
  (void)uart_register("/dev/console", &g_miniuart_port);
#endif

  /* Register the  Mini-UART as /dev/ttyS0 */

  (void)uart_register("/dev/ttyS0", &g_miniuart_port);
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
#ifdef CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE
  struct bcm_dev_s *priv = (struct bcm_dev_s *)g_miniuart_port.priv;
  uint8_t ier;

  bcm_disableuartint(priv, &ier);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      bcm_lowputc('\r');
    }

  bcm_lowputc(ch);
  bcm_restoreuartint(priv, ier);
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
#ifdef CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      bcm_lowputc('\r');
    }

  bcm_lowputc(ch);
#endif

  return ch;
}

#endif /* USE_SERIALDRIVER */
#endif /* CONFIG_BCM2708_MINI_UART */
