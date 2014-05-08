/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_serial.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <arch/serial.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#include "chip.h"
#include "chip/nuc_uart.h"
#include "nuc_lowputc.h"
#include "nuc_serial.h"

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

struct nuc_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t ier;       /* Saved IER value */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (5-8) */
  uint8_t  depth;     /* RX/TX FIFO depth */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
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

#ifdef CONFIG_NUC_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_NUC_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_NUC_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

/* This describes the state of the LPC17xx uart0 port. */

#ifdef CONFIG_NUC_UART0
static struct nuc_dev_s g_uart0priv =
{
  .uartbase       = NUC_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = NUC_IRQ_UART0,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .depth          = (UART0_FIFO_DEPTH-1),
  .stopbits2      = CONFIG_UART0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif /* CONFIG_NUC_UART0 */

/* This describes the state of the LPC17xx uart1 port. */

#ifdef CONFIG_NUC_UART1
static struct nuc_dev_s g_uart1priv =
{
  .uartbase       = NUC_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = NUC_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .depth          = (UART1_FIFO_DEPTH-1),
  .stopbits2      = CONFIG_UART1_2STOP,
};

static uart_dev_t g_uart1port =
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
#endif /* CONFIG_NUC_UART1 */

/* This describes the state of the LPC17xx uart1 port. */

#ifdef CONFIG_NUC_UART2
static struct nuc_dev_s g_uart2priv =
{
  .uartbase       = NUC_UART2_BASE,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = NUC_IRQ_UART2,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
  .depth          = (UART2_FIFO_DEPTH-1),
  .stopbits2      = CONFIG_UART2_2STOP,
};

static uart_dev_t g_uart2port =
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
#endif /* CONFIG_NUC_UART2 */

/* Which UART with be tty0/console and which tty1? tty2? The console, if it
 * exists, will always be ttyS0.  If there is no console then will use the
 * lowest numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART0-2 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port /* UART0 is console */
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port /* UART1 is console */
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart2port /* UART2 is console */
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_NUC_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_NUC_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_NUC_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any two of UART0-2 excluding the console UART. */

#if defined(CONFIG_NUC_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_NUC_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_NUC_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART1-2. It can't be UART0 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-2 could also be the
 * console.
 */

#if defined(CONFIG_NUC_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_NUC_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct nuc_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct nuc_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_setier
 ****************************************************************************/

static uint32_t up_setier(struct nuc_dev_s *priv,
                          uint32_t clrbits, uint32_t setbits)
{
  irqstate_t flags;
  uint32_t retval;

  /* Make sure that this is atomic */

  flags = irqsave();

  /* Get the current IER setting */

  retval = priv->ier;

  /* Modify and write the IER according to the inputs */

  priv->ier &= ~clrbits;
  priv->ier |= setbits;
  up_serialout(priv, NUC_UART_IER_OFFSET, priv->ier);
  irqrestore(flags);

  /* Return the value of the IER before modification */

  return retval;
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct nuc_dev_s *priv, uint32_t *ier)
{
  *ier = up_setier(priv, UART_IER_ALLIE, 0);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct nuc_dev_s *priv, uint32_t ier)
{
  uint32_t setbits = ier & UART_IER_ALLIE;
  uint32_t clrbits = (~ier) & UART_IER_ALLIE;
  (void)up_setier(priv, clrbits, setbits);
}

/****************************************************************************
 * Name: up_rxto_disable
 ****************************************************************************/

static void up_rxto_disable(struct nuc_dev_s *priv)
{
  uint32_t regval;

  /* This function is called at initialization time and also when a timeout
   * interrupt is received when the RX FIFO is empty.
   *
   * Set Rx Trigger Level so that an interrupt will be generated when the
   * very next byte is received.
   */

  regval = up_serialin(priv, NUC_UART_FCR_OFFSET);
  regval &= ~UART_FCR_RFITL_MASK;
  regval |= UART_FCR_RFITL_1;
  up_serialout(priv, NUC_UART_FCR_OFFSET, regval);

  /* Disable the RX timeout interrupt and disable the timeout */

  (void)up_setier(priv, (UART_IER_RTO_IEN | UART_IER_TIME_OUT_EN), 0);
}

/****************************************************************************
 * Name: up_rxto_enable
 ****************************************************************************/

static void up_rxto_enable(struct nuc_dev_s *priv)
{
  uint32_t regval;

  /* This function is called after each RX interrupt.  Data has been received
   * and more may or may not be received.
   *
   * Set the RX FIFO level so that interrupts are only received when there
   * are 8 or 14 bytes in the FIFO (depending on the UART FIFO depth).
   */

  regval = up_serialin(priv, NUC_UART_FCR_OFFSET);
  regval &= ~UART_FCR_RFITL_MASK;
#if defined(CONFIG_NUC_UART0)
#  if defined(CONFIG_NUC_UART0) || defined(CONFIG_NUC_UART0)
  regval |= priv->depth > 16 ? UART_FCR_RFITL_14 : UART_FCR_RFITL_8;
#  else
  regval |= UART_FCR_RFITL_14;
#  endif
#else
  regval |= UART_FCR_RFITL_8;
#endif
  up_serialout(priv, NUC_UART_FCR_OFFSET, regval);

  /* Enable the RX timeout interrupt and enable the timeout */

  (void)up_setier(priv, 0, (UART_IER_RTO_IEN | UART_IER_TIME_OUT_EN));
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  uint32_t regval;

  /* Reset the TX FIFO */

  regval = up_serialin(priv, NUC_UART_FCR_OFFSET);
  up_serialout(priv, NUC_UART_FCR_OFFSET, regval | UART_FCR_TFR);

  /* Reset the RX FIFO */

  up_serialout(priv, NUC_UART_FCR_OFFSET, regval | UART_FCR_RFR);

  /* Set Rx Trigger Level */

  regval &= ~(UART_FCR_RFITL_MASK | UART_FCR_TFR | UART_FCR_RFR);
  regval |= UART_FCR_RFITL_1;
  up_serialout(priv, NUC_UART_FCR_OFFSET, regval);

  /* Set Parity & Data bits and Stop bits */

  regval = 0;
  switch (priv->bits)
    {
    case 5:
      regval |= UART_LCR_WLS_5;
      break;

    case 6:
      regval |= UART_LCR_WLS_6;
      break;

    case 7:
      regval |= UART_LCR_WLS_7;
      break;

    default:
    case 8:
      regval |= UART_LCR_WLS_8;
      break;
    }

  switch (priv->parity)
    {
    default:
    case 0:
      break;

    case 1:
      regval |= UART_LCR_PBE;
      break;

    case 2:
      regval |= (UART_LCR_PBE | UART_LCR_EPE);
      break;
    }

  if (priv->stopbits2)
    {
      regval |= UART_LCR_NSB;
    }

  up_serialout(priv, NUC_UART_LCR_OFFSET, regval);

  /* Configure the RX timeout, but do not enable the interrupt yet */

  regval = UART_TOR_TOIC(60) |  UART_TOR_DLY(0);
  up_serialout(priv, NUC_UART_TOR_OFFSET, regval);

  /* Set the baud */

  nuc_setbaud(priv->uartbase, priv->baud);

  /* Set up the IER */

  priv->ier = up_serialin(priv, NUC_UART_IER_OFFSET);

  /* Enable Flow Control in the Modem Control Register */
  /* Not implemented */

#endif /* CONFIG_SUPPRESS_UART_CONFIG */
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  up_disableuartint(priv, NULL);
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
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
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
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
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
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct nuc_dev_s   *priv;
  uint32_t           isr;
  uint32_t           regval;
  int                passes;
  bool               rxto;
  bool               rxfe;

#ifdef CONFIG_NUC_UART0
  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_NUC_UART1
  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else
#endif
#ifdef CONFIG_NUC_UART2
  if (g_uart2priv.irq == irq)
    {
      dev = &g_uart2port;
    }
  else
#endif
    {
      PANIC();
    }
  priv = (struct nuc_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART interrupt status register (ISR) contents */

       isr = up_serialin(priv, NUC_UART_ISR_OFFSET);

      /* Check if the RX FIFO is empty.  Check if an RX timeout occur.  These affect
       * some later decisions.
       */

      rxfe = ((up_serialin(priv, NUC_UART_FSR_OFFSET) & UART_FSR_RX_EMPTY) != 0);
      rxto = ((isr & UART_ISR_TOUT_INT) != 0);

      /* Check if the RX FIFO is filled to the threshold value OR if the RX
       * timeout occurred with the FIFO non-empty.  Both are cleared
       * by reading from the RBR register.
       */

      if ((isr & UART_ISR_RDA_INT) != 0 || (rxto && !rxfe))
        {
          uart_recvchars(dev);
        }

      /* Enable or disable RX timeouts based on the state of RX FIFO:
       *
       * DISABLE: If the timeout occurred and the RX FIFO was empty.
       * ENABLE: Data was in RX FIFO (may have been removed), RX interrupts
       *   are enabled, and the timeout is not already enabled.
       */

      if (rxto && rxfe)
        {
          /* A timeout interrupt occurred while the RX FIFO is empty.
           * We need to read from the RBR to clear the interrupt.
           */

          (void)up_serialin(priv, NUC_UART_RBR_OFFSET);

          /* Disable, further RX timeout interrupts and set the RX FIFO
           * threshold so that an interrupt will be generated when the
           * very next byte is recieved.
           */

          up_rxto_disable(priv);
        }

      /* Is the timeout enabled?  Are RX interrupts enabled?  Was there
       * data in the RX FIFO when we entered the interrupt handler?
       */

      else if ((priv->ier & (UART_IER_RTO_IEN|UART_IER_RDA_IEN)) == UART_IER_RDA_IEN && !rxfe)
        {
          /* We are receiving data and the RX timeout is not enabled.
           * Set the RX FIFO threshold so that RX interrupts will only be
           * generated after several bytes have been recevied and enable
           * the RX timout.
           */

          up_rxto_enable(priv);
        }

      /* Check if the transmit holding register is empty.  Cleared by writing
       * to the THR register.
       */

      if ((isr & UART_ISR_THRE_INT) != 0)
        {
          uart_xmitchars(dev);
        }

      /* Check for modem status. */

      if ((isr & UART_ISR_MODEM_INT) != 0)
        {
          /* Cleared by setting the DCTSF bit in the modem control register (MCR) */

         regval = up_serialin(priv, NUC_UART_MCR_OFFSET);
         up_serialout(priv, NUC_UART_MCR_OFFSET, regval | UART_MSR_DCTSF);
        }

      /* Check for line status or buffer errors*/

      if ((isr & UART_ISR_RLS_INT) != 0 ||
          (isr & UART_ISR_BUF_ERR_INT) != 0)
        {
          /* Both errors are cleared by reseting the RX FIFO */

          regval = up_serialin(priv, NUC_UART_FCR_OFFSET);
          up_serialout(priv, NUC_UART_FCR_OFFSET, regval | UART_FCR_RFR);
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
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct nuc_dev_s   *priv  = (struct nuc_dev_s*)dev->priv;
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct nuc_dev_s *user = (struct nuc_dev_s*)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, priv, sizeof(struct nuc_dev_s));
          }
      }
      break;
#endif

    case TIOCSBRK:   /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:   /* BSD compatibility: Turn break off, unconditionally */
      ret = -ENOTTY; /* Not supported */
      break;

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios*)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Other termios fields are not yet returned.
         * Note that cfsetospeed is not necessary because we have
         * knowledge that only one speed is supported.
         * Both cfset(i|o)speed() translate to cfsetspeed.
         */

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios*)arg;
        uint32_t           lcr;  /* Holds current values of line control register */
        uint16_t           dl;   /* Divisor latch */

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Handle other termios settings.
         * Note that only cfgetispeed is used because we have knowledge
         * that only one speed is supported.
         */

        /* Get the c_speed field in the termios struct */

        priv->baud = cfgetispeed(termiosp);

        /* Reset the baud */

        nuc_setbaud(priv->base, priv->baud);
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  uint32_t rbr;

  *status = up_serialin(priv, NUC_UART_FSR_OFFSET);
  rbr     = up_serialin(priv, NUC_UART_RBR_OFFSET);
  return rbr;
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable receive data, line status and buffer error interrupts */

      irqstate_t flags = irqsave();
      (void)up_setier(priv, 0,
                      (UART_IER_RDA_IEN | UART_IER_RLS_IEN |
                       UART_IER_BUF_ERR_IEN));

      /* Enable or disable timeouts based on the state of RX FIFO */

      if ((up_serialin(priv, NUC_UART_FSR_OFFSET) & UART_FSR_RX_EMPTY) != 0)
        {
          /* The FIFO is empty.  Disable RX timeout interrupts and set the
           * RX FIFO threshold so that an interrupt will be generated when
           * the very next byte is recieved.
           */

          up_rxto_disable(priv);
        }
      else
        {
          /* Otherwise, set the RX FIFO threshold so that RX interrupts will
           * only be generated after several bytes have been recevied and
           * enable* the RX timout.
           */

          up_rxto_enable(priv);
        }

    irqrestore(flags);
#endif
    }
  else
    {
      /* Enable receive data, line status,  buffer error, and RX timeout
       * interrupts.  Also disables the RX timer.
       */

      (void)up_setier(priv, 0,
                      (UART_IER_RDA_IEN | UART_IER_RLS_IEN | UART_IER_RTO_IEN |
                       UART_IER_BUF_ERR_IEN | UART_IER_TIME_OUT_EN));
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  return ((up_serialin(priv, NUC_UART_FSR_OFFSET) & UART_FSR_RX_EMPTY) == 0);
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  up_serialout(priv, NUC_UART_THR_OFFSET, (uint32_t)ch);
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the THR empty interrupt */

      irqstate_t flags = irqsave();
      (void)up_setier(priv, 0, UART_IER_THRE_IEN);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
      irqrestore(flags);
#endif
    }
  else
    {
      /* Disable the THR empty interrupt */

      (void)up_setier(priv, UART_IER_THRE_IEN, 0);
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  uint32_t regval;
  int depth;

  regval = up_serialin(priv, NUC_UART_FSR_OFFSET);
  depth  = (regval & UART_FSR_TX_POINTER_MASK) >> UART_FSR_TX_POINTER_SHIFT;
  return depth < priv->depth;
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)dev->priv;
  return ((up_serialin(priv, NUC_UART_FSR_OFFSET) & UART_FSR_TE_FLAG) != 0);
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

void up_earlyserialinit(void)
{
  /* Configuration whichever UART is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

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
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
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
  struct nuc_dev_s *priv = (struct nuc_dev_s*)CONSOLE_DEV.priv;
  uint32_t ier;
  up_disableuartint(priv, &ier);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      nuc_lowputc((uint32_t)'\r');
    }

  nuc_lowputc((uint32_t)ch);
#ifdef HAVE_CONSOLE
  up_restoreuartint(priv, ier);
#endif

  return ch;
}

#else /* USE_SERIALDRIVER && HAVE_UART */

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

      nuc_lowputc((uint32_t)'\r');
    }

  nuc_lowputc((uint32_t)ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER && HAVE_UART */
