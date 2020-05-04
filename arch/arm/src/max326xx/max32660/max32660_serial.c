/****************************************************************************
 * arch/arm/src/max326xx/max326_serial.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "chip.h"
#include "max326_config.h"
#include "hardware/max326_uart.h"
#include "max326_clockconfig.h"
#include "max326_lowputc.h"
#include "max326_serial.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/
/* Is there at least one UART enabled and configured as a RS-232 device? */

#ifndef HAVE_UART_DEVICE
#  warning "No UARTs enabled"
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(HAVE_UART_DEVICE) && defined(USE_SERIALDRIVER)

/* Which UART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART0-5 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV        g_uart0port /* UART0 is console */
#    define TTYS0_DEV          g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED     1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV        g_uart1port /* UART1 is console */
#    define TTYS0_DEV          g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED     1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_MAX326XX_UART0)
#    define TTYS0_DEV          g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED     1
#  elif defined(CONFIG_MAX326XX_UART1)
#    define TTYS0_DEV          g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED     1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-1 excluding the console UART. */

#if defined(CONFIG_MAX326XX_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV            g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED       1
#elif defined(CONFIG_MAX326XX_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV            g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED       1
#endif

/* UART events */

#define UART_INT_TX            (UART_INT_TXFIFOLVL | UART_INT_TXFIFOAE)
#define UART_INT_RX            (UART_INT_RXFIFOLVL | UART_INT_RXTO)
#define UART_INT_RXERRORS      (UART_INT_FRAME | UART_INT_PARITY | \
                                UART_INT_RXOVR)

#ifdef CONFIG_DEBUG_FEATURES
#  define UART_INT_ALL         (UART_INT_TX | UART_INT_RX | UART_INT_RXERRORS)
#else
#  define UART_INT_ALL         (UART_INT_TX | UART_INT_RX)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one UART device */

struct max326_dev_s
{
  uintptr_t uartbase;  /* Base address of UART registers */
  uint8_t   irq;       /* IRQ associated with this UART */

  /* UART configuration */

  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  max326_setup(struct uart_dev_s *dev);
static void max326_shutdown(struct uart_dev_s *dev);
static int  max326_attach(struct uart_dev_s *dev);
static void max326_detach(struct uart_dev_s *dev);
static int  max326_interrupt(int irq, void *context, void *arg);
static int  max326_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  max326_receive(struct uart_dev_s *dev, uint32_t *status);
static void max326_rxint(struct uart_dev_s *dev, bool enable);
static bool max326_rxavailable(struct uart_dev_s *dev);
static void max326_send(struct uart_dev_s *dev, int ch);
static void max326_txint(struct uart_dev_s *dev, bool enable);
static bool max326_txready(struct uart_dev_s *dev);
static bool max326_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = max326_setup,
  .shutdown       = max326_shutdown,
  .attach         = max326_attach,
  .detach         = max326_detach,
  .ioctl          = max326_ioctl,
  .receive        = max326_receive,
  .rxint          = max326_rxint,
  .rxavailable    = max326_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = max326_send,
  .txint          = max326_txint,
  .txready        = max326_txready,
  .txempty        = max326_txempty,
};

/* I/O buffers */

#ifdef CONFIG_MAX326XX_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_MAX326XX_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the MAX326xx UART0 port. */

#ifdef CONFIG_MAX326XX_UART0
static struct max326_dev_s g_uart0priv =
{
  .uartbase       = MAX326_UART0_BASE,
  .irq            = MAX326_IRQ_UART0,
  .config         =
  {
    .baud         = CONFIG_UART0_BAUD,
    .parity       = CONFIG_UART0_PARITY,
    .bits         = CONFIG_UART0_BITS,
    .txlevel      = MAX326_UART_TXFIFO_DEPTH / 2,
    .rxlevel      = MAX326_UART_RXFIFO_DEPTH / 4,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rtslevel     = 3 * MAX326_UART_RXFIFO_DEPTH / 4,
#endif
    .stopbits2    = CONFIG_UART0_2STOP,
#ifdef CONFIG_UART0_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_UART0_OFLOWCONTROL
    .oflow        = true,
#endif
  }
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
#endif

/* This describes the state of the MAX326xx UART1 port. */

#ifdef CONFIG_MAX326XX_UART1
static struct max326_dev_s g_uart1priv =
{
  .uartbase       = MAX326_UART1_BASE,
  .irq            = MAX326_IRQ_UART1,
  .config         =
  {
    .baud         = CONFIG_UART1_BAUD,
    .parity       = CONFIG_UART1_PARITY,
    .bits         = CONFIG_UART1_BITS,
    .txlevel      = MAX326_UART_TXFIFO_DEPTH / 2,
    .rxlevel      = MAX326_UART_RXFIFO_DEPTH / 4,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rtslevel     = 3 * MAX326_UART_RXFIFO_DEPTH / 4,
#endif
    .stopbits2    = CONFIG_UART1_2STOP,
#ifdef CONFIG_UART1_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_UART1_OFLOWCONTROL
    .oflow        = true,
#endif
  }
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
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_serialin
 ****************************************************************************/

static inline uint32_t max326_serialin(struct max326_dev_s *priv,
                                       unsigned int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: max326_serialout
 ****************************************************************************/

static inline void max326_serialout(struct max326_dev_s *priv,
                                    unsigned int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: max326_modifyreg
 ****************************************************************************/

static inline void max326_modifyreg(struct max326_dev_s *priv, unsigned int offset,
                                   uint32_t setbits, uint32_t clrbits)
{
  irqstate_t flags;
  uintptr_t regaddr = priv->uartbase + offset;
  uint32_t regval;

  flags   = enter_critical_section();

  regval  = getreg32(regaddr);
  regval &= ~clrbits;
  regval |= setbits;
  putreg32(regval, regaddr);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: max326_int_enable
 ****************************************************************************/

static inline void max326_int_enable(struct max326_dev_s *priv,
                                     uint32_t intset)
{
  irqstate_t flags;
  uint32_t regval;

  flags   = spin_lock_irqsave();
  regval  = max326_serialin(priv, MAX326_UART_INTEN_OFFSET);
  regval |= intset;
  max326_serialout(priv, MAX326_UART_INTEN_OFFSET, regval);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: max326_int_disable
 ****************************************************************************/

static inline void max326_int_disable(struct max326_dev_s *priv,
                                      uint32_t intset)
{
  irqstate_t flags;
  uint32_t regval;

  flags   = spin_lock_irqsave();
  regval  = max326_serialin(priv, MAX326_UART_INTEN_OFFSET);
  regval &= ~intset;
  max326_serialout(priv, MAX326_UART_INTEN_OFFSET, regval);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: max326_int_disableall
 ****************************************************************************/

static void max326_int_disableall(struct max326_dev_s *priv,
                                  uint32_t *intset)
{
  irqstate_t flags;

  flags = spin_lock_irqsave();
  if (intset)
    {
      *intset = max326_serialin(priv, MAX326_UART_INTEN_OFFSET);
    }

  max326_serialout(priv, MAX326_UART_INTEN_OFFSET, 0);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: max326_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int max326_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;

  /* Configure the UART as an RS-232 UART */

  max326_uart_configure(priv->uartbase, &priv->config);
#endif

  /* Make sure that all interrupts are disabled */

  max326_int_disableall(priv, NULL);
  return OK;
}

/****************************************************************************
 * Name: max326_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void max326_shutdown(struct uart_dev_s *dev)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;

  /* Disable interrupts */

  max326_int_disableall(priv, NULL);

  /* Reset hardware and disable Rx and Tx */

  max326_uart_disable(priv->uartbase);
}

/****************************************************************************
 * Name: max326_attach
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

static int max326_attach(struct uart_dev_s *dev)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(priv->irq, max326_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: max326_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void max326_detach(struct uart_dev_s *dev)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;

  /* Disable interrupts */

  max326_int_disableall(priv, NULL);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: max326_interrupt
 *
 * Description:
 *   This is the UART status interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.
 *
 ****************************************************************************/

static int max326_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct max326_dev_s *priv;
  uint32_t intfl;
  uint32_t inten;
  uint32_t stat;
  bool handled;
  int passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct max326_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Read pending interrupt flags, interrupt enables, and UART status
       * registers.
       */

      intfl = max326_serialin(priv, MAX326_UART_INTFL_OFFSET);
      inten = max326_serialin(priv, MAX326_UART_INTEN_OFFSET);
      stat  = max326_serialin(priv, MAX326_UART_STAT_OFFSET);

      /* Clear pending interrupt flags */

      max326_serialout(priv, MAX326_UART_INTFL_OFFSET,
                       intfl & UART_INT_ALL);

      /* Handle incoming, receive bytes.
       * Check if the Rx FIFO level interrupt is enabled and the Rx FIFO is
       * not empty.
       */
#if 0
      if ((intfl & UART_INT_RX) != 0) /* Should work too */
#else
      if ((inten & UART_INT_RX) != 0 && (stat & UART_STAT_RXEMPTY) == 0)
#endif
        {
          /* Process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes.
       * Check if the Tx FIFO level interrupt is enabled and the Tx FIFO is
       * not full.
       */

      if ((inten & UART_INT_TX) != 0 && (stat & UART_STAT_TXFULL) == 0)
        {
          /* Process outgoing bytes */

          uart_xmitchars(dev);
          handled = true;
        }

#ifdef CONFIG_DEBUG_FEATURES
      /* Check for RX error conditions */

      if ((intfl & UART_INT_RXERRORS) != 0)
        {
          /* And now do... what?  Should we reset FIFOs on a FIFO error? */
#warning Missing logic

          handled = true;
        }
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: max326_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int max326_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  struct inode *inode;
  struct uart_dev_s *dev;
  struct max326_dev_s *priv;
  int ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv);
  priv = (struct max326_dev_s *)dev->priv;

  switch (cmd)
    {
    case xxx: /* Add commands here */
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
 * Name: max326_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int max326_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;

  /* Return receiver control information */

  if (status)
    {
      *status = max326_serialin(priv, MAX326_UART_STAT_OFFSET);
    }

  /* Then return the actual received data. */

 return max326_serialin(priv, MAX326_UART_FIFO_OFFSET);
}

/****************************************************************************
 * Name: max326_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void max326_rxint(struct uart_dev_s *dev, bool enable)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifdef CONFIG_DEBUG_FEATURES
      max326_int_enable(priv, UART_INT_RX);
#else
      max326_int_enable(priv, UART_INT_RX + UART_INT_RXERRORS);
#endif
#endif
    }
  else
    {
      max326_int_disable(priv, UART_INT_RX);
    }
}

/****************************************************************************
 * Name: max326_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool max326_rxavailable(struct uart_dev_s *dev)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the receive RXFIFO is not "empty." */

  regval = max326_serialin(priv, MAX326_UART_STAT_OFFSET);
  return (regval & UART_STAT_RXEMPTY) == 0;
}

/****************************************************************************
 * Name: max326_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void max326_send(struct uart_dev_s *dev, int ch)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;
  max326_serialout(priv, MAX326_UART_FIFO_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: max326_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void max326_txint(struct uart_dev_s *dev, bool enable)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      irqstate_t flags;

      /* Enable the TX interrupt */

      flags = enter_critical_section();
      max326_int_enable(priv, UART_INT_TX);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
      leave_critical_section(flags);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      max326_int_disable(priv, UART_INT_TX);
    }
}

/****************************************************************************
 * Name: max326_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool max326_txready(struct uart_dev_s *dev)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the transmit FIFO is "not full." */

  regval = max326_serialin(priv, MAX326_UART_STAT_OFFSET);
  return (regval & UART_STAT_TXFULL) == 0;
}

/****************************************************************************
 * Name: max326_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool max326_txempty(struct uart_dev_s *dev)
{
  struct max326_dev_s *priv = (struct max326_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the transmit FIFO is "empty." */

  regval = max326_serialin(priv, MAX326_UART_STAT_OFFSET);
  return (regval & UART_STAT_TXEMPTY) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before max326_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in xmc_lowsetup() and main clock iniialization
 *   performed in xmc_clock_configure().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void max326_earlyserialinit(void)
{
  /* Disable interrupts from all UARTS.  The console is enabled in
   * pic32mx_consoleinit()
   */

  max326_int_disableall(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  max326_int_disableall(TTYS1_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  max326_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that max326_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef HAVE_UART_CONSOLE
  /* Register the serial console */

  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
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
#ifdef HAVE_UART_CONSOLE
  struct max326_dev_s *priv = (struct max326_dev_s *)CONSOLE_DEV.priv;
  uint32_t intset;

  max326_int_disableall(priv, &intset);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  max326_int_enable(priv, intset);
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
#ifdef HAVE_UART_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  return ch;
}
#endif

#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER */
