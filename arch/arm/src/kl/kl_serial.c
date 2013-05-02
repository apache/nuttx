/****************************************************************************
 * arch/mips/src/kl/kl_serial.c
 *
 *   Copyright (C) 2013-2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

#include "kl_config.h"
#include "kl_lowputc.h"
#include "chip.h"
#include "kl_gpio.h"
#include "chip/kl_uart.h"


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

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1-4?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART0-5 */

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
#  if defined(CONFIG_KL_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_KL_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_KL_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-5 excluding the console UART. */

#if defined(CONFIG_KL_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_KL_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_KL_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART1-5. It can't be UART0 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-5 could also be the
 * console.
 */

#if defined(CONFIG_KL_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_KL_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART2-5. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART 2-5 could also be the console.
 */

#if defined(CONFIG_KL_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS3_DEV           g_uart2port /* UART2 is ttyS3 */
#  define UART2_ASSIGNED      1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase;  /* Base address of UART registers */
  uint32_t  baud;      /* Configured baud */
  uint32_t  clock;     /* Clocking frequency of the UART module */
  uint8_t   irq;       /* IRQ associated with this UART (for enable) */
  uint8_t   ie;        /* Interrupts enabled */
  uint8_t   parity;    /* 0=none, 1=odd, 2=even */
  uint8_t   bits;      /* Number of bits (8 or 9) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupts(int irq, void *context);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

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
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txready,
};

/* I/O buffers */

#ifdef CONFIG_KL_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_KL_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_KL_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

/* This describes the state of the Kinetis UART0 port. */

#ifdef CONFIG_KL_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = KL_UART0_BASE,
  .clock          = BOARD_CORECLK_FREQ,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = KL_IRQ_UART0,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
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

/* This describes the state of the Kinetis UART1 port. */

#ifdef CONFIG_KL_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = KL_UART1_BASE,
  .clock          = BOARD_BUSCLK_FREQ,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = KL_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
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

/* This describes the state of the Kinetis UART2 port. */

#ifdef CONFIG_KL_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = KL_UART2_BASE,
  .clock          = BOARD_BUSCLK_FREQ,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = KL_IRQ_UART2,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
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
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint8_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg8(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint8_t value)
{
  putreg8(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_setuartint
 ****************************************************************************/

static void up_setuartint(struct up_dev_s *priv)
{
  irqstate_t flags;
  uint8_t regval;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in ie */

  flags    = irqsave();
  regval   = up_serialin(priv, KL_UART_C2_OFFSET);
  regval  &= ~UART_C2_ALLINTS;
  regval  |= priv->ie;
  up_serialout(priv, KL_UART_C2_OFFSET, regval);
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static void up_restoreuartint(struct up_dev_s *priv, uint8_t ie)
{
  irqstate_t flags;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in ie */

  flags    = irqsave();
  priv->ie = ie & UART_C2_ALLINTS;
  up_setuartint(priv);
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static void up_disableuartint(struct up_dev_s *priv, uint8_t *ie)
{
  irqstate_t flags;

  flags = irqsave();
  if (ie)
   {
     *ie = priv->ie;
   }

  up_restoreuartint(priv, 0);
  irqrestore(flags);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Configure the UART as an RS-232 UART */

  kl_uartconfigure(priv->uartbase, priv->baud, priv->clock,
                        priv->parity, priv->bits);
#endif

  /* Make sure that all interrupts are disabled */

  up_restoreuartint(priv, 0);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Disable interrupts */

  up_restoreuartint(priv, 0);

  /* Reset hardware and disable Rx and Tx */

  kl_uartreset(priv->uartbase);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(priv->irq, up_interrupts);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Disable interrupts */

  up_restoreuartint(priv, 0);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupts
 *
 * Description:
 *   This is the UART status interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupts(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  int                passes;
  uint8_t            s1;
  bool               handled;

#ifdef CONFIG_KL_UART0
  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_KL_UART1
  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else
#endif
#ifdef CONFIG_KL_UART2
  if (g_uart2priv.irq == irq)
    {
      dev = &g_uart2port;
    }
  else
#endif
    {
      PANIC();
    }
  priv = (struct up_dev_s*)dev->priv;
  DEBUGASSERT(priv);

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Read status register 1 */

      s1 = up_serialin(priv, KL_UART_S1_OFFSET);

      /* Check if the receive data register is full (RDRF).  NOTE:  If
       * FIFOS are enabled, this does not mean that the the FIFO is full,
       * rather, it means that the the number of bytes in the RX FIFO has
       * exceeded the watermark setting.  There may actually be RX data
       * available!
       *
       * The RDRF status indication is cleared when the data is read from
       * the RX data register.
       */

      if ((s1 & UART_S1_RDRF) != 0)
        {
          /* Process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes */

      /* Check if the transmit data register is "empty."  NOTE:  If FIFOS
       * are enabled, this does not mean that the the FIFO is empty, rather,
       * it means that the the number of bytes in the TX FIFO is below the
       * watermark setting.  There could actually be space for additional TX
       * data.
       *
       * The TDRE status indication is cleared when the data is written to
       * the TX data register.
       */

      if ((s1 & UART_S1_TDRE) != 0)
        {
          /* Process outgoing bytes */

          uart_xmitchars(dev);
          handled = true;
        }

      /* Handle error interrupts.  This interrupt may be caused by:
       *
       * FE: Framing error. To clear FE, write a logic one to the FE flag.
       * NF: Noise flag. To clear NF, write logic one to the NF.
       * PF: Parity error flag. To clear PF, write a logic one to the PF.
       * OR: Receiver Overrun Flag.  To clear OR, write a logic 1 to the OR flag.
       */

       if ((s1 & UART_S1_ERRORS) != 0)
         {
           up_serialout(priv, KL_UART_S1_OFFSET, (s1 & UART_S1_ERRORS));
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
#if 0 /* Reserved for future growth */
  struct inode      *inode;
  struct uart_dev_s *dev;
  struct up_dev_s   *priv;
  int                ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv)
  priv = (struct up_dev_s*)dev->priv;

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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint8_t s1;

  /* Get error status information:
   *
   * FE: Framing error. To clear FE, read S1 with FE set and then read
   *     read UART data register (D).
   * NF: Noise flag. To clear NF, read S1 and then read the UART data
   *     register (D).
   * PF: Parity error flag. To clear PF, read S1 and then read the UART
   *     data register (D).
   */

  s1 = up_serialin(priv, KL_UART_S1_OFFSET);

  /* Return status information */

  if (status)
    {
      *status = (uint32_t)s1;
    }

  /* Then return the actual received byte.  Reading S1 then D clears all
   * RX errors.
   */

  return (int)up_serialin(priv, KL_UART_D_OFFSET);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= UART_C2_RIE;
      up_setuartint(priv);
#endif
    }
  else
    {
      priv->ie |= UART_C2_RIE;
      up_setuartint(priv);
    }

  irqrestore(flags);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Return true if the receive data register is full (RDRF).  NOTE:  If
   * FIFOS are enabled, this does not mean that the the FIFO is full,
   * rather, it means that the the number of bytes in the RX FIFO has
   * exceeded the watermark setting.  There may actually be RX data
   * available!
   */

  return (up_serialin(priv, KL_UART_S1_OFFSET) & UART_S1_RDRF) != 0;
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, KL_UART_D_OFFSET, (uint8_t)ch);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= UART_C2_TIE;
      up_setuartint(priv);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->ie &= ~UART_C2_TIE;
      up_setuartint(priv);
    }

  irqrestore(flags);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Return true if the transmit data register is "empty."  NOTE:  If
   * FIFOS are enabled, this does not mean that the the FIFO is empty,
   * rather, it means that the the number of bytes in the TX FIFO is
   * below the watermark setting.  There may actually be space for
   * additional TX data.
   */

  return (up_serialin(priv, KL_UART_S1_OFFSET) & UART_S1_TDRE) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock iniialization
 *   performed in up_clkinitialize().
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Disable interrupts from all UARTS.  The console is enabled in
   * pic32mx_consoleinit()
   */

  up_restoreuartint(TTYS0_DEV.priv, 0);
#ifdef TTYS1_DEV
  up_restoreuartint(TTYS1_DEV.priv, 0);
#endif
#ifdef TTYS2_DEV
  up_restoreuartint(TTYS2_DEV.priv, 0);
#endif
#ifdef TTYS3_DEV
  up_restoreuartint(TTYS3_DEV.priv, 0);
#endif
#ifdef TTYS4_DEV
  up_restoreuartint(TTYS4_DEV.priv, 0);
#endif
#ifdef TTYS5_DEV
  up_restoreuartint(TTYS5_DEV.priv, 0);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  (void)uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  (void)uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif
#ifdef TTYS5_DEV
  (void)uart_register("/dev/ttyS5", &TTYS5_DEV);
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
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint8_t ie;

  up_disableuartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      kl_lowputc('\r');
    }

  kl_lowputc(ch);
  up_restoreuartint(priv, ie);
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
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      kl_lowputc('\r');
    }

  kl_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */

