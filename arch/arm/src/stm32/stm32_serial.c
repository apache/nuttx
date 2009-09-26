/****************************************************************************
 * arch/arm/src/stm32/stm32_serial.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial.h>
#include <arch/serial.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there a USART enabled? */

#if defined(CONFIG_USART1_DISABLE) && defined(CONFIG_USART2_DISABLE) && defined(CONFIG_USART3_DISABLE)
#  error "No USARTs enabled"
#endif

/* Is there a serial console? */

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && !defined(CONFIG_USART1_DISABLE)
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && !defined(CONFIG_USART2_DISABLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && !defined(CONFIG_USART3_DISABLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  warning "No valid CONFIG_USARTn_SERIAL_CONSOLE Setting"
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef CONFIG_USE_SERIALDRIVER

/* Which USART with be tty0/console and which tty1? */

#if defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_USART1port     /* USART1 is console */
#  define TTYS0_DEV       g_USART1port     /* USART1 is ttyS0 */
#  ifndef CONFIG_USART2_DISABLE
#    define TTYS1_DEV     g_USART2port     /* USART2 is ttyS1 */
#    ifndef CONFIG_USART3_DISABLE
#      define TTYS2_DEV   g_USART3port     /* USART3 is ttyS2 */
#    else
#      undef TTYS1_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifndef CONFIG_USART3_DISABLE
#      define TTYS1_DEV   g_USART3port     /* USART3 is ttyS1 */
#    else
#      undef TTYS1_DEV                     /* No ttyS1 */
#    endif
#    undef TTYS2_DEV                       /* No ttyS2 */
#  endif
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_USART2port     /* USART2 is console */
#  define TTYS0_DEV       g_USART2port     /* USART2 is ttyS0 */
#  ifndef CONFIG_USART1_DISABLE
#    define TTYS1_DEV     g_USART1port     /* USART1 is ttyS1 */
#    ifndef CONFIG_USART3_DISABLE
#      define TTYS2_DEV   g_USART3port     /* USART3 is ttyS2 */
#    else
#      undef TTYS1_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifndef CONFIG_USART3_DISABLE
#      define TTYS1_DEV   g_USART3port     /* USART3 is ttyS1 */
#    else
#      undef TTYS1_DEV                     /* No ttyS1 */
#    endif
#    undef TTYS2_DEV                       /* No ttyS2 */
#  endif
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_USART3port     /* USART3 is console */
#  define TTYS0_DEV       g_USART3port     /* USART3 is ttyS0 */
#  ifndef CONFIG_USART1_DISABLE
#    define TTYS1_DEV     g_USART1port     /* USART1 is ttyS1 */
#    ifndef CONFIG_USART2_DISABLE
#      define TTYS2_DEV   g_USART2port     /* USART2 is ttyS2 */
#    else
#      undef TTYS1_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifndef CONFIG_USART2_DISABLE
#      define TTYS1_DEV   g_USART2port     /* USART2 is ttyS1 */
#    else
#      undef TTYS1_DEV                     /* No ttyS1 */
#    endif
#    undef TTYS2_DEV                       /* No ttyS2 */
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32  USARTbase; /* Base address of USART registers */
  uint32  baud;      /* Configured baud */
  uint32  im;        /* Saved IM value */
  ubyte   irq;       /* IRQ associated with this USART */
  ubyte   parity;    /* 0=none, 1=odd, 2=even */
  ubyte   bits;      /* Number of bits (7 or 8) */
  boolean stopbits2; /* TRUE: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     up_setup(struct USART_dev_s *dev);
static void    up_shutdown(struct USART_dev_s *dev);
static int     up_attach(struct USART_dev_s *dev);
static void    up_detach(struct USART_dev_s *dev);
static int     up_interrupt(int irq, void *context);
static int     up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int     up_receive(struct USART_dev_s *dev, uint32 *status);
static void    up_rxint(struct USART_dev_s *dev, boolean enable);
static boolean up_rxavailable(struct USART_dev_s *dev);
static void    up_send(struct USART_dev_s *dev, int ch);
static void    up_txint(struct USART_dev_s *dev, boolean enable);
static boolean up_txready(struct USART_dev_s *dev);
static boolean up_txempty(struct USART_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct USART_ops_s g_USART_ops =
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
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifndef CONFIG_USART1_DISABLE
static char g_USART1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_USART1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifndef CONFIG_USART2_DISABLE
static char g_USART2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_USART2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifndef CONFIG_USART3_DISABLE
static char g_USART3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_USART3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif

/* This describes the state of the STM32 USART1 ports. */

#ifndef CONFIG_USART1_DISABLE
static struct up_dev_s g_USART1priv =
{
  .USARTbase      = STM32_USART1_BASE,
  .baud           = CONFIG_USART1_BAUD,
  .irq            = STM32_IRQ_USART1,
  .parity         = CONFIG_USART1_PARITY,
  .bits           = CONFIG_USART1_BITS,
  .stopbits2      = CONFIG_USART1_2STOP,
};

static USART_dev_t g_USART1port =
{
  .recv     =
  {
    .size   = CONFIG_USART1_RXBUFSIZE,
    .buffer = g_USART1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART1_TXBUFSIZE,
    .buffer = g_USART1txbuffer,
  },
  .ops      = &g_USART_ops,
  .priv     = &g_USART1priv,
};
#endif

/* This describes the state of the STM32 USART2 port. */

#ifndef CONFIG_USART2_DISABLE
static struct up_dev_s g_USART2priv =
{
  .USARTbase      = STM32_USART2_BASE,
  .baud           = CONFIG_USART2_BAUD,
  .irq            = STM32_IRQ_USART2,
  .parity         = CONFIG_USART2_PARITY,
  .bits           = CONFIG_USART2_BITS,
  .stopbits2      = CONFIG_USART2_2STOP,
};

static USART_dev_t g_USART2port =
{
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_USART2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_USART2txbuffer,
   },
  .ops      = &g_USART_ops,
  .priv     = &g_USART2priv,
};
#endif

/* This describes the state of the STM32 USART3 port. */

#ifndef CONFIG_USART3_DISABLE
static struct up_dev_s g_USART3priv =
{
  .USARTbase      = STM32_USART3_BASE,
  .baud           = CONFIG_USART3_BAUD,
  .irq            = STM32_IRQ_USART3,
  .parity         = CONFIG_USART3_PARITY,
  .bits           = CONFIG_USART3_BITS,
  .stopbits2      = CONFIG_USART3_2STOP,
};

static USART_dev_t g_USART3port =
{
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_USART3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_USART3txbuffer,
   },
  .ops      = &g_USART_ops,
  .priv     = &g_USART3priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32 up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->USARTbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint32 value)
{
  putreg32(value, priv->USARTbase + offset);
}

/****************************************************************************
 * Name: up_disableusartint
 ****************************************************************************/

static inline void up_disableusartint(struct up_dev_s *priv, uint32 *im)
{
  /* Return the current interrupt mask value */

  if (im)
    {
      *im = priv->im;
    }

  /* Disable all interrupts */

  priv->im = 0;
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static inline void up_restoreusartint(struct up_dev_s *priv, uint32 im)
{
  priv->im = im;
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct USART_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Note:  The logic here depends on the fact that that the USART module
   * was enabled and the GPIOs were configured in up_lowsetup().
   */

  /* Disable the USART */

  /* Calculate BAUD rate from the SYS clock */

  /* Set the USART to interrupt whenever the TX FIFO is almost empty or when
   * any character is received.
   */

  /* Flush the Rx and Tx FIFOs */

  /* Enable Rx interrupts from the USART except for Tx interrupts.  We don't want
   * Tx interrupts until we have something to send.  We will check for serial
   * errors as part of Rx interrupt processing (no interrupts will be received
   * yet because the interrupt is still disabled at the interrupt controller.
   */

  /* Enable the FIFOs */

#ifdef CONFIG_SUPPRESS_USART_CONFIG
#endif

  /* Enable Rx, Tx, and the USART */

#ifdef CONFIG_SUPPRESS_USART_CONFIG
#endif

  /* Set up the cache IM value */

//  priv->im = 
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct USART_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disableusartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct USART_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
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
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct USART_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite USART_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct USART_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  int                passes;
  boolean            handled;

#ifndef CONFIG_USART1_DISABLE
  if (g_USART1priv.irq == irq)
    {
      dev = &g_USART1port;
    }
  else
#endif
#ifndef CONFIG_USART2_DISABLE
  if (g_USART2priv.irq == irq)
    {
      dev = &g_USART2port;
    }
  else
#endif
#ifndef CONFIG_USART3_DISABLE
  if (g_USART3priv.irq == irq)
    {
      dev = &g_USART3port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = TRUE;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = FALSE;

      /* Get the masked USART status and clear the pending interrupts. */

      /* Handle incoming, receive bytes (with or without timeout) */

        {
           /* Rx buffer not empty ... process incoming bytes */

           uart_recvchars(dev);
           handled = TRUE;
        }

      /* Handle outgoing, transmit bytes */

        {
           /* Tx FIFO not full ... process outgoing bytes */

           uart_xmitchars(dev);
           handled = TRUE;
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
  struct USART_dev_s *dev   = inode->i_private;
  int                ret    = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s*)arg;
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
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct USART_dev_s *dev, uint32 *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32 rxd;

  /* Get the Rx byte plux error information.  Return those in status */

  *status = rxd;

  /* Then return the actual received byte */

  return rxd & 0xff;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct USART_dev_s *dev, boolean enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx FIFO (or an Rx
       * timeout occurs.
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
//      priv->im |= ;
#endif
    }
  else
    {
//      priv->im &= ~();
    }
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return TRUE if the receive fifo is not empty
 *
 ****************************************************************************/

static boolean up_rxavailable(struct USART_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return 0;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void up_send(struct USART_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct USART_dev_s *dev, boolean enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX fifo is half emptied */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
//      priv->im |= ;
      up_serialout(priv, stm32_USART_IM_OFFSET, priv->im);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note that the may recurse and that priv->im
       * may be altered.
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

//      priv->im &= ~();
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return TRUE if the tranmsit fifo is not full
 *
 ****************************************************************************/

static boolean up_txready(struct USART_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return 0;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return TRUE if the transmit fifo is empty
 *
 ****************************************************************************/

static boolean up_txempty(struct USART_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level USART initialization early in 
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the USARTs was performed in
   * up_lowsetup
   */

  /* Disable all USARTS */

  up_disableusartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableusartint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableusartint(TTYS2_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_CONSOLE
  CONSOLE_DEV.isconsole = TRUE;
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

#ifdef HAVE_CONSOLE
  (void)usart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */

  (void)usart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)usart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  (void)usart_register("/dev/ttyS2", &TTYS2_DEV);
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
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint32 im;

  up_disableusartint(priv, &im);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  up_restoreusartint(priv, im);
#endif
  return ch;
}

#else /* CONFIG_USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* CONFIG_USE_SERIALDRIVER */
