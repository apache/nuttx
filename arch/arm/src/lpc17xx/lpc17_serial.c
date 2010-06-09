/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_serial.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
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
#include "os_internal.h"
#include "up_internal.h"

#include "lpc17_uart.h"
#include "lpc17_serial.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(CONFIG_USE_SERIALDRIVER) && defined(HAVE_UART)

/* Configuration *********************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint8_t  ier;       /* Saved IER value */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  uint8_t  cclkdiv;   /* Divisor needed to get PCLK from CCLK */
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

struct uart_ops_s g_uart_ops =
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
  .txempty    = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_LPC17_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC17_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC17_UART2
static char g_uart2rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC17_UART3
static char g_uart3rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the LPC17xx uart0 port. */

#ifdef CONFIG_LPC17_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = LPC17_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = LPC17_IRQ_UART0,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
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
#endif

/* This describes the state of the LPC17xx uart1 port. */

#ifdef CONFIG_LPC17_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = LPC17_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = LPC17_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
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
#endif

/* This describes the state of the LPC17xx uart1 port. */

#ifdef CONFIG_LPC17_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = LPC17_UART2_BASE,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = LPC17_IRQ_UART2,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
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
#endif

/* This describes the state of the LPC17xx uart1 port. */

#ifdef CONFIG_LPC17_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = LPC17_UART3_BASE,
  .baud           = CONFIG_UART3_BAUD,
  .irq            = LPC17_IRQ_UART3,
  .parity         = CONFIG_UART3_PARITY,
  .bits           = CONFIG_UART3_BITS,
  .stopbits2      = CONFIG_UART3_2STOP,
};

static uart_dev_t g_uart3port =
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

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart0port      /* UART0=console */
#  define TTYS0_DEV       g_uart0port      /* UART0=ttyS0 */
#  ifdef CONFIG_LPC17_UART1
#    define TTYS1_DEV     g_uart1port      /* UART0=ttyS0;UART1=ttyS1 */
#    ifdef CONFIG_LPC17_UART2
#      define TTYS2_DEV   g_uart2port      /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2 */
#      ifdef CONFIG_LPC17_UART3
#        define TTYS3_DEV g_uart3port      /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#      else
         undef TTYS3_DEV                   /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_UART3
#        define TTYS2_DEV g_uart3port     /* UART0=ttyS0;UART1=ttyS1;UART3=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* UART0=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_LPC17_UART2
#      define TTYS1_DEV   g_uart2port    /* UART0=ttyS0;UART2=ttyS1;No ttyS3 */
#      ifdef CONFIG_LPC17_UART3
#        define TTYS2_DEV g_uart3port    /* UART0=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#      else
#        undef TTYS2_DEV                 /* UART0=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                   /* No ttyS3 */
#    else
#      ifdef CONFIG_LPC17_UART3
#        define TTYS1_DEV g_uart3port    /* UART0=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#      else
#        undef TTYS1_DEV                 /* UART0=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#      endif
#        undef TTYS2_DEV                  /* No ttyS2 */
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  endif
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart1port     /* UART1=console */
#  define TTYS0_DEV       g_uart1port     /* UART1=ttyS0 */
#  ifdef CONFIG_LPC17_UART
#    define TTYS1_DEV     g_uart0port     /* UART1=ttyS0;UART0=ttyS1 */
#    ifdef CONFIG_LPC17_UART2
#      define TTYS2_DEV   g_uart2port     /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2 */
#      ifdef CONFIG_LPC17_UART3
#        define TTYS3_DEV g_uart3port     /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#      else
#        undef TTYS3_DEV                  /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_UART3
#        define TTYS2_DEV g_uart3port     /* UART1=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* UART1=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_LPC17_UART2
#      define TTYS1_DEV   g_uart2port     /* UART1=ttyS0;UART2=ttyS1 */
#      ifdef CONFIG_LPC17_UART3
#        define TTYS2_DEV g_uart3port     /* UART1=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* UART1=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    else
#      ifdef CONFIG_LPC17_UART3
#        define TTYS1_DEV   g_uart3port   /* UART1=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#      else
#        undef TTYS1_DEV                  /* UART1=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS2_DEV                    /* No ttyS2 */
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  endif
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart2port     /* UART2=console */
#  define TTYS0_DEV       g_uart2port     /* UART2=ttyS0 */
#  ifdef CONFIG_LPC17_UART
#    define TTYS1_DEV     g_uart0port     /* UART2=ttyS0;UART0=ttyS1 */
#    ifdef CONFIG_LPC17_UART1
#      define TTYS2_DEV   g_uart1port     /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#      ifdef CONFIG_LPC17_UART3
#        define TTYS3_DEV g_uart3port     /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2;UART3=ttyS3 */
#      else
#        undef TTYS3_DEV                  /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_UART3
#        define TTYS2_DEV g_uart3port     /* UART2=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* UART2=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_LPC17_UART1
#      define TTYS1_DEV   g_uart1port    /* UART2=ttyS0;UART1=ttyS1 */
#      ifdef CONFIG_LPC17_UART3
#        define TTYS2_DEV g_uart3port    /* UART2=ttyS0;UART1=ttyS1;UART3=ttyS2 */
#      else
#        undef TTYS2_DEV                 /* UART2=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                   /* No ttyS3 */
#    else
#      ifdef CONFIG_LPC17_UART3
#        define TTYS1_DEV g_uart3port    /* UART2=ttyS0;UART3=ttyS1;No ttyS3 */
#      else
#        undef TTYS1_DEV                 /* UART2=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS2_DEV                   /* No ttyS2 */
#      undef TTYS3_DEV                   /* No ttyS3 */
#    endif
#  endif
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart3port    /* UART3=console */
#  define TTYS0_DEV       g_uart3port    /* UART3=ttyS0 */
#  ifdef CONFIG_LPC17_UART
#    define TTYS1_DEV     g_uart0port    /* UART3=ttyS0;UART0=ttyS1 */
#    ifdef CONFIG_LPC17_UART1
#      define TTYS2_DEV   g_uart1port    /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#      ifdef CONFIG_LPC17_UART2
#        define TTYS3_DEV g_uart2port    /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2;UART2=ttyS3 */
#      else
#        undef TTYS3_DEV                 /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_UART2
#        define TTYS2_DEV g_uart2port    /* UART3=ttyS0;UART0=ttyS1;UART2=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                 /* UART3=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#      endif
#        undef TTYS3_DEV                 /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_LPC17_UART1
#      define TTYS1_DEV   g_uart1port    /* UART3=ttyS0;UART1=ttyS1 */
#      ifdef CONFIG_LPC17_UART2
#        define TTYS2_DEV g_uart2port    /* UART3=ttyS0;UART1=ttyS1;UART2=ttyS2;No ttyS3 */
#      else
#        undef TTYS2_DEV                 /* UART3=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                   /* No ttyS3 */
#    else
#      ifdef CONFIG_LPC17_UART2
#        define TTYS1_DEV   g_uart2port  /* UART3=ttyS0;UART2=ttyS1;No ttyS3;No ttyS3 */
#        undef TTYS3_DEV                 /* UART3=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#      else
#        undef TTYS1_DEV                 /* UART3=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS2_DEV                   /* No ttyS2 */
#      undef TTYS3_DEV                   /* No ttyS3 */
#    endif
#  endif
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
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint8_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLIE;
    }

  priv->ier &= ~UART_IER_ALLIE;
  up_serialout(priv, LPC17_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint8_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  up_serialout(priv, LPC17_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint8_t lcr = up_serialin(priv, LPC17_UART_LCR_OFFSET);
  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }
  up_serialout(priv, LPC17_UART_LCR_OFFSET, lcr);
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
#ifndef CONFIG_SUPPRESS_LPC17_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint16_t dl;
  uint8_t lcr;

  /* Clear fifos */

  up_serialout(priv, LPC17_UART_FCR_OFFSET, (UART_FCR_RXRST|UART_FCR_TXRST));

  /* Set trigger */

  up_serialout(priv, LPC17_UART_FCR_OFFSET, (UART_FCR_FIFOEN|UART_FCR_RXTRIGGER_8));

  /* Set up the IER */

  priv->ier = up_serialin(priv, LPC17_UART_IER_OFFSET);

  /* Set up the LCR */

  lcr = 0;

  if (priv->bits == 7)
    {
      lcr |= UART_LCR_WLS_7BIT;
    }
  else
    {
      lcr |= UART_LCR_WLS_8BIT;
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STOP;
    }

  if (priv->parity == 1)
    {
      lcr |= (UART_LCR_PE|UART_LCR_PS_ODD);
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PE|UART_LCR_PS_EVEN);
    }

  /* Enter DLAB=1 */

  up_serialout(priv, LPC17_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  /* Set the BAUD divisor */

  dl = lpc17_uartdl(priv->baud, priv->cclkdiv);
  up_serialout(priv, LPC17_UART_DLM_OFFSET, dl >> 8);
  up_serialout(priv, LPC17_UART_DLL_OFFSET, dl & 0xff);

  /* Clear DLAB */

  up_serialout(priv, LPC17_UART_LCR_OFFSET, lcr);

  /* Configure the FIFOs */

  up_serialout(priv, LPC17_UART_FCR_OFFSET,
               (UART_FCR_RXTRIGGER_8|UART_FCR_TXRST|UART_FCR_RXRST|UART_FCR_FIFOEN));

  /* The NuttX serial driver waits for the first THRE interrrupt before
   * sending serial data... However, it appears that the lpc17xx hardware
   * does not generate that interrupt until a transition from not-empty
   * to empty.  So, the current kludge here is to send one NULL at
   * startup to kick things off.
   */

  up_serialout(priv, LPC17_UART_THR_OFFSET, '\0');
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  uint8_t            status;
  int                passes;

  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

       status = up_serialin(priv, LPC17_UART_IIR_OFFSET);

      /* The UART_IIR_INTSTATUS bit should be zero if there are pending
       * interrupts
       */

      if ((status & UART_IIR_INTSTATUS) != 0)
        {
          /* Break out of the loop when there is no longer a
           * pending interrupt
           */

          break;
        }

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_INTID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_INTID_RDA:
          case UART_IIR_INTID_CTI:
            {
              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_INTID_THRE:
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts (UART1 only) */

          case UART_IIR_INTID_MSI:
            {
              /* Read the modem status register (MSR) to clear */

              status = up_serialin(priv, LPC17_UART_MSR_OFFSET);
              vdbg("MSR: %02x\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_INTID_RLS:
            {
              /* Read the line status register (LSR) to clear */

              status = up_serialin(priv, LPC17_UART_LSR_OFFSET);
              vdbg("LSR: %02x\n", status);
              break;
            }

          /* There should be no other values */

          default:
            {
              dbg("Unexpected IIR: %02x\n", status);
              break;
            }
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
  struct up_dev_s   *priv  = (struct up_dev_s*)dev->priv;
  int                ret    = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s*)arg;
         if (!user)
           {
             *get_errno_ptr() = EINVAL;
             ret = ERROR;
           }
         else
           {
             memcpy(user, dev, sizeof(struct up_dev_s));
           }
       }
       break;

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = irqsave();
        up_enablebreaks(priv, true);
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        up_enablebreaks(priv, false);
        irqrestore(flags);
      }
      break;

    default:
      *get_errno_ptr() = ENOTTY;
      ret = ERROR;
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint8_t rbr;

  *status = up_serialin(priv, LPC17_UART_LSR_OFFSET);
  rbr     = up_serialin(priv, LPC17_UART_RBR_OFFSET);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_RBRIE;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_RBRIE;
    }
  up_serialout(priv, LPC17_UART_IER_OFFSET, priv->ier);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LPC17_UART_LSR_OFFSET) & UART_LSR_RDR) != 0);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, LPC17_UART_THR_OFFSET, (uint8_t)ch);
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
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_THREIE;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_THREIE;
    }
  up_serialout(priv, LPC17_UART_IER_OFFSET, priv->ier);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LPC17_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LPC17_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
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
 *   NOTE: Power, clocking, and pin configuration was performed in
 *   up_lowsetup() very, early in the boot sequence.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Disable all UARTS */

#ifdef TTYS0_DEV
  TTYS0_DEV.cclkdiv = lpc17_uartcclkdiv(CONFIG_UART0_BAUD);
  up_disableuartint(TTYS0_DEV.priv, NULL);
#endif
#ifdef TTYS1_DEV
  TTYS1_DEV.cclkdiv = lpc17_uartcclkdiv(CONFIG_UART1_BAUD);
  up_disableuartint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  TTYS2_DEV.cclkdiv = lpc17_uartcclkdiv(CONFIG_UART2_BAUD);
  up_disableuartint(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  TTYS3_DEV.cclkdiv = lpc17_uartcclkdiv(CONFIG_UART3_BAUD);
  up_disableuartint(TTYS3_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

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
#ifdef TTYS3_DEV
  (void)uart_register("/dev/ttyS3", &TTYS3_DEV);
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
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint8_t ier;

  up_disableuartint(priv, &ier);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  up_restoreuartint(priv, ier);
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
#ifdef HAVE_UART
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
