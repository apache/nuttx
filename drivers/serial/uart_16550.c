/****************************************************************************
 * drivers/serial/uart_16550.c
 * Serial driver for 16550 UART
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <nuttx/uart_16550.h>

#include <arch/board/board.h>

#ifdef CONFIG_UART_16550

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_16550_s
{
  uart_addrwidth_t uartbase;  /* Base address of UART registers */
  uint32_t         baud;      /* Configured baud */
  uint32_t         uartclk;   /* UART clock frequency */
  uart_datawidth_t ier;       /* Saved IER value */
  uint8_t          irq;       /* IRQ associated with this UART */
  uint8_t          parity;    /* 0=none, 1=odd, 2=even */
  uint8_t          bits;      /* Number of bits (7 or 8) */
  bool             stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  uart_setup(struct uart_dev_s *dev);
static void uart_shutdown(struct uart_dev_s *dev);
static int  uart_attach(struct uart_dev_s *dev);
static void uart_detach(struct uart_dev_s *dev);
static int  uart_interrupt(int irq, void *context);
static int  uart_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  uart_receive(struct uart_dev_s *dev, uint32_t *status);
static void uart_rxint(struct uart_dev_s *dev, bool enable);
static bool uart_rxavailable(struct uart_dev_s *dev);
static void uart_send(struct uart_dev_s *dev, int ch);
static void uart_txint(struct uart_dev_s *dev, bool enable);
static bool uart_txready(struct uart_dev_s *dev);
static bool uart_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  .setup          = uart_setup,
  .shutdown       = uart_shutdown,
  .attach         = uart_attach,
  .detach         = uart_detach,
  .ioctl          = uart_ioctl,
  .receive        = uart_receive,
  .rxint          = uart_rxint,
  .rxavailable    = uart_rxavailable,
  .send           = uart_send,
  .txint          = uart_txint,
  .txready        = uart_txready,
  .txempty        = uart_txempty,
};

/* I/O buffers */

#ifdef CONFIG_16550_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART2
static char g_uart2rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART3
static char g_uart3rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the LPC17xx uart0 port. */

#ifdef CONFIG_16550_UART0
static struct uart_16550_s g_uart0priv =
{
  .uartbase       = CONFIG_16550_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .uartclk        = CONFIG_16550_UART0_CLOCK,
  .irq            = CONFIG_16550_UART0_IRQ,
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

#ifdef CONFIG_16550_UART1
static struct uart_16550_s g_uart1priv =
{
  .uartbase       = CONFIG_16550_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .uartclk        = CONFIG_16550_UART1_CLOCK,
  .irq            = CONFIG_16550_UART1_IRQ,
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

#ifdef CONFIG_16550_UART2
static struct uart_16550_s g_uart2priv =
{
  .uartbase       = CONFIG_16550_UART2_BASE,
  .baud           = CONFIG_UART2_BAUD,
  .uartclk        = CONFIG_16550_UART2_CLOCK,
  .irq            = CONFIG_16550_UART2_IRQ,
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

#ifdef CONFIG_16550_UART3
static struct uart_16550_s g_uart3priv =
{
  .uartbase       = CONFIG_16550_UART3_BASE,
  .baud           = CONFIG_UART3_BAUD,
  .uartclk        = CONFIG_16550_UART3_CLOCK,
  .irq            = CONFIG_16550_UART3_IRQ,
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
#  ifdef CONFIG_16550_UART1
#    define TTYS1_DEV     g_uart1port      /* UART0=ttyS0;UART1=ttyS1 */
#    ifdef CONFIG_16550_UART2
#      define TTYS2_DEV   g_uart2port      /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2 */
#      ifdef CONFIG_16550_UART3
#        define TTYS3_DEV g_uart3port      /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#      else
#        undef TTYS3_DEV                   /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_16550_UART3
#        define TTYS2_DEV g_uart3port     /* UART0=ttyS0;UART1=ttyS1;UART3=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* UART0=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_16550_UART2
#      define TTYS1_DEV   g_uart2port    /* UART0=ttyS0;UART2=ttyS1;No ttyS3 */
#      ifdef CONFIG_16550_UART3
#        define TTYS2_DEV g_uart3port    /* UART0=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#      else
#        undef TTYS2_DEV                 /* UART0=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                   /* No ttyS3 */
#    else
#      ifdef CONFIG_16550_UART3
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
#  ifdef CONFIG_16550_UART
#    define TTYS1_DEV     g_uart0port     /* UART1=ttyS0;UART0=ttyS1 */
#    ifdef CONFIG_16550_UART2
#      define TTYS2_DEV   g_uart2port     /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2 */
#      ifdef CONFIG_16550_UART3
#        define TTYS3_DEV g_uart3port     /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#      else
#        undef TTYS3_DEV                  /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_16550_UART3
#        define TTYS2_DEV g_uart3port     /* UART1=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* UART1=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_16550_UART2
#      define TTYS1_DEV   g_uart2port     /* UART1=ttyS0;UART2=ttyS1 */
#      ifdef CONFIG_16550_UART3
#        define TTYS2_DEV g_uart3port     /* UART1=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* UART1=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    else
#      ifdef CONFIG_16550_UART3
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
#  ifdef CONFIG_16550_UART
#    define TTYS1_DEV     g_uart0port     /* UART2=ttyS0;UART0=ttyS1 */
#    ifdef CONFIG_16550_UART1
#      define TTYS2_DEV   g_uart1port     /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#      ifdef CONFIG_16550_UART3
#        define TTYS3_DEV g_uart3port     /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2;UART3=ttyS3 */
#      else
#        undef TTYS3_DEV                  /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_16550_UART3
#        define TTYS2_DEV g_uart3port     /* UART2=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* UART2=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_16550_UART1
#      define TTYS1_DEV   g_uart1port    /* UART2=ttyS0;UART1=ttyS1 */
#      ifdef CONFIG_16550_UART3
#        define TTYS2_DEV g_uart3port    /* UART2=ttyS0;UART1=ttyS1;UART3=ttyS2 */
#      else
#        undef TTYS2_DEV                 /* UART2=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                   /* No ttyS3 */
#    else
#      ifdef CONFIG_16550_UART3
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
#  ifdef CONFIG_16550_UART
#    define TTYS1_DEV     g_uart0port    /* UART3=ttyS0;UART0=ttyS1 */
#    ifdef CONFIG_16550_UART1
#      define TTYS2_DEV   g_uart1port    /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#      ifdef CONFIG_16550_UART2
#        define TTYS3_DEV g_uart2port    /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2;UART2=ttyS3 */
#      else
#        undef TTYS3_DEV                 /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_16550_UART2
#        define TTYS2_DEV g_uart2port    /* UART3=ttyS0;UART0=ttyS1;UART2=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                 /* UART3=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#      endif
#        undef TTYS3_DEV                 /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_16550_UART1
#      define TTYS1_DEV   g_uart1port    /* UART3=ttyS0;UART1=ttyS1 */
#      ifdef CONFIG_16550_UART2
#        define TTYS2_DEV g_uart2port    /* UART3=ttyS0;UART1=ttyS1;UART2=ttyS2;No ttyS3 */
#      else
#        undef TTYS2_DEV                 /* UART3=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                   /* No ttyS3 */
#    else
#      ifdef CONFIG_16550_UART2
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

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/****************************************************************************
 * Name: uart_serialin
 ****************************************************************************/

static inline uart_datawidth_t uart_serialin(struct uart_16550_s *priv, int offset)
{
  return uart_getreg(priv->uartbase, offset);
}

/****************************************************************************
 * Name: uart_serialout
 ****************************************************************************/

static inline void uart_serialout(struct uart_16550_s *priv, int offset, uart_datawidth_t value)
{
  uart_putreg(priv->uartbase, offset, value);
}

/****************************************************************************
 * Name: uart_disableuartint
 ****************************************************************************/

static inline void uart_disableuartint(struct uart_16550_s *priv, uart_datawidth_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLIE;
    }

  priv->ier &= ~UART_IER_ALLIE;
  uart_serialout(priv, UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: uart_restoreuartint
 ****************************************************************************/

static inline void uart_restoreuartint(struct uart_16550_s *priv, uint32_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  uart_serialout(priv, UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: uart_enablebreaks
 ****************************************************************************/

static inline void uart_enablebreaks(struct uart_16550_s *priv, bool enable)
{
  uint32_t lcr = uart_serialin(priv, UART_LCR_OFFSET);
  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }
  uart_serialout(priv, UART_LCR_OFFSET, lcr);
}

/************************************************************************************
 * Name: uart_divisor
 *
 * Descrption:
 *   Select a divider to produce the BAUD from the UART_CLK.
 *
 *     BAUD = UART_CLK / (16 * DL), or
 *     DIV  = UART_CLK / BAUD / 16
 *
 *   Ignoring the fractional divider for now.
 *
 ************************************************************************************/

static inline uint32_t uart_divisor(struct uart_16550_s *priv)
{
  return (priv->uartclk + (priv->baud << 3)) / (priv->baurd << 4);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int uart_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_16550_SUPRESS_CONFIG
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  uint16_t div;
  uint32_t lcr;

  /* Clear fifos */

  uart_serialout(priv, UART_FCR_OFFSET, (UART_FCR_RXRST|UART_FCR_TXRST));

  /* Set trigger */

  uart_serialout(priv, UART_FCR_OFFSET, (UART_FCR_FIFOEN|UART_FCR_RXTRIGGER_8));

  /* Set up the IER */

  priv->ier = uart_serialin(priv, UART_IER_OFFSET);

  /* Set up the LCR */

  lcr = 0;
  switch (priv->bits)
    {
      case 5 :
        lcr |= UART_LCR_WLS_7BIT;
        break;

      case 6 :
        lcr |= UART_LCR_WLS_7BIT;
        break;

      case 7 :
        lcr |= UART_LCR_WLS_7BIT;
        break;

      default:
      case 8 :
        lcr |= UART_LCR_WLS_7BIT;
        break;
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STB;
    }

  if (priv->parity == 1)
    {
      lcr |= UART_LCR_PEN;
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PEN|UART_LCR_EPS);
    }

  /* Enter DLAB=1 */

  uart_serialout(priv, UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  /* Set the BAUD divisor */

  div = uart_divisor(priv);
  uart_serialout(priv, UART_DLM_OFFSET, div >> 8);
  uart_serialout(priv, UART_DLL_OFFSET, div & 0xff);

  /* Clear DLAB */

  uart_serialout(priv, UART_LCR_OFFSET, lcr);

  /* Configure the FIFOs */

  uart_serialout(priv, UART_FCR_OFFSET,
               (UART_FCR_RXTRIGGER_8|UART_FCR_TXRST|UART_FCR_RXRST|UART_FCR_FIFOEN));
#endif
  return OK;
}

/****************************************************************************
 * Name: uart_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void uart_shutdown(struct uart_dev_s *dev)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  uart_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: uart_attach
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

static int uart_attach(struct uart_dev_s *dev)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, uart_interrupt);
#ifndef CONFIG_ARCH_NOINTC
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the UART
        */

       uart_enable_irq(priv->irq);
    }
endif
  return ret;
}

/****************************************************************************
 * Name: uart_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void uart_detach(struct uart_dev_s *dev)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
#ifndef CONFIG_ARCH_NOINTC
  uart_disable_irq(priv->irq);
#endif
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_16550_s structure in order to call these functions.
 *
 ****************************************************************************/

static int uart_interrupt(int irq, void *context)
{
  struct uart_dev_s   *dev = NULL;
  struct uart_16550_s *priv;
  uint32_t             status;
  int                  passes;

#ifdef CONFIG_16550_UART0
  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_16550_UART1
  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else
#endif
#ifdef CONFIG_16550_UART2
  if (g_uart2priv.irq == irq)
    {
      dev = &g_uart2port;
    }
  else
#endif
#ifdef CONFIG_16550_UART3
  if (g_uart3priv.irq == irq)
    {
      dev = &g_uart3port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct uart_16550_s*)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

       status = uart_serialin(priv, UART_IIR_OFFSET);

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

              status = uart_serialin(priv, UART_MSR_OFFSET);
              vdbg("MSR: %02x\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_INTID_RLS:
            {
              /* Read the line status register (LSR) to clear */

              status = uart_serialin(priv, UART_LSR_OFFSET);
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
 * Name: uart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode        *inode = filep->f_inode;
  struct uart_dev_s   *dev   = inode->i_private;
  struct uart_16550_s *priv  = (struct uart_16550_s*)dev->priv;
  int                  ret    = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
         struct uart_16550_s *user = (struct uart_16550_s*)arg;
         if (!user)
           {
             *get_errno_ptr() = EINVAL;
             ret = ERROR;
           }
         else
           {
             memcpy(user, dev, sizeof(struct uart_16550_s));
           }
       }
       break;

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = irqsave();
        uart_enablebreaks(priv, true);
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        uart_enablebreaks(priv, false);
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
 * Name: uart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int uart_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  uint32_t rbr;

  *status = uart_serialin(priv, UART_LSR_OFFSET);
  rbr     = uart_serialin(priv, UART_RBR_OFFSET);
  return rbr;
}

/****************************************************************************
 * Name: uart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void uart_rxint(struct uart_dev_s *dev, bool enable)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_ERBFI;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_ERBFI;
    }
  uart_serialout(priv, UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: uart_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool uart_rxavailable(struct uart_dev_s *dev)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  return ((uart_serialin(priv, UART_LSR_OFFSET) & UART_LSR_RDR) != 0);
}

/****************************************************************************
 * Name: uart_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void uart_send(struct uart_dev_s *dev, int ch)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  uart_serialout(priv, UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: uart_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void uart_txint(struct uart_dev_s *dev, bool enable)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_ETBEI;
      uart_serialout(priv, UART_IER_OFFSET, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_ETBEI;
      uart_serialout(priv, UART_IER_OFFSET, priv->ier);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: uart_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool uart_txready(struct uart_dev_s *dev)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  return ((uart_serialin(priv, UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Name: uart_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool uart_txempty(struct uart_dev_s *dev)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)dev->priv;
  return ((uart_serialin(priv, UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before uart_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by uart_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

void uart_earlyserialinit(void)
{
  /* Configure all UARTs (except the CONSOLE UART) and disable interrupts */

#ifdef CONFIG_16550_UART0
  uart_disableuartint(&g_uart0priv, NULL);
#endif
#ifdef CONFIG_16550_UART1
  uart_disableuartint(&g_uart1priv, NULL);
#endif
  uart_disableuartint(&g_uart2priv, NULL);
#endif
  uart_disableuartint(&g_uart3priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  uart_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: uart_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   uart_earlyserialinit was called previously.
 *
 ****************************************************************************/

void uart_serialinit(void)
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
 * Name: uart_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

#ifdef HAVE_16550_CONSOLE
int uart_putc(int ch)
{
  struct uart_16550_s *priv = (struct uart_16550_s*)CONSOLE_DEV.priv;
  uart_datawidth_t ier;

  uart_disableuartint(priv, &ier);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      uart_lowputc('\r');
    }

  uart_lowputc(ch);
  uart_restoreuartint(priv, ier);
  return ch;
}
#endif

#endif /* CONFIG_UART_16550 */
