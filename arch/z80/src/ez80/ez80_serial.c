/****************************************************************************
 * arch/z80/src/ez08/ez80_serial.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#include "chip/chip.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef CONFIG_USE_SERIALDRIVER

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define BASE_BAUD     115200

/* The system clock frequency is defined in the linkcmd file */

extern unsigned long SYS_CLK_FREQ;
#define _DEFCLK ((unsigned long)&SYS_CLK_FREQ)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ez80_dev_s
{
  unsigned int         uartbase;	/* Base address of UART registers */
  unsigned int         baud;		/* Configured baud */
  ubyte                irq;		/* IRQ associated with this UART */
  ubyte                parity;		/* 0=none, 1=odd, 2=even */
  ubyte                bits;		/* Number of bits (7 or 8) */
  boolean              stopbits2;	/* TRUE: Configure with 2 (vs 1) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ez80_setup(struct uart_dev_s *dev);
static void    ez80_shutdown(struct uart_dev_s *dev);
static int     ez80_attach(struct uart_dev_s *dev);
static void    ez80_detach(struct uart_dev_s *dev);
static int     ez80_interrrupt(int irq, void *context);
static int     ez80_ioctl(struct file *filep, int cmd, unsigned long arg);
static int     ez80_receive(struct uart_dev_s *dev, unsigned int *status);
static void    ez80_rxint(struct uart_dev_s *dev, boolean enable);
static boolean ez80_rxavailable(struct uart_dev_s *dev);
static void    ez80_send(struct uart_dev_s *dev, int ch);
static void    ez80_txint(struct uart_dev_s *dev, boolean enable);
static boolean ez80_txready(struct uart_dev_s *dev);
static boolean ez80_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  ez80_setup,          /* setup */
  ez80_shutdown,       /* shutdown */
  ez80_attach,         /* attach */
  ez80_detach,         /* detach */
  ez80_ioctl,          /* ioctl */
  ez80_receive,        /* receive */
  ez80_rxint,          /* rxint */
  ez80_rxavailable,    /* rxavailable */
  ez80_send,           /* send */
  ez80_txint,          /* txint */
  ez80_txready,        /* txready */
  ez80_txempty         /* txempty */
};

/* I/O buffers */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

/* This describes the state of the UART0 port. */

static struct ez80_dev_s g_uart0priv =
{
  EZ80_UART0_BASE,          /* uartbase */
  CONFIG_UART0_BAUD,        /* baud */
  EZ80_UART0_IRQ,           /* irq */
  CONFIG_UART0_PARITY,      /* parity */
  CONFIG_UART0_BITS,        /* bits */
  CONFIG_UART0_2STOP        /* stopbits2 */
};

static uart_dev_t g_uart0port =
{
  0,                        /* open_count */
  FALSE,                    /* xmitwaiting */
  FALSE,                    /* recvwaiting */
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  TRUE,                     /* isconsole */
#else
  FALSE,                    /* isconsole */
#endif
  { 0 },                    /* closesem */
  { 0 },                    /* xmitsem */
  { 0 },                    /* recvsem */
  {
    { 0 },                  /* xmit.sem */
    0,                      /* xmit.head */
    0,                      /* xmit.tail */
    CONFIG_UART0_TXBUFSIZE, /* xmit.size */
    g_uart0txbuffer,        /* xmit.buffer */
  },
  {
    { 0 },                  /* recv.sem */
    0,                      /* recv.head */
    0,                      /* recv.tail */
    CONFIG_UART0_RXBUFSIZE, /* recv.size */
    g_uart0rxbuffer,        /* recv.buffer */
  },
  &g_uart_ops,              /* ops */
  &g_uart0priv,             /* priv */
};

/* This describes the state of the UART1 port. */

static struct ez80_dev_s g_uart1priv =
{
  EZ80_UART1_BASE,          /* uartbase */
  CONFIG_UART1_BAUD,        /* baud */
  EZ80_UART1_IRQ,           /* irq */
  CONFIG_UART1_PARITY,      /* parity */
  CONFIG_UART1_BITS,        /* bits */
  CONFIG_UART1_2STOP        /* stopbits2 */
};

static uart_dev_t g_uart1port =
{
  0,                        /* open_count */
  FALSE,                    /* xmitwaiting */
  FALSE,                    /* recvwaiting */
#ifdef CONFIG_UART1_SERIAL_CONSOLE
  TRUE,                     /* isconsole */
#else
  FALSE,                    /* isconsole */
#endif
  { 0 },                    /* closesem */
  { 0 },                    /* xmitsem */
  { 0 },                    /* recvsem */
  {
    { 0 },                  /* xmit.sem */
    0,                      /* xmit.head */
    0,                      /* xmit.tail */
    CONFIG_UART1_TXBUFSIZE, /* xmit.size */
    g_uart1txbuffer,        /* xmit.buffer */
  },
  {
    { 0 },                  /* recv.sem */
    0,                      /* recv.head */
    0,                      /* recv.tail */
    CONFIG_UART1_RXBUFSIZE, /* recv.size */
    g_uart0rxbuffer,        /* recv.buffer */
  },
  &g_uart_ops,              /* ops */
  &g_uart1priv,             /* priv */
};

/* Now, which one with be tty0/console and which tty1? */

#ifdef CONFIG_UART0_SERIAL_CONSOLE
# define CONSOLE_DEV     g_uart0port
# define TTYS0_DEV       g_uart0port
# define TTYS1_DEV       g_uart1port
#else
# define CONSOLE_DEV     g_uart1port
# define TTYS0_DEV       g_uart1port
# define TTYS1_DEV       g_uart0port
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_serialin
 ****************************************************************************/

static inline ubyte ez80_serialin(struct ez80_dev_s *priv, uint32 offset)
{
  return getreg8(priv->uartbase + offset);
}

/****************************************************************************
 * Name: ez80_serialout
 ****************************************************************************/

static inline void ez80_serialout(struct ez80_dev_s *priv, ubyte offset, ubyte value)
{
  putreg8(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: ez80_disableuartint
 ****************************************************************************/

static inline void ez80_disableuartint(struct ez80_dev_s *priv)
{
  ubyte ier = ez80_serialin(EZ80_UART_IER);
  ier &= ~EZ80_UARTEIR_INTMASK;
  ez80_serialout(priv, EZ80_UART_IER, ier);
}

/****************************************************************************
 * Name: ez80_restoreuartint
 ****************************************************************************/

static inline void ez80_restoreuartint(struct ez80_dev_s *priv, ubyte bits)
{
  ubyte ier = ez80_serialin(EZ80_UART_IER);
  ier |= bits & (EZ80_UARTEIR_TIE|EZ80_UARTEIR_RIE);
  ez80_serialout(priv, EZ80_UART_IER, ier);
}

/****************************************************************************
 * Name: ez80_waittxready
 ****************************************************************************/

static inline void ez80_waittxready(struct ez80_dev_s *priv)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if ((ez80_serialin(priv, EZ80_UART_LSR) & EZ80_UARTLSR_THRE) != 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Name: ez80_setbaud
 ****************************************************************************/

static inline void ez80_setbaud(struct ez80_dev_s *priv, uint24 baud)
{
  uint24 brg_divisor;
  ubyte lctl;

  /* The resulting BAUD and depends on the system clock frequency and the
   * BRG divisor as follows:
   *
   * BAUD = SYSTEM_CLOCK_FREQUENCY / (16 * BRG_Divisor)
   *
   * Or
   *
   * BRG_Divisor = SYSTEM_CLOCK_FREQUENCY / 16 / BAUD
   */

   brg_divisor = ( _DEFCLK + (bard << 3)) / ((baud << 4);

   /* Set the DLAB bit to enable access to the BRG registers */

   lctl = ez80_serialin(priv, EZ80_UART_LCTL);
   lctl |= EZ80_UARTLCTL_DLAB;
   ez80_serialout(priv, EZ80_UART_LCTL, lctl);

   ez80_serialout(priv, EZ80_UART_BRGL, (ubyte)(brg_divisor & 0xff));
   ez80_serialout(priv, EZ80_UART_BRGH, (ubyte)(brg_divisor >> 8));

   lctl &= ~EZ80_UARTLCTL_DLAB;
   ez80_serialout(priv, EZ80_UART_LCTL, lctl);
}

/****************************************************************************
 * Name: ez80_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is called
 *   the first time that the serial port is opened.
 *
 ****************************************************************************/

static int ez80_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct ez80_dev_s *priv = dev->priv;
  ubyte reg;
  ubyte cval;

  if (priv->bits == 7)
    {
      cval = EZ80_UARTCHAR_7BITS;
    }
  else
    {
      cval = EZ80_UARTCHAR_8BITS;
    }

  if (priv->stopbits2)
    {
      cval |= EZ80_UARTLCTl_2STOP;
    }

  if (priv->parity == 1)   /* Odd parity */
    {
      cval |= (EZ80_UARTLCTL_PEN);
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      cval |= (EZ80_UARTLCTL_PEN|EZ80_UARTLCTL_EPS);
    }

  /* Set the baud rate */

  ez80_disableuartint(priv, NULL);
  ez80_setbaud(priv, priv->baud);
  ez80_serial_out(priv, EZ80_UART_MCTL, 0)

  /* Set the character properties */

  reg = (ez80_serialin(priv, EZ80_UART_LCTL) & 0x3f) | cval;
  ez80_serialout(priv, EZ80_UART_LCTL, reg);

  /* Enable and flush the receive FIFO */

  reg = EZ80_UARTFCTL_FIFOEN;
  ez80_serialout(priv, EZ80_UART_FCTL, reg);
  reg |= (EZ80_UARTFCTL_CLRTxF|EZ80_UARTFCTL_CLRRxF);
  ez80_serialout(priv, EZ80_UART_FCTL, reg);

  /* Set the receive trigger level to 4 */

  reg |= EZ80_UARTTRIG_4;
  ez80_serialout(priv, EZ80_UART_FCTL, reg);
#endif
  return OK;
}

/****************************************************************************
 * Name: ez80_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void ez80_shutdown(struct uart_dev_s *dev)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)CONSOLE_DEV.priv;
  ez80_disableuartint(priv);
}

/****************************************************************************
 * Name: ez80_attach
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

static int ez80_attach(struct uart_dev_s *dev)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, ez80_interrrupt);
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
 * Name: ez80_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void ez80_detach(struct uart_dev_s *dev)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: ez80_interrrupt
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

static int ez80_interrrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct ez80_dev_s   *priv;
  volatile uint32    cause;

  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct ez80_dev_s*)dev->priv;

  cause = ez80_serialin(priv, EZ80_UART_IIR) & EZ80_UARTIIR_CAUSEMASK;

  if (cause == (EZ80_UARTINSTS_CTO|EZ80_UARTIIR_INTBIT) ||
      cause == (EZ80_UARTINSTS_RDR|EZ80_UARTIIR_INTBIT))
    {
      /* Receive characters from the RX fifo */

      uart_recvchars(dev);
    }
  else if (cause == (EZ80_UARTINSTS_TC|EZ80_UARTIIR_INTBIT))
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: ez80_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int ez80_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  *get_errno_ptr() = ENOTTY;
  return ERROR;
}

/****************************************************************************
 * Name: ez80_receive
 *
 * Description:
 * Called (usually) from the interrupt level to receive one character from
 * the UART.  Error bits associated with the receipt are provided in the
 * the return 'status'.
 *
 ****************************************************************************/

static int ez80_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  ubyte rbr = ez80_serialin(priv, EZ80_UART_RBR);
  ubyte lsr = ez80_serialin(priv, EZ80_UART_LSR);

  *status = (unsigned int)lsr;
  return rbr;
}

/****************************************************************************
 * Name: ez80_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void ez80_rxint(struct uart_dev_s *dev, boolean enable)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  ubyte ier = ez80_serialin(priv, EZ80_UART_IER);
  
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      ier |= EZ80_UARTEIR_RIE;
      ez80_serialout(priv, EZ80_UART_IER, ier);
#endif
    }
  else
    {
      ier &= ~EZ80_UARTEIR_RIE;
      ez80_serialout(priv, EZ80_UART_IER, ier);
    }
}

/****************************************************************************
 * Name: ez80_rxavailable
 *
 * Description:
 *   Return TRUE if the receive fifo is not empty
 *
 ****************************************************************************/

static boolean ez80_rxavailable(struct uart_dev_s *dev)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  return (ez80_serialin(priv, EZ80_UART_LSR) & EZ80_UARTLSR_DR) != 0;
}

/****************************************************************************
 * Name: ez80_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void ez80_send(struct uart_dev_s *dev, int ch)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  ez80_serialout(priv, EZ80_UART_THR, (ubyte)ch);
}

/****************************************************************************
 * Name: ez80_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void ez80_txint(struct uart_dev_s *dev, boolean enable)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  ubyte ier = ez80_serialin(priv, EZ80_UART_IER);

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      ier |= EZ80_UARTEIR_TIE;
      ez80_serialout(priv, EZ80_UART_IER, ier);
#endif
    }
  else
    {
      ier &= ~EZ80_UARTEIR_TIE;
      ez80_serialout(priv, EZ80_UART_IER, ier);
    }
}

/****************************************************************************
 * Name: ez80_txready
 *
 * Description:
 *   Return TRUE if the tranmsit fifo is not full
 *
 ****************************************************************************/

static boolean ez80_txready(struct uart_dev_s *dev)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  return (ez80_serialin(priv, EZ80_UART_LSR) & EZ80_UARTLSR_THRE) != 0;
}

/****************************************************************************
 * Name: ez80_txempty
 *
 * Description:
 *   Return TRUE if the transmit fifo is empty
 *
 ****************************************************************************/

static boolean ez80_txempty(struct uart_dev_s *dev)
{
  struct ez80_dev_s *priv = (struct ez80_dev_s*)dev->priv;
  return (ez80_serialin(priv, EZ80_UART_LSR) & EZ80_UARTLSR_TEMT) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in 
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  ez80_disableuartint(TTYS0_DEV.priv);
  ez80_disableuartint(TTYS1_DEV.priv);

  CONSOLE_DEV.isconsole = TRUE;
  ez80_setup(&CONSOLE_DEV);
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
  (void)uart_register("/dev/console", &CONSOLE_DEV);
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
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
  struct ez80_dev_s *priv = (struct ez80_dev_s*)CONSOLE_DEV.priv;
  ubyte ier = ez80_serialin(priv, EZ80_UART_IER);

  ez80_disableuartint(priv);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Output CR before LF*/

      ez80_waittxready(priv);
      ez80_serialout(priv, EZ80_UART_THR, '\r');
    }

  /* Output the character */

  ez80_waittxready(priv);
  ez80_serialout(priv, EZ80_UART_THR, (ubyte)ch);

  /* Wait for the character to be sent before re-enabling interrupts */
 
  ez80_waittxready(priv);
  ez80_restoreuartint(priv, ier);
  return ch;
}

#else /* CONFIG_USE_SERIALDRIVER */

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_UART1_SERIAL_CONSOLE
# define ez80_getreg(offs)     getreg8((EZ80_UART1_BASE+(offs)))
# define ez80_putreg(val,offs) putreg8((val), (EZ80_UART1_BASE+(offs)))
#else
# define ez80_getreg(offs)     getreg8((EZ80_UART0_BASE+(offs)))
# define ez80_putreg(val,offs) putreg8((val), (EZ80_UART0_BASE+(offs)))
#endif

#define ez80_txready() ((ez80_getreg(EZ80_UART_LSR) & EZ80_UARTLSR_THRE) != 0)
#define ez80_send(ch)  ez80_putreg(ch, EZ80_UART_THR)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_putc
 ****************************************************************************/

static void ez80_putc(int ch)
{
  int tmp;
  for (tmp = 1000 ; tmp > 0 && !ez80_txready(); tmp--);
  ez80_send(ch);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Output CR before LF */

      ez80_putc('\r');
    }

  /* Output character */

  ez80_putc(ch);
  return ch;
}

#endif /* CONFIG_USE_SERIALDRIVER */
