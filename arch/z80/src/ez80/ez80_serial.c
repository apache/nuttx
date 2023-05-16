/****************************************************************************
 * arch/z80/src/ez80/ez80_serial.c
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
 * under the License. *
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
#include <nuttx/serial/serial.h>
#include <arch/io.h>

#include "chip.h"
#include "z80_internal.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ez80_dev_s
{
  uint16_t     uartbase;        /* Base address of UART registers */
  unsigned int baud;            /* Configured baud */
  uint8_t      irq;             /* IRQ associated with this UART */
  uint8_t      parity;          /* 0=none, 1=odd, 2=even */
  uint8_t      bits;            /* Number of bits (7 or 8) */
  bool         stopbits2;       /* true: Configure with 2 (vs 1) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  ez80_setup(struct uart_dev_s *dev);
static void ez80_shutdown(struct uart_dev_s *dev);
static int  ez80_attach(struct uart_dev_s *dev);
static void ez80_detach(struct uart_dev_s *dev);
static int  ez80_interrupt(int irq, void *context, void *arg);
static int  ez80_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  ez80_receive(struct uart_dev_s *dev, unsigned int *status);
static void ez80_rxint(struct uart_dev_s *dev, bool enable);
static bool ez80_rxavailable(struct uart_dev_s *dev);
static void ez80_send(struct uart_dev_s *dev, int ch);
static void ez80_txint(struct uart_dev_s *dev, bool enable);
static bool ez80_txready(struct uart_dev_s *dev);
static bool ez80_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  ez80_setup,          /* setup */
  ez80_shutdown,       /* shutdown */
  ez80_attach,         /* attach */
  ez80_detach,         /* detach */
  ez80_ioctl,          /* ioctl */
  ez80_receive,        /* receive */
  ez80_rxint,          /* rxint */
  ez80_rxavailable,    /* rxavailable */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  NULL,                /* rxflowcontrol */
#endif
  ez80_send,           /* send */
  ez80_txint,          /* txint */
  ez80_txready,        /* txready */
  ez80_txempty         /* txempty */
};

/* I/O buffers */

#ifdef CONFIG_EZ80_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_EZ80_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the UART0 port. */

#ifdef CONFIG_EZ80_UART0
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
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  true,                     /* isconsole */
#else
  false,                    /* isconsole */
#endif
  { 0 },                    /* closesem */
  { 0 },                    /* xmitsem */
  { 0 },                    /* recvsem */
  { 0 },                    /* pollsem */
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
  { },                      /* pollfds: all zero */
};
#endif

/* This describes the state of the UART1 port. */

#ifdef CONFIG_EZ80_UART1
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
  false,                    /* xmitwaiting */
  false,                    /* recvwaiting */
#ifdef CONFIG_UART1_SERIAL_CONSOLE
  true,                     /* isconsole */
#else
  false,                    /* isconsole */
#endif
  { 0 },                    /* closesem */
  { 0 },                    /* xmitsem */
  { 0 },                    /* recvsem */
  { 0 },                    /* pollsem */
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
    g_uart1rxbuffer,        /* recv.buffer */
  },
  &g_uart_ops,              /* ops */
  &g_uart1priv,             /* priv */
  { },                      /* pollfds */
};
#endif

/* Now, which one with be tty0/console and which tty1? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_EZ80_UART0)
# define CONSOLE_DEV     g_uart0port
# define TTYS0_DEV       g_uart0port
# if defined(CONFIG_EZ80_UART1)
#   define TTYS1_DEV     g_uart1port
# endif
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_EZ80_UART1)
# define CONSOLE_DEV     g_uart1port
# define TTYS0_DEV       g_uart1port
# if defined(CONFIG_EZ80_UART0)
#   define TTYS1_DEV     g_uart0port
# endif
#elif defined(CONFIG_EZ80_UART0)
# define TTYS0_DEV       g_uart0port
# if defined(CONFIG_EZ80_UART1)
#   define TTYS1_DEV     g_uart1port
# endif
#elif defined(CONFIG_EZ80_UART0)
# define TTYS0_DEV       g_uart1port
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_serialin
 ****************************************************************************/

static uint8_t ez80_serialin(FAR struct ez80_dev_s *priv, uint8_t offset)
{
  return inp(priv->uartbase + offset);
}

/****************************************************************************
 * Name: ez80_serialout
 ****************************************************************************/

static void ez80_serialout(FAR struct ez80_dev_s *priv, uint8_t offset,
                           uint8_t value)
{
  outp(priv->uartbase + offset, value);
}

/****************************************************************************
 * Name: ez80_disableuartint
 ****************************************************************************/

static void ez80_disableuartint(FAR struct ez80_dev_s *priv)
{
  uint8_t ier = ez80_serialin(priv, EZ80_UART_IER);
  ier &= ~EZ80_UARTEIR_INTMASK;
  ez80_serialout(priv, EZ80_UART_IER, ier);
}

/****************************************************************************
 * Name: ez80_restoreuartint
 ****************************************************************************/

static void ez80_restoreuartint(FAR struct ez80_dev_s *priv, uint8_t bits)
{
  uint8_t ier = ez80_serialin(priv, EZ80_UART_IER);
  ier |= bits & (EZ80_UARTEIR_TIE | EZ80_UARTEIR_RIE);
  ez80_serialout(priv, EZ80_UART_IER, ier);
}

/****************************************************************************
 * Name: ez80_waittxready
 ****************************************************************************/

static void ez80_waittxready(FAR struct ez80_dev_s *priv)
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

static void ez80_setbaud(FAR struct ez80_dev_s *priv, uint24_t baud)
{
  uint32_t brg_divisor;
  uint8_t lctl;

  /* The resulting BAUD and depends on the system clock frequency and the
   * BRG divisor as follows:
   *
   * BAUD = SYSTEM_CLOCK_FREQUENCY / (16 * BRG_Divisor)
   *
   * Or
   *
   * BRG_Divisor = SYSTEM_CLOCK_FREQUENCY / 16 / BAUD
   */

  brg_divisor = (ez80_systemclock + (baud << 3)) / (baud << 4);

  /* Set the DLAB bit to enable access to the BRG registers */

  lctl = ez80_serialin(priv, EZ80_UART_LCTL);
  lctl |= EZ80_UARTLCTL_DLAB;
  ez80_serialout(priv, EZ80_UART_LCTL, lctl);

  ez80_serialout(priv, EZ80_UART_BRGL, (uint8_t)(brg_divisor & 0xff));
  ez80_serialout(priv, EZ80_UART_BRGH, (uint8_t)(brg_divisor >> 8));

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

static int ez80_setup(FAR struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  FAR struct ez80_dev_s *priv = dev->priv;
  uint8_t reg;
  uint8_t cval;

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
      cval |= EZ80_UARTLCTL_2STOP;
    }

  if (priv->parity == 1)   /* Odd parity */
    {
      cval |= (EZ80_UARTLCTL_PEN);
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      cval |= (EZ80_UARTLCTL_PEN | EZ80_UARTLCTL_EPS);
    }

  /* Set the baud rate */

  ez80_disableuartint(priv);
  ez80_setbaud(priv, priv->baud);
  ez80_serialout(priv, EZ80_UART_MCTL, 0);

  /* Set the character properties */

  reg = (ez80_serialin(priv, EZ80_UART_LCTL) & ~EZ80_UARTLCTL_MASK) | cval;
  ez80_serialout(priv, EZ80_UART_LCTL, reg);

  /* Enable and flush the receive FIFO */

  reg  = EZ80_UARTFCTL_FIFOEN;
  ez80_serialout(priv, EZ80_UART_FCTL, reg);
  reg |= (EZ80_UARTFCTL_CLRTXF | EZ80_UARTFCTL_CLRRXF);
  ez80_serialout(priv, EZ80_UART_FCTL, reg);

  /* Set the Rx FIFO trigger level.  Small values assure the quickest
   * response to get data from the Rx FIFO.  This minimizes the
   * likelihood of Rx overruns with a penalty of more time spent
   * handling Rx interrupts.
   */

#if defined(CONFIG_EZ80_UART_RXFIFO_1)
  reg |= EZ80_UARTTRIG_1;
#elif defined(CONFIG_EZ80_UART_RXFIFO_4)
  reg |= EZ80_UARTTRIG_4;
#elif defined(CONFIG_EZ80_UART_RXFIFO_8)
  reg |= EZ80_UARTTRIG_8;
#elif defined(CONFIG_EZ80_UART_RXFIFO_14)
  reg |= EZ80_UARTTRIG_14;
#else
#  error No Rx FIFO trigger level
#endif

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

static void ez80_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  ez80_disableuartint(priv);
}

/****************************************************************************
 * Name: ez80_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int ez80_attach(FAR struct uart_dev_s *dev)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;

  /* Attach the IRQ */

  return irq_attach(priv->irq, ez80_interrupt, dev);
}

/****************************************************************************
 * Name: ez80_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void ez80_detach(FAR struct uart_dev_s *dev)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  ez80_disableuartint(priv);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: ez80_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int ez80_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;
  struct ez80_dev_s *priv;
  volatile uint32_t  cause;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct ez80_dev_s *)dev->priv;

  cause = ez80_serialin(priv, EZ80_UART_IIR) & EZ80_UARTIIR_CAUSEMASK;

  /* Check for character timeout (CTO) or Receiver Data Ready (RDR) */

  if (cause == EZ80_UARTINSTS_CTO || cause == EZ80_UARTINSTS_RDR)
    {
      /* Receive characters from the RX fifo */

      uart_recvchars(dev);
    }

  /* Check for transmission buffer empty */

  else if (cause == EZ80_UARTINSTS_TBE)
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

static int ez80_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
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

static int ez80_receive(FAR struct uart_dev_s *dev, FAR unsigned int *status)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  uint8_t rbr = ez80_serialin(priv, EZ80_UART_RBR);
  uint8_t lsr = ez80_serialin(priv, EZ80_UART_LSR);

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

static void ez80_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  uint8_t ier = ez80_serialin(priv, EZ80_UART_IER);

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
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool ez80_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  return (ez80_serialin(priv, EZ80_UART_LSR) & EZ80_UARTLSR_DR) != 0;
}

/****************************************************************************
 * Name: ez80_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void ez80_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  ez80_serialout(priv, EZ80_UART_THR, (uint8_t)ch);
}

/****************************************************************************
 * Name: ez80_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void ez80_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  uint8_t ier = ez80_serialin(priv, EZ80_UART_IER);

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
 *   Return true if the transmit fifo is not full
 *
 ****************************************************************************/

static bool ez80_txready(FAR struct uart_dev_s *dev)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  return (ez80_serialin(priv, EZ80_UART_LSR) & EZ80_UARTLSR_THRE) != 0;
}

/****************************************************************************
 * Name: ez80_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool ez80_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)dev->priv;
  return (ez80_serialin(priv, EZ80_UART_LSR) & EZ80_UARTLSR_TEMT) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z80_serial_initialize
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ****************************************************************************/

void z80_serial_initialize(void)
{
  uint8_t regval;

  /* Make sure that all UART interrupts are disabled */

  ez80_disableuartint(TTYS0_DEV.priv);
#ifdef TTYS1_DEV
  ez80_disableuartint(TTYS1_DEV.priv);
#endif

  /* Configure pins for usage of UARTs */

#ifdef CONFIG_EZ80_UART0
  /* Set Port D, pins 0 and 1 for their alternate function (Mode 7)
   * to enable UART0
   */

  regval  = inp(EZ80_PD_DDR);
  regval |= 3;
  outp(EZ80_PD_DDR, regval);

  regval  = inp(EZ80_PD_ALT1);
  regval &= ~3;
  outp(EZ80_PD_ALT1, regval);

  regval  = inp(EZ80_PD_ALT2);
  regval |= 3;
  outp(EZ80_PD_ALT2, regval);
#endif

#ifdef CONFIG_EZ80_UART1
  /* Set Port C, pins 0 and 1 for their alternate function (Mode 7)
   * to enable UART1
   */

  regval  = inp(EZ80_PC_DDR);
  regval |= 3;
  outp(EZ80_PC_DDR, regval);

  regval  = inp(EZ80_PC_ALT1);
  regval &= ~3;
  outp(EZ80_PC_ALT1, regval);

  regval  = inp(EZ80_PC_ALT2);
  regval |= 3;
  outp(EZ80_PC_ALT2, regval);
#endif

  /* If there is a console, then configure the console now */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  ez80_setup(&CONSOLE_DEV);
#endif

  /* Register console and tty devices */

#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
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
#ifdef CONSOLE_DEV
  FAR struct ez80_dev_s *priv = (FAR struct ez80_dev_s *)CONSOLE_DEV.priv;
  uint8_t ier = ez80_serialin(priv, EZ80_UART_IER);

  ez80_disableuartint(priv);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Output CR before LF */

      ez80_waittxready(priv);
      ez80_serialout(priv, EZ80_UART_THR, '\r');
    }

  /* Output the character */

  ez80_waittxready(priv);
  ez80_serialout(priv, EZ80_UART_THR, (uint8_t)ch);

  /* Wait for the character to be sent before re-enabling interrupts */

  ez80_waittxready(priv);
  ez80_restoreuartint(priv, ier);
  return ch;
#endif
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_UART1_SERIAL_CONSOLE
# define ez80_inp(offs)      inp((EZ80_UART1_BASE+(offs)))
# define ez80_outp(offs,val) outp((EZ80_UART1_BASE+(offs)), (val))
#else
# define ez80_inp(offs)      inp((EZ80_UART0_BASE+(offs)))
# define ez80_outp(offs,val) outp((EZ80_UART0_BASE+(offs)), (val))
#endif

#define ez80_txready()       ((ez80_inp(EZ80_UART_LSR) & EZ80_UARTLSR_THRE) != 0)
#define ez80_send(ch)        ez80_outp(EZ80_UART_THR, ch)

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

#endif /* USE_SERIALDRIVER */
