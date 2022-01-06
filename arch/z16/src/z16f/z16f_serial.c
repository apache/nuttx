/****************************************************************************
 * arch/z16/src/z16f/z16f_serial.c
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
#include <nuttx/serial/serial.h>

#include "chip.h"
#include "z16_internal.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System clock frequency value from ZDS target settings */

extern _Erom unsigned long SYS_CLK_FREQ;
#define _DEFCLK ((unsigned long)&SYS_CLK_FREQ)

#define STATE_DISABLED   0
#define STATE_RXENABLED  1
#define STATE_TXENABLED  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct z16f_uart_s
{
  uint32_t     uartbase;    /* Base address of UART registers */
  uint32_t     baud;        /* Configured baud */
  bool         rxenabled;   /* RX interrupt enabled */
  bool         txenabled;   /* TX interrupt enabled */
  uint8_t      rxirq;       /* RX IRQ associated with this UART */
  uint8_t      txirq;       /* RX IRQ associated with this UART */
  uint8_t      parity;      /* 0=none, 1=odd, 2=even */
  bool         stopbits2;   /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  z16f_setup(struct uart_dev_s *dev);
static void z16f_shutdown(struct uart_dev_s *dev);
static int  z16f_attach(struct uart_dev_s *dev);
static void z16f_detach(struct uart_dev_s *dev);
static int  z16f_rxinterrupt(int irq, void *context, void *arg);
static int  z16f_txinterrupt(int irq, void *context, void *arg);
static int  z16f_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  z16f_receive(struct uart_dev_s *dev, uint32_t *status);
static void z16f_rxint(struct uart_dev_s *dev, bool enable);
static bool z16f_rxavailable(struct uart_dev_s *dev);
static void z16f_send(struct uart_dev_s *dev, int ch);
static void z16f_txint(struct uart_dev_s *dev, bool enable);
static bool z16f_txready(struct uart_dev_s *dev);
static bool z16f_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  z16f_setup,          /* setup */
  z16f_shutdown,       /* shutdown */
  z16f_attach,         /* attach */
  z16f_detach,         /* detach */
  z16f_ioctl,          /* ioctl */
  z16f_receive,        /* receive */
  z16f_rxint,          /* rxint */
  z16f_rxavailable,    /* rxavailable */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  NULL,                /* rxflowcontrol */
#endif
  z16f_send,           /* send */
  z16f_txint,          /* txint */
  z16f_txready,        /* txready */
  z16f_txempty         /* txempty */
};

/* I/O buffers */

#ifdef CONFIG_Z16F_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_Z16F_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_Z16F_UART0
/* This describes the state of the ZNEO uart0 port. */

static struct z16f_uart_s g_uart0priv =
{
  Z16F_UART0_BASE,          /* uartbase */
  CONFIG_UART0_BAUD,        /* baud */
  false,                    /* rxenabled */
  false,                    /* txenabled */
  Z16F_IRQ_UART0RX,         /* rxirq */
  Z16F_IRQ_UART0TX,         /* txirq */
  CONFIG_UART0_PARITY,      /* parity */
  CONFIG_UART0_2STOP        /* stopbits2 */
};

static uart_dev_t g_uart0port;
#endif

#ifdef CONFIG_Z16F_UART1
/* This describes the state of the ZNEO uart1 port. */

static struct z16f_uart_s g_uart1priv =
{
  Z16F_UART1_BASE,          /* uartbase */
  CONFIG_UART1_BAUD,        /* baud */
  false,                    /* rxenabled */
  false,                    /* txenabled */
  Z16F_IRQ_UART1RX,         /* rxirq */
  Z16F_IRQ_UART1TX,         /* txirq */
  CONFIG_UART1_PARITY,      /* parity */
  CONFIG_UART1_2STOP        /* stopbits2 */
};

static uart_dev_t g_uart1port;
#endif

/* Now, which one with be tty0/console and which tty1? */

#ifndef CONFIG_Z16F_UART0
#  undef CONFIG_UART0_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z16F_UART1
#  undef CONFIG_UART1_SERIAL_CONSOLE
#endif

/* First pick the console and ttys0.  This could be either of UART0-1 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port /* UART0 is console */
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port /* UART1 is console */
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_Z16F_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_Z16F_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be either of UART0-1 excluding the console UART. */

#if defined(CONFIG_Z16F_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_Z16F_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16f_disableuartirq
 ****************************************************************************/

static uint8_t z16f_disableuartirq(struct uart_dev_s *dev)
{
  struct z16f_uart_s *priv  = (struct z16f_uart_s *)dev->priv;
  irqstate_t          flags = enter_critical_section();
  uint8_t             state = priv->rxenabled ? STATE_RXENABLED :
                                                STATE_DISABLED |
                              priv->txenabled ? STATE_TXENABLED :
                                                STATE_DISABLED;

  z16f_txint(dev, false);
  z16f_rxint(dev, false);

  leave_critical_section(flags);
  return state;
}

/****************************************************************************
 * Name: z16f_restoreuartirq
 ****************************************************************************/

static void z16f_restoreuartirq(struct uart_dev_s *dev, uint8_t state)
{
  irqstate_t flags = enter_critical_section();

  z16f_txint(dev, (state & STATE_TXENABLED) ? true : false);
  z16f_rxint(dev, (state & STATE_RXENABLED) ? true : false);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: z16f_consoleput
 ****************************************************************************/

#ifdef CONSOLE_DEV
static void z16f_consoleput(uint8_t ch)
{
  struct z16f_uart_s *priv = (struct z16f_uart_s *)CONSOLE_DEV.priv;
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if (z16f_txready(&CONSOLE_DEV))
        {
          break;
        }
    }

  putreg8(ch,  priv->uartbase + Z16F_UART_TXD);
}
#endif

/****************************************************************************
 * Name: z16f_setup
 *
 * Description:
 *   Configure the UART baud, parity, etc. This method is called the first
 *   time that the serial port is opened.
 *
 ****************************************************************************/

static int z16f_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct z16f_uart_s *priv = (struct z16f_uart_s *)dev->priv;
  uint32_t brg;
  uint8_t ctl0;
  uint8_t ctl1;

  /* Calculate and set the baud rate generation register.
   * BRG = (freq + baud * 8)/(baud * 16)
   */

  brg = (_DEFCLK + (priv->baud << 3)) / (priv->baud << 4);
  putreg16((uint16_t)brg, priv->uartbase + Z16F_UART_BR);

  /* Configure STOP bits */

  ctl0 = 0;
  ctl1 = 0;

  if (priv->stopbits2)
    {
      ctl0 |= Z16F_UARTCTL0_STOP;
    }

  /* Configure parity */

  if (priv->parity == 1)
    {
      ctl0 |= (Z16F_UARTCTL0_PEN | Z16F_UARTCTL0_PSEL);
    }
  else if (priv->parity == 2)
    {
      ctl0 |= Z16F_UARTCTL0_PEN;
    }

  putreg8(ctl0, priv->uartbase + Z16F_UART_CTL0);
  putreg8(ctl1, priv->uartbase + Z16F_UART_CTL1);

  /* Enable UART receive (REN) and transmit (TEN) */

  ctl0 |= (Z16F_UARTCTL0_TEN | Z16F_UARTCTL0_REN);
  putreg8(ctl0, priv->uartbase + Z16F_UART_CTL0);
#endif

  return OK;
}

/****************************************************************************
 * Name: z16f_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void z16f_shutdown(struct uart_dev_s *dev)
{
  z16f_disableuartirq(dev);
}

/****************************************************************************
 * Name: z16f_attach
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

static int z16f_attach(struct uart_dev_s *dev)
{
  struct z16f_uart_s *priv = (struct z16f_uart_s *)dev->priv;
  int ret;

  /* Attach the RX IRQ */

  ret = irq_attach(priv->rxirq, z16f_rxinterrupt, dev);
  if (ret == OK)
    {
      /* Attach the TX IRQ */

      ret = irq_attach(priv->txirq, z16f_txinterrupt, dev);
      if (ret != OK)
        {
          irq_detach(priv->rxirq);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: z16f_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void z16f_detach(struct uart_dev_s *dev)
{
  struct z16f_uart_s *priv = (struct z16f_uart_s *)dev->priv;

  up_disable_irq(priv->rxirq);
  up_disable_irq(priv->txirq);

  irq_detach(priv->rxirq);
  irq_detach(priv->txirq);
}

/****************************************************************************
 * Name: z16f_rxinterrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an RX
 *   event occurs at the Z16F's LIN-UART.
 *
 ****************************************************************************/

static int z16f_rxinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s  *dev = (struct uart_dev_s *)arg;
  struct z16f_uart_s *priv;
  uint8_t            status;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct z16f_uart_s *)dev->priv;

  /* Check the LIN-UART status 0 register to determine whether the source of
   * the interrupt is error, break, or received data
   */

  status = getreg8(priv->uartbase + Z16F_UART_STAT0);

  /* REVISIT error and break handling */

  /* Check if received data is available  */

  if (status & Z16F_UARTSTAT0_RDA)
    {
      /* Handle an incoming, received byte */

      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: z16f_txinterrupt
 *
 * Description:
 *   This is the UART TX interrupt handler.  This interrupt handler will
 *   be invoked when the X16F LIN UART transmit data register is empty.
 *
 ****************************************************************************/

static int z16f_txinterrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s  *dev = (struct uart_dev_s *)arg;
  struct z16f_uart_s *priv;
  uint8_t            status;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct z16f_uart_s *)dev->priv;

  /* Verify that the transmit data register is empty */

  status = getreg8(priv->uartbase + Z16F_UART_STAT0);
  if (status & Z16F_UARTSTAT0_TDRE)
    {
      /* Handle outgoing, transmit bytes */

      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: z16f_ioctl
 *
 * Description:
 *   All ioctl that are not handled by the upper half serial driver will be
 *   routed through this method
 *
 ****************************************************************************/

static int z16f_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: z16f_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character from
 *   the UART.  Error bits associated with the receipt are provided in the
 *   return 'status'.
 *
 ****************************************************************************/

static int z16f_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct z16f_uart_s *priv = (struct z16f_uart_s *)dev->priv;
  uint8_t rxd;
  uint8_t stat0;

  rxd     = getreg8(priv->uartbase + Z16F_UART_RXD);
  stat0   = getreg8(priv->uartbase + Z16F_UART_STAT0);
  *status = (uint32_t)rxd | (((uint32_t)stat0) << 8);

  return rxd;
}

/****************************************************************************
 * Name: z16f_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void z16f_rxint(struct uart_dev_s *dev, bool enable)
{
  struct z16f_uart_s *priv  = (struct z16f_uart_s *)dev->priv;
  irqstate_t          flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_enable_irq(priv->rxirq);
#endif
    }
  else
    {
      up_disable_irq(priv->rxirq);
    }

  priv->rxenabled = enable;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: z16f_rxavailable
 *
 * Description:
 *   Return true if the receive FIFO is not empty
 *
 ****************************************************************************/

static bool z16f_rxavailable(struct uart_dev_s *dev)
{
  struct z16f_uart_s *priv = (struct z16f_uart_s *)dev->priv;
  return ((getreg8(priv->uartbase + Z16F_UART_STAT0) &
          Z16F_UARTSTAT0_RDA) != 0);
}

/****************************************************************************
 * Name: z16f_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void z16f_send(struct uart_dev_s *dev, int ch)
{
  struct z16f_uart_s *priv = (struct z16f_uart_s *)dev->priv;
  putreg8(ch, priv->uartbase + Z16F_UART_TXD);
}

/****************************************************************************
 * Name: z16f_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void z16f_txint(struct uart_dev_s *dev, bool enable)
{
  struct z16f_uart_s *priv  = (struct z16f_uart_s *)dev->priv;
  irqstate_t          flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_enable_irq(priv->txirq);
#endif

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
    }
  else
    {
      up_disable_irq(priv->txirq);
    }

  priv->txenabled = enable;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: z16f_txready
 *
 * Description:
 *   Return true if the transmit FIFO is not full
 *
 ****************************************************************************/

static bool z16f_txready(struct uart_dev_s *dev)
{
  struct z16f_uart_s *priv = (struct z16f_uart_s *)dev->priv;
  return ((getreg8(priv->uartbase + Z16F_UART_STAT0) &
           Z16F_UARTSTAT0_TDRE) != 0);
}

/****************************************************************************
 * Name: z16f_txempty
 *
 * Description:
 *   Return true if the transmit FIFO is empty
 *
 ****************************************************************************/

static bool z16f_txempty(struct uart_dev_s *dev)
{
  struct z16f_uart_s *priv = (struct z16f_uart_s *)dev->priv;
  return ((getreg8(priv->uartbase + Z16F_UART_STAT0) &
          Z16F_UARTSTAT0_TXE) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: z16_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before z16f_serialinit.
 *
 ****************************************************************************/

void z16_earlyserialinit(void)
{
  uint8_t regval;

  /* Configure UART alternate pin functions.  This may duplicate logic in
   * z16f_lowuartinit() or z16f_board_initialize().
   */

#ifdef CONFIG_Z16F_UART0
  /* UART0 is PA4 and PA5, alternate function 1 */

  regval  = getreg8(Z16F_GPIOA_AFL);
  regval |= 0x30;
  putreg8(regval, Z16F_GPIOA_AFL);

  regval  = getreg8(Z16F_GPIOA_AFH);
  regval &= ~0x30;
  putreg8(regval, Z16F_GPIOA_AFH);

  /* Initialize the UART structure */

  memset(&g_uart0port, 0, sizeof(uart_dev_t));
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  g_uart0port.isconsole   = true;
#endif
  g_uart0port.xmit.size   = CONFIG_UART0_TXBUFSIZE;
  g_uart0port.xmit.buffer = g_uart0txbuffer;
  g_uart0port.recv.size   = CONFIG_UART0_RXBUFSIZE;
  g_uart0port.recv.buffer = g_uart0rxbuffer;
  g_uart0port.ops         = &g_uart_ops;
  g_uart0port.priv        = &g_uart0priv;
#endif

#ifdef CONFIG_Z16F_UART1
  /* UART1 is PD4 and PD5, alternate function 1 */

  regval  = getreg8(Z16F_GPIOD_AFL);
  regval |= 0x30;
  putreg8(regval, Z16F_GPIOD_AFL);

  regval  = getreg8(Z16F_GPIOD_AFH);
  regval &= ~0x30;
  putreg8(regval, Z16F_GPIOD_AFH);

  /* Initialize the UART structure */

  memset(&g_uart1port, 0, sizeof(uart_dev_t));
#ifdef CONFIG_UART1_SERIAL_CONSOLE
  g_uart1port.isconsole   = true;
#endif
  g_uart1port.xmit.size   = CONFIG_UART1_TXBUFSIZE;
  g_uart1port.xmit.buffer = g_uart1txbuffer;
  g_uart1port.recv.size   = CONFIG_UART1_RXBUFSIZE;
  g_uart1port.recv.buffer = g_uart1rxbuffer;
  g_uart1port.ops         = &g_uart_ops;
  g_uart1port.priv        = &g_uart1priv;
#endif

  /* Disable UART interrupts */

#ifdef TTYS0_DEV
  z16f_disableuartirq(&TTYS0_DEV);
#endif

#ifdef TTYS1_DEV
  z16f_disableuartirq(&TTYS1_DEV);
#endif

  /* Configuration any serial console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  z16f_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: z16_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   z16_earlyserialinit was called previously.
 *
 ****************************************************************************/

void z16_serialinit(void)
{
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

#ifdef CONSOLE_DEV
int up_putc(int ch)
{
  uint8_t  state;

  /* Keep interrupts disabled so that we do not interfere with normal
   * driver operation.
   *
   * REVISIT:  I can imagine scenarios where the follow logic gets pre-empted
   * and the UART interrupts get left in a bad state.
   */

  state = z16f_disableuartirq(&CONSOLE_DEV);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR before LF */

      z16f_consoleput('\r');
    }

  /* Output the character */

  z16f_consoleput((uint8_t)ch);

  /* It is important to restore the TX interrupt while the send is pending.
   * otherwise, TRDE interrupts can be lost since they do not pend after the
   * TRDE false->true transition.
   */

  z16f_restoreuartirq(&CONSOLE_DEV, state);
  return ch;
}
#endif

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_UART1_SERIAL_CONSOLE
# define z16f_contrde() \
  ((getreg8(Z16F_UART1_STAT0) & Z16F_UARTSTAT0_TDRE) != 0)
# define z16f_contxd(ch) \
  putreg8((uint8_t)(ch), Z16F_UART1_TXD)
#else
# define z16f_contrde() \
  ((getreg8(Z16F_UART0_STAT0) & Z16F_UARTSTAT0_TDRE) != 0)
# define z16f_contxd(ch) \
  putreg8((uint8_t)(ch), Z16F_UART0_TXD)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16f_putc
 ****************************************************************************/

static void z16f_putc(int ch)
{
  int tmp;
  for (tmp = 1000 ; tmp > 0 && !z16f_contrde(); tmp--);
  z16f_contxd(ch);
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

      z16f_putc('\r');
    }

  /* Output character */

  z16f_putc(ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
