/****************************************************************************
 * arch/arm/src/efm32/efm32_leserial.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/efm32_leuart.h"
#include "efm32_config.h"
#include "efm32_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there at least one UART enabled and configured as a RS-232 device? */

#ifndef HAVE_LEUART_DEVICE
#  warning "No LEUARTs enabled"
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which LEUART with be ttyLE0/console and which ttyLE1?  The console will
 * always be ttyLE0.  If there is no console then will use the lowest
 * numbered LEUART.
 */

/* First pick the console and ttys0.  This could be either LEUART0-1 */

#if defined(CONFIG_LEUART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_leuart0port /* LEUART0 is console */
#    define TTYLE0_DEV          g_leuart0port /* LEUART0 is ttyLE0 */
#    define LEUART0_ASSIGNED    1
#elif defined(CONFIG_LEUART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_leuart1port /* LEUART1 is console */
#    define TTYLE0_DEV          g_leuart1port /* LEUART1 is ttyLE0 */
#    define LEUART1_ASSIGNED    1
#else
#  undef CONSOLE_DEV                          /* No console */
#  if defined(CONFIG_EFM32_LEUART0)
#    define TTYLE0_DEV          g_leuart0port /* LEUART0 is ttyLE0 */
#    define LEUART0_ASSIGNED    1
#  elif defined(CONFIG_EFM32_LEUART1)
#    define TTYLE0_DEV          g_leuart1port /* LEUART1 is ttyLE0 */
#    define LEUART1_ASSIGNED    1
#  endif
#endif

/* Pick ttyLE1.  This could be any of LEUART0-1, excluding the and the
 * LEUART already selected for ttyLE0
 */

#if defined(CONFIG_EFM32_LEUART0) && !defined(LEUART0_ASSIGNED)
#  define TTYLE1_DEV            g_leuart0port /* LEUART0 is ttyLE1 */
#  define LEUART0_ASSIGNED      1
#elif defined(CONFIG_EFM32_LEUART1) && !defined(LEUART1_ASSIGNED)
#  define TTYLE1_DEV            g_leuart1port /* LEUART1 is ttyLE1 */
#  define LEUART1_ASSIGNED      1
#endif

/* TX/RX interrupts */

#define EFM32_TXERR_INTS      (LEUART_IEN_TXOF)
#define EFM32_RXERR_INTS      (LEUART_IEN_RXOF | LEUART_IEN_PERR | \
                               LEUART_IEN_FERR)
#ifdef CONFIG_DEBUG_FEATURES
#  define EFM32_TX_INTS       (LEUART_IEN_TXBL | EFM32_TXERR_INTS)
#  define EFM32_RX_INTS       (LEUART_IEN_RXDATAV | EFM32_RXERR_INTS)
#else
#  define EFM32_TX_INTS        LEUART_IEN_TXBL
#  define EFM32_RX_INTS        LEUART_IEN_RXDATAV
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct efm32_config_s
{
  uintptr_t uartbase;  /* Base address of UART registers */
  uint32_t  baud;      /* Configured baud */
  uint8_t   irq;       /* IRQ associated with this LEUART (for enable) */
  uint8_t   parity;    /* 0=none, 1=odd, 2=even */
  uint8_t   bits;      /* Number of bits (8 or 9) */
  bool      stop2;     /* True: 2 stop bits */
};

struct efm32_leuart_s
{
  const struct efm32_config_s *config;
  uint16_t  ien;       /* Interrupts enabled */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t efm32_serialin(struct efm32_leuart_s *priv,
                                      int offset);
static inline void efm32_serialout(struct efm32_leuart_s *priv, int offset,
                                   uint32_t value);
static inline void efm32_setuartint(struct efm32_leuart_s *priv);

static void efm32_restoreuartint(struct efm32_leuart_s *priv, uint32_t ien);
static void efm32_disableuartint(struct efm32_leuart_s *priv, uint32_t *ien);
static int  efm32_setup(struct uart_dev_s *dev);
static void efm32_shutdown(struct uart_dev_s *dev);
static int  efm32_attach(struct uart_dev_s *dev);
static void efm32_detach(struct uart_dev_s *dev);
static int  efm32_interrupt(int irq, void *context, void *arg);
static int  efm32_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  efm32_receive(struct uart_dev_s *dev, unsigned int *status);
static void efm32_rxint(struct uart_dev_s *dev, bool enable);
static bool efm32_rxavailable(struct uart_dev_s *dev);
static void efm32_send(struct uart_dev_s *dev, int ch);
static void efm32_txint(struct uart_dev_s *dev, bool enable);
static bool efm32_txready(struct uart_dev_s *dev);
static bool efm32_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_leuart_ops =
{
  .setup          = efm32_setup,
  .shutdown       = efm32_shutdown,
  .attach         = efm32_attach,
  .detach         = efm32_detach,
  .ioctl          = efm32_ioctl,
  .receive        = efm32_receive,
  .rxint          = efm32_rxint,
  .rxavailable    = efm32_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = efm32_send,
  .txint          = efm32_txint,
  .txready        = efm32_txready,
  .txempty        = efm32_txempty,
};

/* I/O buffers */

#ifdef CONFIG_EFM32_LEUART0
static char g_leuart0rxbuffer[CONFIG_LEUART0_RXBUFSIZE];
static char g_leuart0txbuffer[CONFIG_LEUART0_TXBUFSIZE];
#endif
#ifdef CONFIG_EFM32_LEUART1
static char g_leuart1rxbuffer[CONFIG_LEUART1_RXBUFSIZE];
static char g_leuart1txbuffer[CONFIG_LEUART1_TXBUFSIZE];
#endif

/* This describes the state of the EFM32 LEUART0 port. */

#ifdef CONFIG_EFM32_LEUART0
static const struct efm32_config_s g_leuart0config =
{
  .uartbase  = EFM32_LEUART0_BASE,
  .baud      = CONFIG_LEUART0_BAUD,
  .irq       = EFM32_IRQ_LEUART0,
  .parity    = CONFIG_LEUART0_PARITY,
  .bits      = CONFIG_LEUART0_BITS,
  .stop2     = CONFIG_LEUART0_2STOP,
};

static struct efm32_leuart_s g_leuart0priv =
{
  .config    = &g_leuart0config,
};

static struct uart_dev_s g_leuart0port =
{
  .recv      =
  {
    .size    = CONFIG_LEUART0_RXBUFSIZE,
    .buffer  = g_leuart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_LEUART0_TXBUFSIZE,
    .buffer  = g_leuart0txbuffer,
  },
  .ops       = &g_leuart_ops,
  .priv      = &g_leuart0priv,
};
#endif

/* This describes the state of the EFM32 LEUART1 port. */

#ifdef CONFIG_EFM32_LEUART1
static struct efm32_config_s g_leuart1config =
{
  .uartbase  = EFM32_LEUART1_BASE,
  .baud      = CONFIG_LEUART1_BAUD,
  .irq       = EFM32_IRQ_LEUART1,
  .parity    = CONFIG_LEUART1_PARITY,
  .bits      = CONFIG_LEUART1_BITS,
  .stop2     = CONFIG_LEUART1_2STOP,
};

static struct efm32_leuart_s g_leuart1priv =
{
  .config    = &g_leuart1config,
};

static struct uart_dev_s g_leuart1port =
{
  .recv      =
  {
    .size    = CONFIG_LEUART1_RXBUFSIZE,
    .buffer  = g_leuart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_LEUART1_TXBUFSIZE,
    .buffer  = g_leuart1txbuffer,
  },
  .ops       = &g_leuart_ops,
  .priv      = &g_leuart1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_serialin
 ****************************************************************************/

static inline uint32_t efm32_serialin(struct efm32_leuart_s *priv,
                                      int offset)
{
  return getreg32(priv->config->uartbase + offset);
}

/****************************************************************************
 * Name: efm32_serialout
 ****************************************************************************/

static inline void efm32_serialout(struct efm32_leuart_s *priv, int offset,
                                   uint32_t value)
{
  putreg32(value, priv->config->uartbase + offset);
}

/****************************************************************************
 * Name: efm32_setuartint
 ****************************************************************************/

static inline void efm32_setuartint(struct efm32_leuart_s *priv)
{
  efm32_serialout(priv, EFM32_LEUART_IEN_OFFSET, priv->ien);
}

/****************************************************************************
 * Name: efm32_restoreuartint
 ****************************************************************************/

static void efm32_restoreuartint(struct efm32_leuart_s *priv, uint32_t ien)
{
  irqstate_t flags;

  /* Re-enable/re-disable interrupts corresponding to the state of
   * bits in ien.
   */

  flags     = enter_critical_section();
  priv->ien = ien;
  efm32_setuartint(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: efm32_disableuartint
 ****************************************************************************/

static void efm32_disableuartint(struct efm32_leuart_s *priv, uint32_t *ien)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (ien)
    {
      *ien = priv->ien;
    }

  efm32_restoreuartint(priv, 0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: efm32_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int efm32_setup(struct uart_dev_s *dev)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;

#ifndef CONFIG_SUPPRESS_LEUART_CONFIG
  const struct efm32_config_s *config = priv->config;

  /* Configure the UART as an RS-232 UART */

  efm32_leuartconfigure(config->uartbase, config->baud, config->parity,
                        config->bits, config->stop2);
#endif

  /* Make sure that all interrupts are disabled */

  efm32_restoreuartint(priv, 0);
  return OK;
}

/****************************************************************************
 * Name: efm32_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void efm32_shutdown(struct uart_dev_s *dev)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;

  /* Disable interrupts */

  efm32_restoreuartint(priv, 0);

  /* Reset the LEUART/UART by disabling it and restoring all of the registers
   * to the initial, reset value.  Only the ROUTE data set by efm32_lowsetup
   * is preserved.
   */

  efm32_leuart_reset(priv->config->uartbase);
}

/****************************************************************************
 * Name: efm32_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int efm32_attach(struct uart_dev_s *dev)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;
  const struct efm32_config_s *config = priv->config;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(config->irq, efm32_interrupt, dev);
  if (ret >= 0)
    {
      up_enable_irq(config->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: efm32_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port
 *   is closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void efm32_detach(struct uart_dev_s *dev)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;
  const struct efm32_config_s *config = priv->config;

  /* Disable interrupts */

  efm32_restoreuartint(priv, 0);
  up_disable_irq(config->irq);

  /* Detach from the interrupt(s) */

  irq_detach(config->irq);
}

/****************************************************************************
 * Name: efm32_interrupt
 *
 * Description:
 *   This is the common UART RX interrupt handler.
 *
 ****************************************************************************/

static int efm32_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct efm32_leuart_s *priv;
  uint32_t intflags;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct efm32_leuart_s *)dev->priv;

  /* Read the interrupt flags register */

  intflags = efm32_serialin(priv, EFM32_LEUART_IF_OFFSET);

  /* Clear pending interrupts by writing to the interrupt flag clear
   * register.
   */

  efm32_serialout(priv, EFM32_LEUART_IFC_OFFSET, intflags);

  /* Check if the receive data is available is full (RXDATAV). */

  if ((intflags & LEUART_IEN_RXDATAV) != 0)
    {
      /* Process incoming bytes */

      uart_recvchars(dev);
    }

  /* Check if the transmit data buffer became empty */

  if ((intflags & LEUART_IEN_TXBL) != 0)
    {
      /* Process outgoing bytes */

      uart_xmitchars(dev);
    }

#ifdef CONFIG_DEBUG_FEATURES
  /* Check for receive errors */

  if ((intflags & EFM32_RXERR_INTS) != 0)
    {
      /* RXOF - RX Overflow Interrupt Enable
       * RXUF - RX Underflow Interrupt Enable
       * TXUF - TX Underflow Interrupt Enable
       * PERR - Parity Error Interrupt Enable
       * FERR - Framing Error Interrupt Enable
       */

      _err("RX ERROR: %08x\n", intflags);
    }

  /* Check for transmit errors */

  if ((intflags & EFM32_TXERR_INTS) != 0)
    {
      /* TXOF - TX Overflow Interrupt Enable */

      _err("RX ERROR: %08x\n", intflags);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: efm32_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int efm32_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  struct inode *inode;
  struct uart_dev_s *dev;
  struct efm32_leuart_s *priv;
  int ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv);
  priv = (struct efm32_leuart_s *)dev->priv;

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
 * Name: efm32_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int efm32_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;
  uint32_t rxdatax;

  /* Get error status information:
   *
   *   FERR Data Framing Error
   *   PERR Data Parity Error
   */

  rxdatax = efm32_serialin(priv, EFM32_LEUART_RXDATAX_OFFSET);

  /* Return status information */

  if (status)
    {
      *status = rxdatax;
    }

  /* Then return the actual received byte. */

  return (int)(rxdatax & _LEUART_RXDATAX_RXDATA_MASK);
}

/****************************************************************************
 * Name: efm32_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void efm32_rxint(struct uart_dev_s *dev, bool enable)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Receive an interrupt when there is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ien |= EFM32_RX_INTS;
      efm32_setuartint(priv);
#endif
    }
  else
    {
      priv->ien &= ~EFM32_RX_INTS;
      efm32_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: efm32_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty.
 *
 ****************************************************************************/

static bool efm32_rxavailable(struct uart_dev_s *dev)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;

  /* Return true if the receive data is available (RXDATAV). */

  return (efm32_serialin(priv, EFM32_LEUART_STATUS_OFFSET) &
          LEUART_STATUS_RXDATAV) != 0;
}

/****************************************************************************
 * Name: efm32_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void efm32_send(struct uart_dev_s *dev, int ch)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;
  efm32_serialout(priv, EFM32_LEUART_TXDATA_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: efm32_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void efm32_txint(struct uart_dev_s *dev, bool enable)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ien |= EFM32_TX_INTS;
      efm32_setuartint(priv);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->ien &= ~EFM32_TX_INTS;
      efm32_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: efm32_txready
 *
 * Description:
 *   Return true if the transmit data register is not full
 *
 ****************************************************************************/

static bool efm32_txready(struct uart_dev_s *dev)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;

  /* The TX Buffer Level (TXBL) status bit indicates the level of the
   * transmit buffer.  Set when the transmit buffer is empty, and cleared
   * when it is full.
   */

  return (efm32_serialin(priv, EFM32_LEUART_STATUS_OFFSET) &
          LEUART_STATUS_TXBL) != 0;
}

/****************************************************************************
 * Name: efm32_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool efm32_txempty(struct uart_dev_s *dev)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)dev->priv;

  /* TX Complete (TXC) is set when a transmission has completed and no more
   * data is available in the transmit buffer.
   */

  return (efm32_serialin(priv, EFM32_LEUART_STATUS_OFFSET) &
          LEUART_STATUS_TXC) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in efm32_consoleinit() and main clock
 *   initialization performed in efm32_clkinitialize().
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* Disable interrupts from all UARTS. */

  efm32_restoreuartint(TTYLE0_DEV.priv, 0);
#ifdef TTYLE1_DEV
  efm32_restoreuartint(TTYLE1_DEV.priv, 0);
#endif

  /* Configuration whichever one is the console. */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  efm32_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  /* Register the console */

#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyLE0", &TTYLE0_DEV);
#ifdef TTYLE1_DEV
  uart_register("/dev/ttyLE1", &TTYLE1_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

#ifdef HAVE_LEUART_CONSOLE
int up_putc(int ch)
{
  struct efm32_leuart_s *priv = (struct efm32_leuart_s *)CONSOLE_DEV.priv;
  uint32_t ien;

  efm32_disableuartint(priv, &ien);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      efm32_lowputc('\r');
    }

  efm32_lowputc(ch);
  efm32_restoreuartint(priv, ien);
  return ch;
}
#endif

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

#ifdef HAVE_LEUART_CONSOLE
int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      efm32_lowputc('\r');
    }

  efm32_lowputc(ch);
  return ch;
}
#endif

#endif /* USE_SERIALDRIVER */
