/****************************************************************************
 * arch/arm/src/efm32/efm32_serial.c
 *
 *   Copyright (C) 2024 Gregory Nutt. All rights reserved.
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

#include "chip/efm32_usart.h"
#include "efm32_config.h"
#include "efm32_lowputc.h"

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

/* First pick the console and ttys0.  This could be any of USART0-2 or
 * UART0-1.
 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart0port /* USART0 is console */
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart1port /* USART1 is console */
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED     1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart2port /* USART2 is console */
#    define TTYS0_DEV           g_usart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#elif defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port  /* UART0 is console */
#    define TTYS0_DEV           g_uart0port  /* UART0 is ttyS0 */
#    define UART1_ASSIGNED      1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port  /* UART1 is console */
#    define TTYS0_DEV           g_uart1port  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#else
#  undef CONSOLE_DEV                         /* No console */
#  if defined(CONFIG_EFM32_USART0)
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#  elif defined(CONFIG_EFM32_USART1)
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED     1
#  elif defined(CONFIG_EFM32_USART2)
#    define TTYS0_DEV           g_usart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#  elif defined(CONFIG_EFM32_UART0)
#    define TTYS0_DEV           g_uart0port  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_EFM32_UART1)
#    define TTYS0_DEV           g_uart1port  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of USART0-2 or UART0-1, excluding the
 * console UART.  There are really only 4 unassigned.
 */

#if defined(CONFIG_EFM32_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV           g_usart0port /* USART0 is ttyS1 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_EFM32_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS1_DEV           g_usart1port /* USART1 is ttyS1 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_EFM32_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS1_DEV           g_usart2port /* USART2 is ttyS1 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_EFM32_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port  /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_EFM32_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port  /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of USART1-5 or UART0-1. It can't be USART0
 * because that was either assigned as ttyS0 or ttys1.  One of these could
 * also be the console.  There are really only 3 unassigned.
 */

#if defined(CONFIG_EFM32_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS2_DEV           g_usart1port /* USART1 is ttyS2 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_EFM32_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS2_DEV           g_usart2port /* USART2 is ttyS2 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_EFM32_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS2_DEV           g_uart0port /* UART0 is ttyS2 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_EFM32_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART2 or UART0-1. It can't be USART0-1
 * because those have already been assigned to ttsyS0, 1, or 2.  One of
 * these could also be the console.  There are really only 2 unassigned.
 */

#if defined(CONFIG_EFM32_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS3_DEV           g_usart2port /* USART2 is ttyS3 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_EFM32_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS3_DEV           g_uart0port /* UART0 is ttyS3 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_EFM32_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS3_DEV           g_uart1port /* UART1 is ttyS3 */
#  define UART1_ASSIGNED      1
#endif

/* Pick ttys4. This could be one of UART0-1. It can't be USART0-2 because
* those have already been assigned to ttsyS0, 1, 2, or 3.  One of
 * these could also be the console.  There is really only 1 unassigned.
 */

#if defined(CONFIG_EFM32_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS4_DEV           g_uart0port /* UART0 is ttyS4 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_EFM32_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS4_DEV           g_uart1port /* UART1 is ttyS4 */
#  define UART1_ASSIGNED      1
#endif

/* TX/RX interrupts */

#define EFM32_TXERR_INTS      (USART_IEN_TXOF)
#define EFM32_RXERR_INTS      (USART_IEN_RXOF | USART_IEN_RXUF | \
                               USART_IEN_TXUF | USART_IEN_PERR | \
                               USART_IEN_FERR)
#ifdef CONFIG_DEBUG
#  define EFM32_TX_INTS       (USART_IEN_TXBL | EFM32_TXERR_INTS)
#  define EFM32_RX_INTS       (USART_IEN_RXDATAV | EFM32_RXERR_INTS)
#else
#  define EFM32_TX_INTS        USART_IEN_TXBL
#  define EFM32_RX_INTS        USART_IEN_RXDATAV
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct efm32_config_s
{
  uintptr_t uartbase;  /* Base address of UART registers */
  xcpt_t    rxhandler; /* RX interrupt handler */
  xcpt_t    txhandler; /* TX interrupt handler */
  uint32_t  baud;      /* Configured baud */
  uint8_t   rxirq;     /* RX IRQ associated with this UART (for enable) */
  uint8_t   txirq;     /* TX IRQ associated with this UART (for enable) */
  uint8_t   parity;    /* 0=none, 1=odd, 2=even */
  uint8_t   bits;      /* Number of bits (8 or 9) */
  bool      stop2;     /* True: 2 stop bits */
};

struct efm32_usart_s
{
  const struct efm32_config_s *config;
  uint16_t  ien;       /* Interrupts enabled */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t efm32_serialin(struct efm32_usart_s *priv, int offset);
static inline void efm32_serialout(struct efm32_usart_s *priv, int offset, 
                                   uint32_t value);
static inline void efm32_setuartint(struct efm32_usart_s *priv);

static void efm32_restoreuartint(struct efm32_usart_s *priv, uint32_t ien);
static void efm32_disableuartint(struct efm32_usart_s *priv, uint32_t *ien);
static int  efm32_setup(struct uart_dev_s *dev);
static void efm32_shutdown(struct uart_dev_s *dev);
static int  efm32_attach(struct uart_dev_s *dev);
static void efm32_detach(struct uart_dev_s *dev);
static int  efm32_rxinterrupt(struct uart_dev_s *dev);
#if defined(CONFIG_EFM32_USART0)
static int  efm32_usart0_rxinterrupt(int irq, void *context);
#endif
#if defined(CONFIG_EFM32_USART1)
static int  efm32_usart1_rxinterrupt(int irq, void *context);
#endif
#if defined(CONFIG_EFM32_USART2)
static int  efm32_usart2_rxinterrupt(int irq, void *context);
#endif
#if defined(CONFIG_EFM32_UART0)
static int  efm32_uart0_rxinterrupt(int irq, void *context);
#endif
#if defined(CONFIG_EFM32_UART1)
static int  efm32_uart1_rxinterrupt(int irq, void *context);
#endif
static int  efm32_txinterrupt(struct uart_dev_s *dev);
#if defined(CONFIG_EFM32_USART0)
static int  efm32_usart0_txinterrupt(int irq, void *context);
#endif
#if defined(CONFIG_EFM32_USART1)
static int  efm32_usart1_txinterrupt(int irq, void *context);
#endif
#if defined(CONFIG_EFM32_USART2)
static int  efm32_usart2_txinterrupt(int irq, void *context);
#endif
#if defined(CONFIG_EFM32_UART0)
static int  efm32_uart0_txinterrupt(int irq, void *context);
#endif
#if defined(CONFIG_EFM32_UART1)
static int  efm32_uart1_txinterrupt(int irq, void *context);
#endif
static int  efm32_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  efm32_receive(struct uart_dev_s *dev, uint32_t *status);
static void efm32_rxint(struct uart_dev_s *dev, bool enable);
static bool efm32_rxavailable(struct uart_dev_s *dev);
static void efm32_send(struct uart_dev_s *dev, int ch);
static void efm32_txint(struct uart_dev_s *dev, bool enable);
static bool efm32_txready(struct uart_dev_s *dev);
static bool efm32_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
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

#ifdef CONFIG_EFM32_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_EFM32_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef CONFIG_EFM32_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef CONFIG_EFM32_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_EFM32_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the EFM32 USART0 port. */

#ifdef CONFIG_EFM32_USART0
static const struct efm32_usart_s g_usart0config =
{
  .uartbase  = EFM32_USART0_BASE,
  .rxhandler = efm32_usart0_rxinterrupt,
  .txhandler = efm32_usart0_txinterrupt,
  .baud      = CONFIG_USART0_BAUD,
  .rxirq     = EFM32_IRQ_USART0_RX,
  .txirq     = EFM32_IRQ_USART0_TX,
  .parity    = CONFIG_USART0_PARITY,
  .bits      = CONFIG_USART0_BITS,
  .stop2     = CONFIG_USART0_2STOP,
};

static struct efm32_usart_s g_usart0priv =
{
  .config    = &g_usart0config,
};

static struct uart_dev_s g_usart0port =
{
  .recv      =
  {
    .size    = CONFIG_USART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_USART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
   },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

/* This describes the state of the EFM32 USART1 port. */

#ifdef CONFIG_EFM32_USART1
static struct efm32_config_s g_usart1config =
{
  .uartbase  = EFM32_USART1_BASE,
  .rxhandler = efm32_usart1_rxinterrupt,
  .rxhandler = efm32_usart1_txinterrupt,
  .baud      = CONFIG_USART1_BAUD,
  .rxirq     = EFM32_IRQ_USART1_RX,
  .txirq     = EFM32_IRQ_USART1_TX,
  .parity    = CONFIG_USART1_PARITY,
  .bits      = CONFIG_USART1_BITS,
  .stop2     = CONFIG_USART1_2STOP,
};

static struct efm32_usart_s g_usart1priv =
{
  .config    = &g_usart1config,
};

static struct uart_dev_s g_usart1port =
{
  .recv      =
  {
    .size    = CONFIG_USART1_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_USART1_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
   },
  .ops       = &g_uart_ops,
  .priv      = &g_uart1priv,
};
#endif

/* This describes the state of the EFM32 USART2 port. */

#ifdef CONFIG_EFM32_USART2
static struct efm32_config_s g_usart2config =
{
  .uartbase  = EFM32_USART2_BASE,
  .rxhandler = efm32_usart2_rxinterrupt,
  .txhandler = efm32_usart2_txinterrupt,
  .baud      = CONFIG_USART2_BAUD,
  .rxirq     = EFM32_IRQ_USART2_RX,
  .txirq     = EFM32_IRQ_USART2_TX,
  .parity    = CONFIG_USART2_PARITY,
  .bits      = CONFIG_USART2_BITS,
  .stop2     = CONFIG_USART2_2STOP,
};

static struct efm32_usart_s g_usart2priv =
{
  .config    = &g_usart2config,
};

static struct uart_dev_s g_usart2port =
{
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

/* This describes the state of the EFM32 UART0 port. */

#ifdef CONFIG_EFM32_UART0
static struct efm32_config_s g_uart0config =
{
  .uartbase  = EFM32_UART0_BASE,
  .rxhandler = efm32_uart0_rxinterrupt,
  .txhandler = efm32_uart0_txinterrupt,
  .baud      = CONFIG_UART0_BAUD,
  .rxirq     = EFM32_IRQ_UART0_RX,
  .txirq     = EFM32_IRQ_UART0_TX,
  .parity    = CONFIG_UART0_PARITY,
  .bits      = CONFIG_UART0_BITS,
  .stop2     = CONFIG_UART0_2STOP,
};

static struct efm32_usart_s g_uart0priv =
{
  .config    = &g_uart0config,
};

static struct uart_dev_s g_uart0port =
{
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
   },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

/* This describes the state of the EFM32 UART1 port. */

#ifdef CONFIG_EFM32_UART1
static struct efm32_usart_s g_uart1config =
{
  .uartbase  = EFM32_UART1_BASE,
  .rxhandler = efm32_uart1_rxinterrupt,
  .txhandler = efm32_uart1_txinterrupt,
  .baud      = CONFIG_UART1_BAUD,
  .rxirq     = EFM32_IRQ_UART1_RX,
  .txirq     = EFM32_IRQ_UART1_TX,
  .parity    = CONFIG_UART1_PARITY,
  .bits      = CONFIG_UART1_BITS,
  .stop2     = CONFIG_UART1_2STOP,
};

static struct efm32_usart_s g_uart1priv =
{
  .config    = &g_uart1config,
};

static struct uart_dev_s g_uart1port =
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
 * Name: efm32_serialin
 ****************************************************************************/

static inline uint32_t efm32_serialin(struct efm32_usart_s *priv, int offset)
{
  return getreg32(priv->config->uartbase + offset);
}

/****************************************************************************
 * Name: efm32_serialout
 ****************************************************************************/

static inline void efm32_serialout(struct efm32_usart_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->config->uartbase + offset);
}

/****************************************************************************
 * Name: efm32_setuartint
 ****************************************************************************/

static inline void efm32_setuartint(struct efm32_usart_s *priv)
{
  efm32_serialout(priv, EFM32_USART_IEN_OFFSET, priv->ien);
}

/****************************************************************************
 * Name: efm32_restoreuartint
 ****************************************************************************/

static void efm32_restoreuartint(struct efm32_usart_s *priv, uint32_t ien)
{
  irqstate_t flags;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in ien */

  flags     = irqsave();
  priv->ien = ien;
  efm32_setuartint(priv);
  irqrestore(flags);
}

/****************************************************************************
 * Name: efm32_disableuartint
 ****************************************************************************/

static void efm32_disableuartint(struct efm32_usart_s *priv, uint32_t *ien)
{
  irqstate_t flags;

  flags = irqsave();
  if (ien)
   {
     *ien = priv->ien;
   }

  efm32_restoreuartint(priv, 0);
  irqrestore(flags);
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
 struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
 uint32_t regval;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
 const struct efm32_config_s *config = priv->config;

 /* Configure the UART as an RS-232 UART */

  efm32_uartconfigure(config->uartbase, config->baud, config->parity,
                      config->bits, config->stop2);
#endif

  /* Make sure that all interrupts are disabled */

  efm32_restoreuartint(priv, 0);

  /* Set the TXIL bit in the USART CTRL register.  This will cause TXBL
   * interrupts when the TX buffer is half full.
   */

  regval = efm32_serialin(priv, EFM32_USART_CTRL_OFFSET);
  regval |= USART_CTRL_TXBIL_HALFFULL;
  efm32_serialout(priv, EFM32_USART_CTRL_OFFSET, regval);
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
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;

  /* Disable interrupts */

  efm32_restoreuartint(priv, 0);

  /* Reset the USART/UART by disabling it and restoring all of the registers
   * to the initial, reset value.  Only the ROUTE data set by efm32_lowsetup
   * is preserved.
   */

  efm32_uart_reset(priv->config->uartbase);
}

/****************************************************************************
 * Name: efm32_attach
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

static int efm32_attach(struct uart_dev_s *dev)
{
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
  const struct efm32_config_s *config = priv->config;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(config->rxirq, config->rxhandler);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(config->txirq, config->txhandler);
  if (ret < 0)
    {
      irq_detach(config->rxirq);
      return ret;
    }

  up_enable_irq(config->rxirq);
  up_enable_irq(config->txirq);
  return ret;
}

/****************************************************************************
 * Name: efm32_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void efm32_detach(struct uart_dev_s *dev)
{
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
  const struct efm32_config_s *config = priv->config;

  /* Disable interrupts */

  efm32_restoreuartint(priv, 0);
  up_disable_irq(config->rxirq);
  up_disable_irq(config->txirq);

  /* Detach from the interrupt(s) */

  irq_detach(config->rxirq);
  irq_detach(config->txirq);
}

/****************************************************************************
 * Name: efm32_rxinterrupt
 *
 * Description:
 *   This is the common UART RX interrupt handler.
 *
 ****************************************************************************/

static int  efm32_rxinterrupt(struct uart_dev_s *dev)
{
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
  uint32_t intflags;

  DEBUGASSERT(priv);

  /* Read the interrupt flags register */

  intflags = efm32_serialin(priv, EFM32_USART_IF_OFFSET);

  /* Check if the receive data is available is full (RXDATAV). */

  if ((intflags & USART_IEN_RXDATAV) != 0)
    {
      /* Process incoming bytes */

      uart_recvchars(dev);
    }

#ifdef CONFIG_DEBUG
  /* Check for receive errors */

  if ((intflags & EFM32_RXERR_INTS) != 0)
    {
      /* RXOF - RX Overflow Interrupt Enable
       * RXUF - RX Underflow Interrupt Enable
       * TXUF - TX Underflow Interrupt Enable
       * PERR - Parity Error Interrupt Enable
       * FERR - Framing Error Interrupt Enable
       */

      lldbg("RX ERROR: %08x\n", intflags);
    }
#endif

  /* Clear pending interrupts by writing to the interrupt flag clear
   * register.
   */

  efm32_serialout(priv, EFM32_USART_IFC_OFFSET, intflags & EFM32_RX_INTS);
  return OK;
}

#if defined(CONFIG_EFM32_USART0)
static int efm32_usart0_rxinterrupt(int irq, void *context)
{
  return efm32_rxinterrupt(&g_usart0port);
}
#endif

#if defined(CONFIG_EFM32_USART1)
static int  efm32_usart1_rxinterrupt(int irq, void *context)
{
  return efm32_rxinterrupt(&g_usart1port);
}
#endif

#if defined(CONFIG_EFM32_USART2)
static int  efm32_usart2_rxinterrupt(int irq, void *context)
{
  return efm32_rxinterrupt(&g_usart2port);
}
#endif

#if defined(CONFIG_EFM32_UART0)
static int  efm32_uart0_rxinterrupt(int irq, void *context)
{
  return efm32_rxinterrupt(&g_uart0port);
}
#endif

#if defined(CONFIG_EFM32_UART1)
static int  efm32_uart1_rxinterrupt(int irq, void *context)
{
  return efm32_rxinterrupt(&g_uart1port);
}
#endif

/****************************************************************************
 * Name: efm32_txinterrupt
 *
 * Description:
 *   This is the common UART TX interrupt handler.
 *
 ****************************************************************************/

static int  efm32_txinterrupt(struct uart_dev_s *dev)
{
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
  uint32_t intflags;

  DEBUGASSERT(priv);

  /* Read the interrupt flags register */

  intflags = efm32_serialin(priv, EFM32_USART_IF_OFFSET);

  /* Check if the transmit data buffer became half full */

  if ((intflags & USART_IEN_TXBL) != 0)
    {
      /* Process outgoing bytes */

      uart_xmitchars(dev);
    }

#ifdef CONFIG_DEBUG
  /* Check for transmit errors */

  if ((intflags & EFM32_TXERR_INTS) != 0)
    {
      /* TXOF - TX Overflow Interrupt Enable */

      lldbg("RX ERROR: %08x\n", intflags);
    }
#endif

  /* Clear pending interrupts by writing to the interrupt flag clear
   * register.  We won't clear RX errors until they have been reported.
   */

  efm32_serialout(priv, EFM32_USART_IFC_OFFSET, intflags & EFM32_TX_INTS);
  return OK;
}

#if defined(CONFIG_EFM32_USART0)
static int efm32_usart0_txinterrupt(int irq, void *context)
{
  return efm32_txinterrupt(&g_usart0port);
}
#endif

#if defined(CONFIG_EFM32_USART1)
static int  efm32_usart1_txinterrupt(int irq, void *context)
{
  return efm32_txinterrupt(&g_usart1port);
}
#endif

#if defined(CONFIG_EFM32_USART2)
static int  efm32_usart2_txinterrupt(int irq, void *context)
{
  return efm32_txinterrupt(&g_usart2port);
}
#endif

#if defined(CONFIG_EFM32_UART0)
static int  efm32_uart0_txinterrupt(int irq, void *context)
{
  return efm32_txinterrupt(&g_uart0port);
}
#endif

#if defined(CONFIG_EFM32_UART1)
static int  efm32_uart1_txinterrupt(int irq, void *context)
{
  return efm32_txinterrupt(&g_uart1port);
}
#endif

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
  struct efm32_usart_s *priv;
  int ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv)
  priv = (struct efm32_usart_s*)dev->priv;

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

static int efm32_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
  uint32_t rxdatax;

  /* Get error status information:
   *
   * FE: Framing error. To clear FE, read S1 with FE set and then read
   *     read UART data register (D).
   * NF: Noise flag. To clear NF, read S1 and then read the UART data
   *     register (D).
   * PF: Parity error flag. To clear PF, read S1 and then read the UART
   *     data register (D).
   */

  rxdatax = efm32_serialin(priv, EFM32_USART_RXDATAX_OFFSET);

  /* Return status information */

  if (status)
    {
      *status = rxdatax;
    }

  /* Then return the actual received byte.  Reading S1 then D clears all
   * RX errors.
   */

  return (int)(rxdatax & _USART_RXDATAX_RXDATA_MASK);
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
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
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

  irqrestore(flags);
}

/****************************************************************************
 * Name: efm32_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool efm32_rxavailable(struct uart_dev_s *dev)
{
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;

  /* Return true if the receive data is available is full (RXDATAV). */

  return (efm32_serialin(priv, EFM32_USART_STATUS_OFFSET) & USART_STATUS_RXDATAV) != 0;
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
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
  efm32_serialout(priv, EFM32_USART_TXDATA_OFFSET, (uint32_t)ch);
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
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
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

  irqrestore(flags);
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
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;

  /* The TX Buffer Level (TXBL) status bit indicates the level of the
   * transmit buffer.  If TXBIL is set, TXBL is set whenever the transmit
   * buffer is half-full or empty.
   */

  return (efm32_serialin(priv, EFM32_USART_STATUS_OFFSET) & USART_STATUS_TXBL) != 0;
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
  struct efm32_usart_s *priv = (struct efm32_usart_s*)dev->priv;

  /* TX Complete (TXC) is set when a transmission has completed and no more
   * data is available in the transmit buffer. Cleared when data is written
   * to the transmit buffer
   */

  return (efm32_serialin(priv, EFM32_USART_STATUS_OFFSET) & USART_STATUS_TXC) != 0;
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
 *   configuration performed in efm32_consoleinit() and main clock iniialization
 *   performed in efm32_clkinitialize().
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Disable interrupts from all UARTS.  The console is enabled in
   * pic32mx_consoleinit()
   */

  efm32_restoreuartint(TTYS0_DEV.priv, 0);
#ifdef TTYS1_DEV
  efm32_restoreuartint(TTYS1_DEV.priv, 0);
#endif
#ifdef TTYS2_DEV
  efm32_restoreuartint(TTYS2_DEV.priv, 0);
#endif
#ifdef TTYS3_DEV
  efm32_restoreuartint(TTYS3_DEV.priv, 0);
#endif
#ifdef TTYS4_DEV
  efm32_restoreuartint(TTYS4_DEV.priv, 0);
#endif

  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  efm32_setup(&CONSOLE_DEV);
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
  /* Register the console */

#ifdef CONSOLE_DEV
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
  struct efm32_usart_s *priv = (struct efm32_usart_s*)CONSOLE_DEV.priv;
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

      efm32_lowputc('\r');
    }

  efm32_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
