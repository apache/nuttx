/****************************************************************************
 * arch/arm/src/xmc4/xmc4_serial.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "xmc4_config.h"
#include "chip/xmc4_usic.h"
#include "xmc4_lowputc.h"

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
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart3port /* UART3 is console */
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart4port /* UART4 is console */
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart5port /* UART5 is console */
#    define TTYS5_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(HAVE_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(HAVE_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(HAVE_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(HAVE_UART3)
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  elif defined(HAVE_UART4)
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  elif defined(HAVE_UART5)
#    define TTYS0_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-5 excluding the console UART. */

#if defined(HAVE_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(HAVE_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(HAVE_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(HAVE_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(HAVE_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#elif defined(HAVE_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS1_DEV           g_uart5port /* UART5 is ttyS1 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART1-5. It can't be UART0 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-5 could also be the
 * console.
 */

#if defined(HAVE_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(HAVE_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(HAVE_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(HAVE_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#elif defined(HAVE_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS2_DEV           g_uart5port /* UART5 is ttyS2 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART2-5. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART 2-5 could also be the console.
 */

#if defined(HAVE_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS3_DEV           g_uart2port /* UART2 is ttyS3 */
#  define UART2_ASSIGNED      1
#elif defined(HAVE_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(HAVE_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#elif defined(HAVE_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS3_DEV           g_uart5port /* UART5 is ttyS3 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys4. This could be one of UART3-5. It can't be UART0-2 because
 * those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * UART 3-5 could also be the console.
 */

#if defined(HAVE_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS4_DEV           g_uart3port /* UART3 is ttyS4 */
#  define UART3_ASSIGNED      1
#elif defined(HAVE_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#elif defined(HAVE_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS4_DEV           g_uart5port /* UART5 is ttyS4 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys5. This could be one of UART4-5. It can't be UART0-3 because
 * those have already been assigned to ttsyS0, 1, 2, 3 or 4.  One of
 * UART 4-5 could also be the console.
 */

#if defined(HAVE_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS5_DEV           g_uart4port /* UART4 is ttyS5 */
#  define UART4_ASSIGNED      1
#elif defined(HAVE_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS5_DEV           g_uart5port /* UART5 is ttyS5 */
#  define UART5_ASSIGNED      1
#endif

/* Event sets */

#ifdef CONFIG_DEBUG_FEATURES
#  define CCR_RX_EVENTS       (USIC_CCR_RIEN | USIC_CCR_AIEN | USIC_CCR_DLIEN)
#else
#  define CCR_RX_EVENTS       (USIC_CCR_RIEN | USIC_CCR_AIEN)
#endif

#define CCR_TX_EVENTS         (USIC_CCR_TBIEN)
#define CCR_ALL_EVENTS        (CCR_RX_EVENTS | CCR_TX_EVENTS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one UART device */

struct xmc4_dev_s
{
  uintptr_t uartbase;  /* Base address of UART registers */
  uint8_t   channel;   /* USIC channel identification */
  uint8_t   irq;       /* Status IRQ associated with this UART (for enable) */
  uint8_t   ccr;       /* Interrupts enabled in CCR */

  /* UART configuration */

  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  xmc4_setup(struct uart_dev_s *dev);
static void xmc4_shutdown(struct uart_dev_s *dev);
static int  xmc4_attach(struct uart_dev_s *dev);
static void xmc4_detach(struct uart_dev_s *dev);
static int  xmc4_interrupt(int irq, void *context, FAR void *arg);
static int  xmc4_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  xmc4_receive(struct uart_dev_s *dev, uint32_t *status);
static void xmc4_rxint(struct uart_dev_s *dev, bool enable);
static bool xmc4_rxavailable(struct uart_dev_s *dev);
static void xmc4_send(struct uart_dev_s *dev, int ch);
static void xmc4_txint(struct uart_dev_s *dev, bool enable);
static bool xmc4_txready(struct uart_dev_s *dev);
static bool xmc4_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = xmc4_setup,
  .shutdown       = xmc4_shutdown,
  .attach         = xmc4_attach,
  .detach         = xmc4_detach,
  .ioctl          = xmc4_ioctl,
  .receive        = xmc4_receive,
  .rxint          = xmc4_rxint,
  .rxavailable    = xmc4_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = xmc4_send,
  .txint          = xmc4_txint,
  .txready        = xmc4_txready,
  .txempty        = xmc4_txempty,
};

/* I/O buffers */

#ifdef HAVE_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef HAVE_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef HAVE_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif
#ifdef HAVE_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif
#ifdef HAVE_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif
#ifdef HAVE_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
#endif

/* This describes the state of the XMC4 UART0 port. */

#ifdef HAVE_UART0
static struct xmc4_dev_s g_uart0priv =
{
  .uartbase       = XMC4_USIC0_CH0_BASE,
  .channel        = (uint8_t)USIC0_CHAN0,
  .irq            = XMC4_IRQ_USIC0_SR0,
  .config         =
  {
    .baud         = CONFIG_UART0_BAUD,
    .dx           = BOARD_UART0_DX,
    .parity       = CONFIG_UART0_PARITY,
    .nbits        = CONFIG_UART0_BITS,
    .stop2        = CONFIG_UART0_2STOP,
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

/* This describes the state of the XMC4 UART1 port. */

#ifdef HAVE_UART1
static struct xmc4_dev_s g_uart1priv =
{
  .uartbase       = XMC4_USIC0_CH1_BASE,
  .channel        = (uint8_t)USIC0_CHAN1,
  .irq            = XMC4_IRQ_USIC0_SR1,
  .config         =
  {
    .baud         = CONFIG_UART1_BAUD,
    .dx           = BOARD_UART1_DX,
    .parity       = CONFIG_UART1_PARITY,
    .nbits        = CONFIG_UART1_BITS,
    .stop2        = CONFIG_UART1_2STOP,
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

/* This describes the state of the XMC4 UART2 port. */

#ifdef HAVE_UART2
static struct xmc4_dev_s g_uart2priv =
{
  .uartbase       = XMC4_USIC1_CH0_BASE,
  .channel        = (uint8_t)USIC1_CHAN0,
  .irq            = XMC4_IRQ_USIC1_SR0,
  .config         =
  {
    .baud         = CONFIG_UART2_BAUD,
    .dx           = BOARD_UART2_DX,
    .parity       = CONFIG_UART2_PARITY,
    .nbits        = CONFIG_UART2_BITS,
    .stop2        = CONFIG_UART2_2STOP,
  }
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

/* This describes the state of the XMC4 UART3 port. */

#ifdef HAVE_UART3
static struct xmc4_dev_s g_uart3priv =
{
  .uartbase       = XMC4_USIC1_CH1_BASE,
  .channel        = (uint8_t)USIC1_CHAN1,
  .irq            = XMC4_IRQ_USIC1_SR1,
  .config         =
  {
    .baud         = CONFIG_UART3_BAUD,
    .dx           = BOARD_UART3_DX,
    .parity       = CONFIG_UART3_PARITY,
    .nbits        = CONFIG_UART3_BITS,
    .stop2        = CONFIG_UART3_2STOP,
  }
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

/* This describes the state of the XMC4 UART4 port. */

#ifdef HAVE_UART4
static struct xmc4_dev_s g_uart4priv =
{
  .uartbase       = XMC4_USIC2_CH0_BASE,
  .channel        = (uint8_t)USIC2_CHAN0,
  .irq            = XMC4_IRQ_USIC2_SR0,
  .config         =
  {
    .baud         = CONFIG_UART4_BAUD,
    .dx           = BOARD_UART4_DX,
    .parity       = CONFIG_UART4_PARITY,
    .nbits        = CONFIG_UART4_BITS,
    .stop2        = CONFIG_UART4_2STOP,
  }
};

static uart_dev_t g_uart4port =
{
  .recv     =
  {
    .size   = CONFIG_UART4_RXBUFSIZE,
    .buffer = g_uart4rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART4_TXBUFSIZE,
    .buffer = g_uart4txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart4priv,
};
#endif

/* This describes the state of the XMC4 UART5 port. */

#ifdef HAVE_UART5
static struct xmc4_dev_s g_uart5priv =
{
  .uartbase       = XMC4_USIC2_CH1_BASE,
  .channel        = (uint8_t)USIC2_CHAN1,
  .irq            = XMC4_IRQ_USIC2_SR1,
  .config         =
  {
    .baud         = CONFIG_UART5_BAUD,
    .dx           = BOARD_UART5_DX,
    .parity       = CONFIG_UART5_PARITY,
    .nbits        = CONFIG_UART5_BITS,
    .stop2        = CONFIG_UART5_2STOP,
  }
};

static uart_dev_t g_uart5port =
{
  .recv     =
  {
    .size   = CONFIG_UART5_RXBUFSIZE,
    .buffer = g_uart5rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART5_TXBUFSIZE,
    .buffer = g_uart5txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart5priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_serialin
 ****************************************************************************/

static inline uint32_t xmc4_serialin(struct xmc4_dev_s *priv,
                                     unsigned int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: xmc4_serialout
 ****************************************************************************/

static inline void xmc4_serialout(struct xmc4_dev_s *priv,
                                  unsigned int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: xmc4_modifyreg
 ****************************************************************************/

static inline void xmc4_modifyreg(struct xmc4_dev_s *priv, unsigned int offset,
                                  uint32_t setbits, uint32_t clrbits)
{
  irqstate_t flags;
  uintptr_t regaddr = priv->uartbase + offset;
  uint32_t regval;

  flags = enter_critical_section();

  regval = getreg32(regaddr);
  regval &= ~clrbits;
  regval |= setbits;
  putreg32(regval, regaddr);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: xmc4_setuartint
 ****************************************************************************/

static void xmc4_setuartint(struct xmc4_dev_s *priv)
{
  irqstate_t flags;

  /* Re-enable/re-disable event interrupts corresponding to the state of
   * bits in priv->ccr.
   */

  flags = enter_critical_section();
  xmc4_modifyreg(priv, XMC4_USIC_CCR_OFFSET, CCR_ALL_EVENTS, priv->ccr);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: xmc4_restoreuartint
 ****************************************************************************/

static void xmc4_restoreuartint(struct xmc4_dev_s *priv, uint32_t ccr)
{
  irqstate_t flags;

  /* Re-enable/re-disable event interrupts corresponding to the state of bits
   * in the ccr argument.
   */

  flags = enter_critical_section();
  priv->ccr = ccr;
  xmc4_setuartint(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: xmc4_disableuartint
 ****************************************************************************/

static void xmc4_disableuartint(struct xmc4_dev_s *priv, uint32_t *ccr)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (ccr)
    {
      *ccr = priv->ccr;
    }

  xmc4_restoreuartint(priv, 0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: xmc4_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int xmc4_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;

  /* Configure the UART as an RS-232 UART */

  xmc4_uart_configure(priv->channel, &priv->config);
#endif

  /* Make sure that all interrupts are disabled */

  xmc4_restoreuartint(priv, 0);
  return OK;
}

/****************************************************************************
 * Name: xmc4_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void xmc4_shutdown(struct uart_dev_s *dev)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;

  /* Disable interrupts */

  xmc4_restoreuartint(priv, 0);

  /* Reset hardware and disable Rx and Tx */

  xmc4_uart_disable(priv->channel);
}

/****************************************************************************
 * Name: xmc4_attach
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

static int xmc4_attach(struct uart_dev_s *dev)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(priv->irq, xmc4_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: xmc4_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void xmc4_detach(struct uart_dev_s *dev)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;

  /* Disable interrupts */

  xmc4_restoreuartint(priv, 0);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: xmc4_interrupt
 *
 * Description:
 *   This is the UART status interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int xmc4_interrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct xmc4_dev_s *priv;
  int passes;
  uint32_t regval;
  bool handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct xmc4_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Handle incoming, receive bytes.
       * Check if the received FIFO is not empty.
       */

      regval = xmc4_serialin(priv, XMC4_USIC_TRBSR_OFFSET);
      if ((regval & USIC_TRBSR_REMPTY) == 0)
        {
          /* Process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes.
       * Check if the received FIFO is not full.
       */

      regval = xmc4_serialin(priv, XMC4_USIC_TRBSR_OFFSET);
      if ((regval & USIC_TRBSR_TFULL) == 0)
        {
          /* Process outgoing bytes */

          uart_xmitchars(dev);
          handled = true;
        }

#ifdef CONFIG_DEBUG_FEATURES
      /* Check for error conditions */
#warning Misssing logic
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: xmc4_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int xmc4_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  struct inode      *inode;
  struct uart_dev_s *dev;
  struct xmc4_dev_s   *priv;
  int                ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv);
  priv = (struct xmc4_dev_s *)dev->priv;

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
 * Name: xmc4_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int xmc4_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;
  uint32_t outr;

  /* Get input data along with receiver control information */

  outr = xmc4_serialin(priv, XMC4_USIC_OUTR_OFFSET);

  /* Return receiver control information */

  if (status)
    {
      *status = outr >> USIC_OUTR_RCI_SHIFT;
    }

  /* Then return the actual received data. */

  return outr & USIC_OUTR_DSR_MASK;
}

/****************************************************************************
 * Name: xmc4_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void xmc4_rxint(struct uart_dev_s *dev, bool enable)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

      priv->ccr |= CCR_RX_EVENTS;
      xmc4_setuartint(priv);
#endif
    }
  else
    {
      priv->ccr &= ~CCR_RX_EVENTS;
      xmc4_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: xmc4_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool xmc4_rxavailable(struct uart_dev_s *dev)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the receive buffer/fifo is not "empty." */

  regval = xmc4_serialin(priv, XMC4_USIC_TRBSR_OFFSET);
  return ((regval & USIC_TRBSR_REMPTY) == 0);
}

/****************************************************************************
 * Name: xmc4_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void xmc4_send(struct uart_dev_s *dev, int ch)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;
  xmc4_serialout(priv, XMC4_USIC_IN_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: xmc4_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void xmc4_txint(struct uart_dev_s *dev, bool enable)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the TX interrupt */

      priv->ccr |= CCR_TX_EVENTS;
      xmc4_setuartint(priv);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->ccr &= ~CCR_TX_EVENTS;
      xmc4_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: xmc4_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool xmc4_txready(struct uart_dev_s *dev)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the transmit buffer/fifo is "not full." */

  regval = xmc4_serialin(priv, XMC4_USIC_TRBSR_OFFSET);
  return ((regval & USIC_TRBSR_TFULL) == 0);
}

/****************************************************************************
 * Name: xmc4_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool xmc4_txempty(struct uart_dev_s *dev)
{
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the transmit buffer/fifo is "empty." */

  regval = xmc4_serialin(priv, XMC4_USIC_TRBSR_OFFSET);
  return ((regval & USIC_TRBSR_TEMPTY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before xmc4_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in xmc_lowsetup() and main clock iniialization
 *   performed in xmc_clock_configure().
 *
 ****************************************************************************/

#if defined(USE_EARLYSERIALINIT)
void xmc4_earlyserialinit(void)
{
  /* Disable interrupts from all UARTS.  The console is enabled in
   * pic32mx_consoleinit()
   */

  xmc4_restoreuartint(TTYS0_DEV.priv, 0);
#ifdef TTYS1_DEV
  xmc4_restoreuartint(TTYS1_DEV.priv, 0);
#endif
#ifdef TTYS2_DEV
  xmc4_restoreuartint(TTYS2_DEV.priv, 0);
#endif
#ifdef TTYS3_DEV
  xmc4_restoreuartint(TTYS3_DEV.priv, 0);
#endif
#ifdef TTYS4_DEV
  xmc4_restoreuartint(TTYS4_DEV.priv, 0);
#endif
#ifdef TTYS5_DEV
  xmc4_restoreuartint(TTYS5_DEV.priv, 0);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  xmc4_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that xmc4_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_serialinit(void)
{
#ifdef HAVE_UART_CONSOLE
  /* Register the serial console */

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
#ifdef HAVE_UART_CONSOLE
  struct xmc4_dev_s *priv = (struct xmc4_dev_s *)CONSOLE_DEV.priv;
  uint32_t ccr;

  xmc4_disableuartint(priv, &ccr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  xmc4_restoreuartint(priv, ccr);
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

      up_lowputc('\r');
    }

  up_lowputc(ch);
  return ch;
}
#endif

#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER */
