/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_serial.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/power/pm.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/s32k1xx_lpuart.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_config.h"
#include "s32k1xx_pin.h"
#include "s32k1xx_lowputc.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which LPUART with be tty0/console and which tty1-7?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered LPUART.
 */

/* First pick the console and ttys0.  This could be any of LPUART0-2 */

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_uart0port /* LPUART0 is console */
#  define TTYS0_DEV           g_uart0port /* LPUART0 is ttyS0 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_uart1port /* LPUART1 is console */
#  define TTYS0_DEV           g_uart1port /* LPUART1 is ttyS0 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_uart2port /* LPUART2 is console */
#  define TTYS0_DEV           g_uart2port /* LPUART2 is ttyS0 */
#  define UART3_ASSIGNED      1
#else
#  undef CONSOLE_DEV                      /* No console */
#  if defined(CONFIG_S32K1XX_LPUART0)
#    define TTYS0_DEV         g_uart0port /* LPUART0 is ttyS0 */
#    define UART1_ASSIGNED    1
#  elif defined(CONFIG_S32K1XX_LPUART1)
#    define TTYS0_DEV         g_uart1port /* LPUART1 is ttyS0 */
#    define UART2_ASSIGNED    1
#  elif defined(CONFIG_S32K1XX_LPUART2)
#    define TTYS0_DEV         g_uart2port /* LPUART2 is ttyS0 */
#    define UART3_ASSIGNED    1
#  endif
#endif

/* Pick ttys1.  This could be any of LPUART0-2 excluding the console UART.
 * One of LPUART0-8 could be the console; one of UART0-2 has already been
 * assigned to ttys0.
 */

#if defined(CONFIG_S32K1XX_LPUART0) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* LPUART0 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_S32K1XX_LPUART1) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* LPUART1 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_S32K1XX_LPUART2) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* LPUART2 is ttyS1 */
#  define UART3_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of LPUART0-2. It can't be LPUART0 because
 * that was either assigned as ttyS0 or ttys1.  One of LPUART0-2 could be the
 * console.  One of UART1-2 has already been assigned to ttys0 or ttyS1.
 */

#if defined(CONFIG_S32K1XX_LPUART1) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* LPUART1 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_S32K1XX_LPUART2) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* LPUART2 is ttyS2 */
#  define UART3_ASSIGNED      1
#endif

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_S32K1XX_PM_SERIAL_ACTIVITY)
#  define CONFIG_S32K1XX_PM_SERIAL_ACTIVITY 10
#endif

#if defined(CONFIG_PM)
#  define PM_IDLE_DOMAIN      0 /* Revisit */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct s32k1xx_uart_s
{
  uint32_t uartbase;        /* Base address of UART registers */
  uint32_t baud;            /* Configured baud */
  uint32_t ie;              /* Saved enabled interrupts */
  uint8_t  irq;             /* IRQ associated with this UART */
  uint8_t  parity;          /* 0=none, 1=odd, 2=even */
  uint8_t  bits;            /* Number of bits (7 or 8) */
#if defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)
  uint8_t  inviflow:1;      /* Invert RTS sense */
  const uint32_t rts_gpio;  /* U[S]ART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t cts_gpio;  /* U[S]ART CTS GPIO pin configuration */
#endif

  uint8_t  stopbits2:1;     /* 1: Configure with 2 stop bits vs 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t  iflow:1;         /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t  oflow:1;         /* output flow control (CTS) enabled */
#endif
#ifdef CONFIG_SERIAL_RS485CONTROL
  uint8_t rs485mode:1;      /* We are in RS485 (RTS on TX) mode */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t s32k1xx_serialin(struct s32k1xx_uart_s *priv,
                                      uint32_t offset);
static inline void s32k1xx_serialout(struct s32k1xx_uart_s *priv,
                                   uint32_t offset, uint32_t value);
static inline void s32k1xx_disableuartint(struct s32k1xx_uart_s *priv,
                                        uint32_t *ie);
static inline void s32k1xx_restoreuartint(struct s32k1xx_uart_s *priv,
                                        uint32_t ie);

static int  s32k1xx_setup(struct uart_dev_s *dev);
static void s32k1xx_shutdown(struct uart_dev_s *dev);
static int  s32k1xx_attach(struct uart_dev_s *dev);
static void s32k1xx_detach(struct uart_dev_s *dev);
static int  s32k1xx_interrupt(int irq, void *context, FAR void *arg);
static int  s32k1xx_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  s32k1xx_receive(struct uart_dev_s *dev, uint32_t *status);
static void s32k1xx_rxint(struct uart_dev_s *dev, bool enable);
static bool s32k1xx_rxavailable(struct uart_dev_s *dev);
static void s32k1xx_send(struct uart_dev_s *dev, int ch);
static void s32k1xx_txint(struct uart_dev_s *dev, bool enable);
static bool s32k1xx_txready(struct uart_dev_s *dev);
static bool s32k1xx_txempty(struct uart_dev_s *dev);

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int dowmin,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup          = s32k1xx_setup,
  .shutdown       = s32k1xx_shutdown,
  .attach         = s32k1xx_attach,
  .detach         = s32k1xx_detach,
  .ioctl          = s32k1xx_ioctl,
  .receive        = s32k1xx_receive,
  .rxint          = s32k1xx_rxint,
  .rxavailable    = s32k1xx_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = s32k1xx_send,
  .txint          = s32k1xx_txint,
  .txready        = s32k1xx_txready,
  .txempty        = s32k1xx_txempty,
};

/* I/O buffers */

#ifdef CONFIG_S32K1XX_LPUART0
static char g_uart0rxbuffer[CONFIG_LPUART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_LPUART0_TXBUFSIZE];
#endif

#ifdef CONFIG_S32K1XX_LPUART1
static char g_uart1rxbuffer[CONFIG_LPUART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_LPUART1_TXBUFSIZE];
#endif

#ifdef CONFIG_S32K1XX_LPUART2
static char g_uart2rxbuffer[CONFIG_LPUART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_LPUART2_TXBUFSIZE];
#endif

/* This describes the state of the S32K1XX lpuart0 port. */

#ifdef CONFIG_S32K1XX_LPUART0
static struct s32k1xx_uart_s g_uart0priv =
{
  .uartbase     = S32K1XX_LPUART0_BASE,
  .baud         = CONFIG_LPUART0_BAUD,
  .irq          = S32K1XX_IRQ_LPUART0,
  .parity       = CONFIG_LPUART0_PARITY,
  .bits         = CONFIG_LPUART0_BITS,
  .stopbits2    = CONFIG_LPUART0_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART0_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART0_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART0_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART0_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART0_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART0_RTS,
#endif

#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART0_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART0_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif
};

static struct uart_dev_s g_uart0port =
{
  .recv         =
  {
    .size       = CONFIG_LPUART0_RXBUFSIZE,
    .buffer     = g_uart0rxbuffer,
  },
  .xmit         =
  {
    .size       = CONFIG_LPUART0_TXBUFSIZE,
    .buffer     = g_uart0txbuffer,
  },
  .ops          = &g_uart_ops,
  .priv         = &g_uart0priv,
};
#endif

/* This describes the state of the S32K1XX lpuart1 port. */

#ifdef CONFIG_S32K1XX_LPUART1
static struct s32k1xx_uart_s g_uart1priv =
{
  .uartbase     = S32K1XX_LPUART1_BASE,
  .baud         = CONFIG_LPUART1_BAUD,
  .irq          = S32K1XX_IRQ_LPUART1,
  .parity       = CONFIG_LPUART1_PARITY,
  .bits         = CONFIG_LPUART1_BITS,
  .stopbits2    = CONFIG_LPUART1_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART1_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART1_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART1_RTS,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART1_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif
};

static struct uart_dev_s g_uart1port =
{
  .recv           =
    {
      .size       = CONFIG_LPUART1_RXBUFSIZE,
      .buffer     = g_uart1rxbuffer,
    },
  .xmit           =
    {
      .size       = CONFIG_LPUART1_TXBUFSIZE,
      .buffer     = g_uart1txbuffer,
    },
  .ops            = &g_uart_ops,
  .priv           = &g_uart1priv,
};
#endif

#ifdef CONFIG_S32K1XX_LPUART2
static struct s32k1xx_uart_s g_uart2priv =
{
  .uartbase     = S32K1XX_LPUART2_BASE,
  .baud         = CONFIG_LPUART2_BAUD,
  .irq          = S32K1XX_IRQ_LPUART2,
  .parity       = CONFIG_LPUART2_PARITY,
  .bits         = CONFIG_LPUART2_BITS,
  .stopbits2    = CONFIG_LPUART2_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART2_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = GPIO_LPUART2_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)
  .iflow        = 1,
#endif
# if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)))
  .rts_gpio     = GPIO_LPUART2_RTS,
#endif
#if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART2_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#endif

#if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)
  .rs485mode    = 1,
#endif
};

static struct uart_dev_s g_uart2port =
{
  .recv           =
    {
      .size       = CONFIG_LPUART2_RXBUFSIZE,
      .buffer     = g_uart2rxbuffer,
    },
  .xmit           =
    {
      .size       = CONFIG_LPUART2_TXBUFSIZE,
      .buffer     = g_uart2txbuffer,
    },
  .ops            = &g_uart_ops,
  .priv           = &g_uart2priv,
};
#endif

#ifdef CONFIG_PM
static  struct pm_callback_s g_serial_pmcb =
{
  .notify       = up_pm_notify,
  .prepare      = up_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_serialin
 ****************************************************************************/

static inline uint32_t s32k1xx_serialin(struct s32k1xx_uart_s *priv,
                                      uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: s32k1xx_serialout
 ****************************************************************************/

static inline void s32k1xx_serialout(struct s32k1xx_uart_s *priv,
                                     uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: s32k1xx_disableuartint
 ****************************************************************************/

static inline void s32k1xx_disableuartint(struct s32k1xx_uart_s *priv,
                                          uint32_t *ie)
{
  irqstate_t flags;
  uint32_t regval;

  flags  = spin_lock_irqsave();
  regval = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);

  /* Return the current Rx and Tx interrupt state */

  if (ie != NULL)
    {
      *ie = regval & LPUART_ALL_INTS;
    }

  regval &= ~LPUART_ALL_INTS;
  s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: s32k1xx_restoreuartint
 ****************************************************************************/

static inline void s32k1xx_restoreuartint(struct s32k1xx_uart_s *priv,
                                        uint32_t ie)
{
  irqstate_t flags;
  uint32_t regval;

  /* Enable/disable any interrupts that are currently disabled but should be
   * enabled/disabled.
   */

  flags   = spin_lock_irqsave();
  regval  = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= ie;
  s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: s32k1xx_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int s32k1xx_setup(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
#ifndef CONFIG_SUPPRESS_LPUART_CONFIG
  struct uart_config_s config =
    {
      0
    };

  int ret;

  /* Configure the UART */

  config.baud       = priv->baud;       /* Configured baud */
  config.parity     = priv->parity;     /* 0=none, 1=odd, 2=even */
  config.bits       = priv->bits;       /* Number of bits (5-9) */
  config.stopbits2  = priv->stopbits2;  /* true: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  config.usects     = priv->iflow;      /* Flow control on inbound side */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  config.userts     = priv->oflow;      /* Flow control on outbound side */
#endif
#ifdef CONFIG_SERIAL_RS485CONTROL
  config.users485   = priv->rs485mode;  /* Switch into RS485 mode */
#endif
#if defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)
  config.invrts     = priv->inviflow;   /* Inversion of outbound flow control */
#endif

  ret = s32k1xx_lpuart_configure(priv->uartbase, &config);

  priv->ie = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET) & \
             LPUART_ALL_INTS;
  return ret;

#else
  priv->ie = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET) & \
             LPUART_ALL_INTS;
  return OK;
#endif
}

/****************************************************************************
 * Name: s32k1xx_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void s32k1xx_shutdown(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;

  /* Disable the UART */

  s32k1xx_serialout(priv, S32K1XX_LPUART_GLOBAL_OFFSET, LPUART_GLOBAL_RST);
}

/****************************************************************************
 * Name: s32k1xx_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate
 *   in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int s32k1xx_attach(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, s32k1xx_interrupt, dev);
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
 * Name: s32k1xx_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void s32k1xx_detach(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: s32k1xx_interrupt (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It should cal
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ****************************************************************************/

static int s32k1xx_interrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct s32k1xx_uart_s *priv;
  uint32_t usr;
  int passes = 0;
  bool handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct s32k1xx_uart_s *)dev->priv;

#if defined(CONFIG_PM) && CONFIG_S32K1XX_PM_SERIAL_ACTIVITY > 0
  /* Report serial activity to the power management logic */

  pm_activity(PM_IDLE_DOMAIN, CONFIG_S32K1XX_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the current UART status and check for loop
       * termination conditions
       */

      usr  = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
      usr &= (LPUART_STAT_RDRF | LPUART_STAT_TC | LPUART_STAT_OR |
              LPUART_STAT_FE);

      /* Clear serial overrun and framing errors */

      if ((usr & LPUART_STAT_OR) != 0)
        {
          s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET,
                            LPUART_STAT_OR);
        }

      if ((usr & LPUART_STAT_FE) != 0)
        {
          s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET,
                            LPUART_STAT_FE);
        }

      /* Handle incoming, receive bytes */

      if ((usr & LPUART_STAT_RDRF) != 0 &&
          (priv->ie & LPUART_CTRL_RIE) != 0)
        {
          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((usr & LPUART_STAT_TC) != 0 &&
          (priv->ie & LPUART_CTRL_TCIE) != 0)
        {
          uart_xmitchars(dev);
          handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int s32k1xx_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || defined(CONFIG_SERIAL_TERMIOS)
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct s32k1xx_uart_s *user = (struct s32k1xx_uart_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct s32k1xx_uart_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return parity */

        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= (priv->stopbits2) ? CSTOPB : 0;

        /* Return flow control */

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        termiosp->c_cflag |= ((priv->oflow) ? CCTS_OFLOW : 0);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        termiosp->c_cflag |= ((priv->iflow) ? CRTS_IFLOW : 0);
#endif
        /* Return baud */

        cfsetispeed(termiosp, priv->baud);

        /* Return number of bits */

        switch (priv->bits)
          {
          case 5:
            termiosp->c_cflag |= CS5;
            break;

          case 6:
            termiosp->c_cflag |= CS6;
            break;

          case 7:
            termiosp->c_cflag |= CS7;
            break;

          default:
          case 8:
            termiosp->c_cflag |= CS8;
            break;

          case 9:
            termiosp->c_cflag |= CS8 /* CS9 */;
            break;
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
        uint32_t baud;
        uint32_t ie;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;

        if ((!termiosp)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#endif
           )
          {
            ret = -EINVAL;
            break;
          }

        /* Decode baud. */

        ret = OK;
        baud = cfgetispeed(termiosp);

        /* Decode number of bits */

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            nbits = 5;
            break;

          case CS6:
            nbits = 6;
            break;

          case CS7:
            nbits = 7;
            break;

          case CS8:
            nbits = 8;
            break;
#if 0
          case CS9:
            nbits = 9;
            break;
#endif
          default:
            ret = -EINVAL;
            break;
          }

        /* Decode parity */

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        /* Decode stop bits */

        stop2 = (termiosp->c_cflag & CSTOPB) != 0;

        /* Verify that all settings are valid before committing */

        if (ret == OK)
          {
            /* Commit */

            priv->baud      = baud;
            priv->parity    = parity;
            priv->bits      = nbits;
            priv->stopbits2 = stop2;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow     = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow     = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
            /* effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            s32k1xx_disableuartint(priv, &ie);
            ret = s32k1xx_setup(dev);

            /* Restore the interrupt state */

            s32k1xx_restoreuartint(priv, ie);
            priv->ie = ie;
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_S32K1XX_LPUART_INVERT
    case TIOCSINVERT:
      {
        uint32_t ctrl;
        uint32_t stat;
        uint32_t regval;
        irqstate_t flags;
        struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;

        flags  = spin_lock_irqsave();
        ctrl   = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);
        stat   = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
        regval = ctrl;

        /* {R|T}XINV bit field can only be written when the receiver is
        * disabled (RE=0).
        */

        regval &= ~LPUART_CTRL_RE;

        s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);

        /* Enable/disable signal inversion. */

        if (arg & SER_INVERT_ENABLED_RX)
          {
            stat |= LPUART_STAT_RXINV;
          }
        else
          {
            stat &= ~LPUART_STAT_RXINV;
          }

        if (arg & SER_INVERT_ENABLED_TX)
          {
            ctrl |= LPUART_CTRL_TXINV;
          }
        else
          {
            ctrl &= ~LPUART_CTRL_TXINV;
          }

        s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET, stat);
        s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, ctrl);

        spin_unlock_irqrestore(flags);
      }
      break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int s32k1xx_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  uint32_t rxd;

  rxd     = s32k1xx_serialin(priv, S32K1XX_LPUART_DATA_OFFSET);
  *status = rxd >> LPUART_DATA_STATUS_SHIFT;
  return (rxd & LPUART_DATA_MASK) >> LPUART_DATA_SHIFT;
}

/****************************************************************************
 * Name: s32k1xx_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void s32k1xx_rxint(struct uart_dev_s *dev, bool enable)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  irqstate_t flags;
  uint32_t regval;

  /* Enable interrupts for data available at Rx */

  flags = spin_lock_irqsave();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_RIE | LPUART_CTRL_FEIE | LPUART_CTRL_ORIE;
#endif
    }
  else
    {
      priv->ie &= ~(LPUART_CTRL_RIE | LPUART_CTRL_FEIE | LPUART_CTRL_ORIE);
    }

  regval  = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= priv->ie;
  s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: s32k1xx_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool s32k1xx_rxavailable(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  uint32_t regval;

  /* Return true is data is ready in the Rx FIFO */

  regval = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_RDRF) != 0);
}

/****************************************************************************
 * Name: s32k1xx_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void s32k1xx_send(struct uart_dev_s *dev, int ch)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  s32k1xx_serialout(priv, S32K1XX_LPUART_DATA_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: s32k1xx_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void s32k1xx_txint(struct uart_dev_s *dev, bool enable)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  irqstate_t flags;
  uint32_t regval;

  /* Enable interrupt for TX complete */

  flags = spin_lock_irqsave();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_TCIE;
#endif
    }
  else
    {
      priv->ie &= ~LPUART_CTRL_TCIE;
    }

  regval  = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= priv->ie;
  s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: s32k1xx_txready
 *
 * Description:
 *   Return true if the transmit is completed
 *
 ****************************************************************************/

static bool s32k1xx_txready(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  uint32_t regval;

  regval = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_TC) != 0);
}

/****************************************************************************
 * Name: s32k1xx_txempty
 *
 * Description:
 *   Return true if the transmit reg is empty
 *
 ****************************************************************************/

static bool s32k1xx_txempty(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  uint32_t regval;

  regval = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_TDRE) != 0);
}

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */
        }
        break;

      case(PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */
        }
        break;

      case(PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */
        }
        break;

      case(PM_SLEEP):
        {
          /* Logic for PM_SLEEP goes here */
        }
        break;

      default:

        /* Should not get here */

        break;
    }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void s32k1xx_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function s32k1xx_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  s32k1xx_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that s32k1xx_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef CONFIG_PM
  int ret;

  /* Register to receive power management callbacks */

  ret = pm_register(&g_serial_pmcb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
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
#ifdef CONSOLE_DEV
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)CONSOLE_DEV.priv;
  uint32_t ie;

  s32k1xx_disableuartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      s32k1xx_lowputc('\r');
    }

  s32k1xx_lowputc(ch);
  s32k1xx_restoreuartint(priv, ie);
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
#if CONSOLE_LPUART > 0
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#endif

  return ch;
}

#endif /* USE_SERIALDRIVER */
