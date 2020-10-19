/****************************************************************************
 * arch/arm/src/a1x/a1x_serial.c
 *
 *   Copyright (C) 2013-2014, 2017 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>
#include <debug.h>
#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "chip.h"
#include "hardware/a1x_uart.h"
#include "a1x_pio.h"
#include "a1x_serial.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART_DEVICE)

/* SCLK is the UART input clock.
 *
 * Through experimentation, it has been found that the serial clock is
 * OSC24M
 */

#define A1X_SCLK 24000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t ier;       /* Saved IER value */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  uart_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_A1X_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_A1X_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_A1X_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_A1X_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

#ifdef CONFIG_A1X_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

#ifdef CONFIG_A1X_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
#endif

#ifdef CONFIG_A1X_UART6
static char g_uart6rxbuffer[CONFIG_UART6_RXBUFSIZE];
static char g_uart6txbuffer[CONFIG_UART6_TXBUFSIZE];
#endif

#ifdef CONFIG_A1X_UART7
static char g_uart7rxbuffer[CONFIG_UART7_RXBUFSIZE];
static char g_uart7txbuffer[CONFIG_UART7_TXBUFSIZE];
#endif

/* This describes the state of the A1X UART0 port. */

#ifdef CONFIG_A1X_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = A1X_UART0_VADDR,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = A1X_IRQ_UART0,
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

/* This describes the state of the A1X UART1 port. */

#ifdef CONFIG_A1X_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = A1X_UART1_VADDR,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = A1X_IRQ_UART1,
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

/* This describes the state of the A1X UART2 port. */

#ifdef CONFIG_A1X_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = A1X_UART2_VADDR,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = A1X_IRQ_UART2,
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

/* This describes the state of the A1X UART3 port. */

#ifdef CONFIG_A1X_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = A1X_UART3_VADDR,
  .baud           = CONFIG_UART3_BAUD,
  .irq            = A1X_IRQ_UART3,
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

/* This describes the state of the A1X UART4 port. */

#ifdef CONFIG_A1X_UART4
static struct up_dev_s g_uart4priv =
{
  .uartbase       = A1X_UART4_VADDR,
  .baud           = CONFIG_UART4_BAUD,
  .irq            = A1X_IRQ_UART4,
  .parity         = CONFIG_UART4_PARITY,
  .bits           = CONFIG_UART4_BITS,
  .stopbits2      = CONFIG_UART4_2STOP,
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

/* This describes the state of the A1X UART5 port. */

#ifdef CONFIG_A1X_UART5
static struct up_dev_s g_uart5priv =
{
  .uartbase       = A1X_UART5_VADDR,
  .baud           = CONFIG_UART5_BAUD,
  .irq            = A1X_IRQ_UART5,
  .parity         = CONFIG_UART5_PARITY,
  .bits           = CONFIG_UART5_BITS,
  .stopbits2      = CONFIG_UART5_2STOP,
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

/* This describes the state of the A1X UART6 port. */

#ifdef CONFIG_A1X_UART6
static struct up_dev_s g_uart6priv =
{
  .uartbase       = A1X_UART6_VADDR,
  .baud           = CONFIG_UART6_BAUD,
  .irq            = A1X_IRQ_UART6,
  .parity         = CONFIG_UART6_PARITY,
  .bits           = CONFIG_UART6_BITS,
  .stopbits2      = CONFIG_UART6_2STOP,
};

static uart_dev_t g_uart6port =
{
  .recv     =
  {
    .size   = CONFIG_UART6_RXBUFSIZE,
    .buffer = g_uart6rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART6_TXBUFSIZE,
    .buffer = g_uart6txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart6priv,
};
#endif

/* This describes the state of the A1X UART7 port. */

#ifdef CONFIG_A1X_UART7
static struct up_dev_s g_uart7priv =
{
  .uartbase       = A1X_UART7_VADDR,
  .baud           = CONFIG_UART7_BAUD,
  .irq            = A1X_IRQ_UART7,
  .parity         = CONFIG_UART7_PARITY,
  .bits           = CONFIG_UART7_BITS,
  .stopbits2      = CONFIG_UART7_2STOP,
};

static uart_dev_t g_uart7port =
{
  .recv     =
  {
    .size   = CONFIG_UART7_RXBUFSIZE,
    .buffer = g_uart7rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART7_TXBUFSIZE,
    .buffer = g_uart7txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart7priv,
};
#endif

/* Which UART with be tty0/console and which tty1-7?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART0-7 */

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
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart6port /* UART6 is console */
#    define TTYS5_DEV           g_uart6port /* UART6 is ttyS0 */
#    define UART6_ASSIGNED      1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart7port /* UART7 is console */
#    define TTYS5_DEV           g_uart7port /* UART7 is ttyS0 */
#    define UART7_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_A1X_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_A1X_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_A1X_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(CONFIG_A1X_UART3)
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  elif defined(CONFIG_A1X_UART4)
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  elif defined(CONFIG_A1X_UART5)
#    define TTYS0_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#  elif defined(CONFIG_A1X_UART6)
#    define TTYS0_DEV           g_uart6port /* UART6 is ttyS0 */
#    define UART6_ASSIGNED      1
#  elif defined(CONFIG_A1X_UART7)
#    define TTYS0_DEV           g_uart7port /* UART7 is ttyS0 */
#    define UART7_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-7 excluding the console UART. */

#if defined(CONFIG_A1X_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_A1X_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_A1X_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_A1X_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A1X_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_A1X_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS1_DEV           g_uart5port /* UART5 is ttyS1 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_A1X_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS1_DEV           g_uart6port /* UART6 is ttyS1 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_A1X_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS1_DEV           g_uart7port /* UART7 is ttyS1 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART1-7. It can't be UART0 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-7 could also be the
 * console.
 */

#if defined(CONFIG_A1X_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_A1X_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_A1X_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A1X_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_A1X_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS2_DEV           g_uart5port /* UART5 is ttyS2 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_A1X_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS2_DEV           g_uart6port /* UART6 is ttyS2 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_A1X_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS2_DEV           g_uart7port /* UART7 is ttyS2 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART2-7. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART 2-7 could also be the console.
 */

#if defined(CONFIG_A1X_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS3_DEV           g_uart2port /* UART2 is ttyS3 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_A1X_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A1X_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_A1X_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS3_DEV           g_uart5port /* UART5 is ttyS3 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_A1X_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS3_DEV           g_uart6port /* UART6 is ttyS3 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_A1X_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS3_DEV           g_uart7port /* UART7 is ttyS3 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys4. This could be one of UART3-7. It can't be UART0-2 because
 * those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * UART 3-7 could also be the console.
 */

#if defined(CONFIG_A1X_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS4_DEV           g_uart3port /* UART3 is ttyS4 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A1X_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_A1X_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS4_DEV           g_uart5port /* UART5 is ttyS4 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_A1X_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS4_DEV           g_uart6port /* UART6 is ttyS4 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_A1X_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS4_DEV           g_uart7port /* UART7 is ttyS4 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys5. This could be one of UART4-7. It can't be UART0-3 because
 * those have already been assigned to ttsyS0, 1, 2, 3 or 4.  One of
 * UART 4-7 could also be the console.
 */

#if defined(CONFIG_A1X_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS5_DEV           g_uart4port /* UART4 is ttyS5 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_A1X_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS5_DEV           g_uart5port /* UART5 is ttyS5 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_A1X_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS5_DEV           g_uart6port /* UART6 is ttyS5 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_A1X_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS5_DEV           g_uart7port /* UART7 is ttyS5 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys6. This could be one of UART5-7. It can't be UART0-4 because
 * those have already been assigned to ttsyS0, 1, 2, 3, 4, or 5.  One of
 * UART 5-7 could also be the console.
 */

#if defined(CONFIG_A1X_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS6_DEV           g_uart5port /* UART5 is ttyS6 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_A1X_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS6_DEV           g_uart6port /* UART6 is ttyS6 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_A1X_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS6_DEV           g_uart7port /* UART7 is ttyS6 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys7. This could be one of UART6-7. It can't be UART0-5 because
 * those have already been assigned to ttsyS0, 1, 2, 3, 4, 5, or 6.  One of
 * UART 6-7 could also be the console.
 */

#if defined(CONFIG_A1X_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS7_DEV           g_uart6port /* UART6 is ttyS7 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_A1X_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS7_DEV           g_uart7port /* UART7 is ttyS7 */
#  define UART7_ASSIGNED      1
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLIE;
    }

  priv->ier &= ~UART_IER_ALLIE;
  up_serialout(priv, A1X_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  up_serialout(priv, A1X_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint32_t lcr = up_serialin(priv, A1X_UART_LCR_OFFSET);

  if (enable)
    {
      lcr |= UART_LCR_BC;
    }
  else
    {
      lcr &= ~UART_LCR_BC;
    }

  up_serialout(priv, A1X_UART_LCR_OFFSET, lcr);
}

/****************************************************************************
 * Name: a1x_uart0config, uart1config, uart2config, ..., uart7config
 *
 * Description:
 *   Configure the UART
 *
 ****************************************************************************/

#ifdef CONFIG_A1X_UART0
static inline void a1x_uart0config(void)
{
  irqstate_t flags;

  /* Step 1: Enable power to UART0 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking to UART0 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a1x_pio_config(PIO_UART0_TX);
  a1x_pio_config(PIO_UART0_RX);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_A1X_UART1
static inline void a1x_uart1config(void)
{
  irqstate_t flags;

  /* Step 1: Enable power to UART1 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking to UART1 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a1x_pio_config(PIO_UART1_TX);
  a1x_pio_config(PIO_UART1_RX);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_A1X_UART2
static inline void a1x_uart2config(void)
{
  irqstate_t flags;

  /* Step 1: Enable power to UART2 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking on UART2 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a1x_pio_config(PIO_UART2_TX);
  a1x_pio_config(PIO_UART2_RX);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_A1X_UART3
static inline void a1x_uart3config(void)
{
  irqstate_t flags;

  /* Step 1: Enable power to UART3 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking to UART3 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a1x_pio_config(PIO_UART3_TX);
  a1x_pio_config(PIO_UART3_RX);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_A1X_UART4
static inline void a1x_uart4config(void)
{
  irqstate_t flags;

  /* Step 1: Enable power to UART4 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking to UART4 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a1x_pio_config(PIO_UART4_TX);
  a1x_pio_config(PIO_UART4_RX);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_A1X_UART5
static inline void a1x_uart5config(void)
{
  irqstate_t flags;

  /* Step 1: Enable power to UART5 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking to UART5 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a1x_pio_config(PIO_UART5_TX);
  a1x_pio_config(PIO_UART5_RX);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_A1X_UART6
static inline void a1x_uart6config(void)
{
  irqstate_t flags;

  /* Step 1: Enable power to UART6 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking to UART6 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a1x_pio_config(PIO_UART6_TX);
  a1x_pio_config(PIO_UART6_RX);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_A1X_UART7
static inline void a1x_uart7config(void)
{
  irqstate_t flags;

  /* Step 1: Enable power to UART7 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking to UART7 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a1x_pio_config(PIO_UART7_TX);
  a1x_pio_config(PIO_UART7_RX);
  leave_critical_section(flags);
};
#endif

/****************************************************************************
 * Name: a1x_uartdl
 *
 * Description:
 *   Select a divider to produce the BAUD from the UART PCLK.
 *
 *     BAUD = PCLK / (16 * DL), or
 *     DL   = PCLK / BAUD / 16
 *
 ****************************************************************************/

static inline uint32_t a1x_uartdl(uint32_t baud)
{
  return A1X_SCLK / (baud << 4);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint16_t dl;
  uint32_t lcr;

  /* Clear fifos */

  up_serialout(priv, A1X_UART_FCR_OFFSET,
              (UART_FCR_RFIFOR | UART_FCR_XFIFOR));

  /* Set trigger */

  up_serialout(priv, A1X_UART_FCR_OFFSET,
              (UART_FCR_FIFOE | UART_FCR_RT_HALF));

  /* Set up the IER */

  priv->ier = up_serialin(priv, A1X_UART_IER_OFFSET);

  /* Set up the LCR */

  lcr = 0;

  switch (priv->bits)
    {
    case 5:
      lcr |= UART_LCR_DLS_5BITS;
      break;

    case 6:
      lcr |= UART_LCR_DLS_6BITS;
      break;

    case 7:
      lcr |= UART_LCR_DLS_7BITS;
      break;

    case 8:
    default:
      lcr |= UART_LCR_DLS_8BITS;
      break;
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STOP;
    }

  if (priv->parity == 1)
    {
      lcr |= UART_LCR_PEN;
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

  /* Enter DLAB=1 */

  up_serialout(priv, A1X_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  /* Set the BAUD divisor */

  dl = a1x_uartdl(priv->baud);
  up_serialout(priv, A1X_UART_DLH_OFFSET, dl >> 8);
  up_serialout(priv, A1X_UART_DLL_OFFSET, dl & 0xff);

  /* Clear DLAB */

  up_serialout(priv, A1X_UART_LCR_OFFSET, lcr);

  /* Configure the FIFOs */

  up_serialout(priv, A1X_UART_FCR_OFFSET,
               (UART_FCR_RT_HALF | UART_FCR_XFIFOR | UART_FCR_RFIFOR |
                UART_FCR_FIFOE));

  /* Enable Auto-Flow Control in the Modem Control Register */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
#  warning Missing logic
#endif

#endif
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the  the setup() method is called,
 *   however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the hardware supports multiple levels of interrupt enabling).
 *   The RX and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, uart_interrupt, dev);
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
 *   Detach UART interrupts.
 *   This method is called when the serial port is closed normally
 *   just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: uartN_interrupt and uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int uart_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv;
  uint32_t status;
  int passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status */

      status = up_serialin(priv, A1X_UART_IIR_OFFSET);

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_IID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_IID_RECV:
          case UART_IIR_IID_TIMEOUT:
            {
              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_IID_TXEMPTY:
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts (UART1 only) */

          case UART_IIR_IID_MODEM:
            {
              /* Read the modem status register (MSR) to clear */

              status = up_serialin(priv, A1X_UART_MSR_OFFSET);
              _info("MSR: %02x\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_IID_LINESTATUS:
            {
              /* Read the line status register (LSR) to clear */

              status = up_serialin(priv, A1X_UART_LSR_OFFSET);
              _info("LSR: %02x\n", status);
              break;
            }

          /* Busy detect.
           * Just ignore.
           * Cleared by reading the status register
           */

          case UART_IIR_IID_BUSY:
            {
              /* Read from the UART status register
               * to clear the BUSY condition
               */

              status = up_serialin(priv, A1X_UART_USR_OFFSET);
              break;
            }

          /* No further interrupts pending... return now */

          case UART_IIR_IID_NONE:
            {
              return OK;
            }

            /* Otherwise we have received an interrupt
             * that we cannot handle
             */

          default:
            {
              _err("ERROR: Unexpected IIR: %02x\n", status);
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
  struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct up_dev_s *user = (struct up_dev_s *)arg;
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
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = enter_critical_section();
        up_enablebreaks(priv, true);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = enter_critical_section();
        up_enablebreaks(priv, false);
        leave_critical_section(flags);
      }
      break;

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Other termios fields are not yet returned.
         * Note that cfsetospeed is not necessary because we have
         * knowledge that only one speed is supported.
         * Both cfset(i|o)speed() translate to cfsetspeed.
         */

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        uint32_t           lcr;  /* Holds current values of line control register */
        uint16_t           dl;   /* Divisor latch */

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Handle other termios settings.
         * Note that only cfgetispeed is used because we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);

        /* TODO: Re-calculate the optimal CCLK divisor for the new baud and
         * and reset the divider in the CLKSEL0/1 register.
         */

        /* DLAB open latch */

        /* REVISIT:
         * Shouldn't we just call up_setup() to do all of the following?
         */

        lcr = getreg32(priv->uartbase + A1X_UART_LCR_OFFSET);
        up_serialout(priv, A1X_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

        /* Set the BAUD divisor */

        dl = a1x_uartdl(priv->baud);
        up_serialout(priv, A1X_UART_DLH_OFFSET, dl >> 8);
        up_serialout(priv, A1X_UART_DLL_OFFSET, dl & 0xff);

        /* Clear DLAB */

        up_serialout(priv, A1X_UART_LCR_OFFSET, lcr);
      }
      break;
#endif

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
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rbr;

  *status = up_serialin(priv, A1X_UART_LSR_OFFSET);
  rbr     = up_serialin(priv, A1X_UART_RBR_OFFSET);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
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

  up_serialout(priv, A1X_UART_IER_OFFSET, priv->ier);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, A1X_UART_LSR_OFFSET) & UART_LSR_DR) != 0);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, A1X_UART_THR_OFFSET, (uint32_t)ch);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_ETBEI;
      up_serialout(priv, A1X_UART_IER_OFFSET, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_ETBEI;
      up_serialout(priv, A1X_UART_IER_OFFSET, priv->ier);
    }

  leave_critical_section(flags);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, A1X_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, A1X_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
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
 *   before arm_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by up_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* Configure all UARTs (except the CONSOLE UART) and disable interrupts */

#ifdef CONFIG_A1X_UART0
#ifndef CONFIG_UART0_SERIAL_CONSOLE
  a1x_uart0config();
#endif
  up_disableuartint(&g_uart0priv, NULL);
#endif

#ifdef CONFIG_A1X_UART1
#ifndef CONFIG_UART1_SERIAL_CONSOLE
  a1x_uart1config();
#else
#endif
  up_disableuartint(&g_uart1priv, NULL);
#endif

#ifdef CONFIG_A1X_UART2
#ifndef CONFIG_UART2_SERIAL_CONSOLE
  a1x_uart2config();
#endif
  up_disableuartint(&g_uart2priv, NULL);
#endif

#ifdef CONFIG_A1X_UART3
#ifndef CONFIG_UART3_SERIAL_CONSOLE
  a1x_uart3config();
#endif
  up_disableuartint(&g_uart3priv, NULL);
#endif

#ifdef CONFIG_A1X_UART4
#ifndef CONFIG_UART4_SERIAL_CONSOLE
  a1x_uart4config();
#endif
  up_disableuartint(&g_uart4priv, NULL);
#endif

#ifdef CONFIG_A1X_UART5
#ifndef CONFIG_UART5_SERIAL_CONSOLE
  a1x_uart5config();
#endif
  up_disableuartint(&g_uart5priv, NULL);
#endif

#ifdef CONFIG_A1X_UART6
#ifndef CONFIG_UART6_SERIAL_CONSOLE
  a1x_uart6config();
#endif
  up_disableuartint(&g_uart6priv, NULL);
#endif

#ifdef CONFIG_A1X_UART7
#ifndef CONFIG_UART7_SERIAL_CONSOLE
  a1x_uart7config();
#endif
  up_disableuartint(&g_uart7priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
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
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif
#ifdef TTYS5_DEV
  uart_register("/dev/ttyS5", &TTYS5_DEV);
#endif
#ifdef TTYS6_DEV
  uart_register("/dev/ttyS6", &TTYS6_DEV);
#endif
#ifdef TTYS7_DEV
  uart_register("/dev/ttyS7", &TTYS7_DEV);
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
#ifdef HAVE_SERIAL_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t ier;
  up_disableuartint(priv, &ier);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#ifdef HAVE_SERIAL_CONSOLE
  up_restoreuartint(priv, ier);
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
#ifdef HAVE_UART_DEVICE
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
