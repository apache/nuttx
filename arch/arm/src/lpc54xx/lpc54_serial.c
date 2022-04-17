/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_serial.c
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
#include "chip.h"
#include "lpc54_config.h"
#include "hardware/lpc54_usart.h"
#include "lpc54_clockconfig.h"
#include "lpc54_lowputc.h"
#include "lpc54_serial.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there at least one USART enabled and configured as a RS-232 device? */

#ifndef HAVE_USART_DEVICE
#  warning "No USARTs enabled"
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(HAVE_USART_DEVICE) && defined(USE_SERIALDRIVER)

/* Which USART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered USART.
 */

/* First pick the console and ttys0.  This could be any of USART0-5 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port /* USART0 is console */
#    define TTYS0_DEV           g_uart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port /* USART1 is console */
#    define TTYS0_DEV           g_uart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED     1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart2port /* USART2 is console */
#    define TTYS0_DEV           g_uart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart3port /* USART3 is console */
#    define TTYS0_DEV           g_uart3port /* USART3 is ttyS0 */
#    define USART3_ASSIGNED     1
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart4port /* USART4 is console */
#    define TTYS0_DEV           g_uart4port /* USART4 is ttyS0 */
#    define USART4_ASSIGNED     1
#elif defined(CONFIG_USART5_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart5port /* USART5 is console */
#    define TTYS5_DEV           g_uart5port /* USART5 is ttyS0 */
#    define USART5_ASSIGNED     1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart6port /* USART6 is console */
#    define TTYS6_DEV           g_uart6port /* USART6 is ttyS0 */
#    define USART6_ASSIGNED     1
#elif defined(CONFIG_USART7_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart7port /* USART7 is console */
#    define TTYS7_DEV           g_uart7port /* USART7 is ttyS0 */
#    define USART7_ASSIGNED     1
#elif defined(CONFIG_USART8_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart8port /* USART8 is console */
#    define TTYS8_DEV           g_uart8port /* USART8 is ttyS0 */
#    define USART8_ASSIGNED     1
#elif defined(CONFIG_USART9_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart9port /* USART9 is console */
#    define TTYS9_DEV           g_uart9port /* USART9 is ttyS0 */
#    define USART9_ASSIGNED     1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(HAVE_USART0)
#    define TTYS0_DEV           g_uart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#  elif defined(HAVE_USART1)
#    define TTYS0_DEV           g_uart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED     1
#  elif defined(HAVE_USART2)
#    define TTYS0_DEV           g_uart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#  elif defined(HAVE_USART3)
#    define TTYS0_DEV           g_uart3port /* USART3 is ttyS0 */
#    define USART3_ASSIGNED     1
#  elif defined(HAVE_USART4)
#    define TTYS0_DEV           g_uart4port /* USART4 is ttyS0 */
#    define USART4_ASSIGNED     1
#  elif defined(HAVE_USART5)
#    define TTYS0_DEV           g_uart5port /* USART5 is ttyS0 */
#    define USART5_ASSIGNED     1
#  elif defined(HAVE_USART6)
#    define TTYS0_DEV           g_uart6port /* USART6 is ttyS0 */
#    define USART7_ASSIGNED     1
#  elif defined(HAVE_USART7)
#    define TTYS0_DEV           g_uart7port /* USART7 is ttyS0 */
#    define USART7_ASSIGNED     1
#  elif defined(HAVE_USART8)
#    define TTYS0_DEV           g_uart8port /* USART8 is ttyS0 */
#    define USART8_ASSIGNED     1
#  elif defined(HAVE_USART9)
#    define TTYS0_DEV           g_uart9port /* USART9 is ttyS0 */
#    define USART9_ASSIGNED     1
#  endif
#endif

/* Pick ttys1.  This could be any of USART0-9 excluding the console USART. */

#if defined(HAVE_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV             g_uart0port /* USART0 is ttyS1 */
#  define USART0_ASSIGNED       1
#elif defined(HAVE_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS1_DEV             g_uart1port /* USART1 is ttyS1 */
#  define USART1_ASSIGNED       1
#elif defined(HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS1_DEV             g_uart2port /* USART2 is ttyS1 */
#  define USART2_ASSIGNED       1
#elif defined(HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS1_DEV             g_uart3port /* USART3 is ttyS1 */
#  define USART3_ASSIGNED       1
#elif defined(HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS1_DEV             g_uart4port /* USART4 is ttyS1 */
#  define USART4_ASSIGNED       1
#elif defined(HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS1_DEV             g_uart5port /* USART5 is ttyS1 */
#  define USART5_ASSIGNED       1
#elif defined(HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS1_DEV             g_uart6port /* USART6 is ttyS1 */
#  define USART6_ASSIGNED       1
#elif defined(HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS1_DEV             g_uart7port /* USART7 is ttyS1 */
#  define USART7_ASSIGNED       1
#elif defined(HAVE_USART8) && !defined(USART8_ASSIGNED)
#  define TTYS1_DEV             g_uart8port /* USART8 is ttyS1 */
#  define USART8_ASSIGNED       1
#elif defined(HAVE_USART9) && !defined(USART9_ASSIGNED)
#  define TTYS1_DEV             g_uart9port /* USART9 is ttyS1 */
#  define USART9_ASSIGNED       1
#endif

/* Pick ttys2.  This could be one of USART1-9. It can't be USART0 because
 * that is either not enabled OR was already assigned as ttyS0 or ttys1.
 * One of USART 1-9 could also be the console.
 */

#if defined(HAVE_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS2_DEV             g_uart1port /* USART1 is ttyS2 */
#  define USART1_ASSIGNED       1
#elif defined(HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS2_DEV             g_uart2port /* USART2 is ttyS2 */
#  define USART2_ASSIGNED       1
#elif defined(HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS2_DEV             g_uart3port /* USART3 is ttyS2 */
#  define USART3_ASSIGNED       1
#elif defined(HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS2_DEV             g_uart4port /* USART4 is ttyS2 */
#  define USART4_ASSIGNED       1
#elif defined(HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS2_DEV             g_uart5port /* USART5 is ttyS2 */
#  define USART5_ASSIGNED       1
#elif defined(HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS2_DEV             g_uart6port /* USART6 is ttyS2 */
#  define USART6_ASSIGNED       1
#elif defined(HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS2_DEV             g_uart7port /* USART7 is ttyS2 */
#  define USART7_ASSIGNED       1
#elif defined(HAVE_USART8) && !defined(USART8_ASSIGNED)
#  define TTYS2_DEV             g_uart8port /* USART8 is ttyS2 */
#  define USART8_ASSIGNED       1
#elif defined(HAVE_USART9) && !defined(USART9_ASSIGNED)
#  define TTYS2_DEV             g_uart9port /* USART9 is ttyS2 */
#  define USART9_ASSIGNED       1
#endif

/* Pick ttys3.  This could be one of USART2-9. It can't be USART0-1 because
 * those are either not enabled OR were already assigned as ttyS0, 1, or 2.
 * One of USART 2-9 could also be the console.
 */

#if defined(HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS3_DEV             g_uart2port /* USART2 is ttyS3 */
#  define USART2_ASSIGNED       1
#elif defined(HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS3_DEV             g_uart3port /* USART3 is ttyS3 */
#  define USART3_ASSIGNED       1
#elif defined(HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS3_DEV             g_uart4port /* USART4 is ttyS3 */
#  define USART4_ASSIGNED       1
#elif defined(HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS3_DEV             g_uart5port /* USART5 is ttyS3 */
#  define USART5_ASSIGNED       1
#elif defined(HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS3_DEV             g_uart6port /* USART6 is ttyS3 */
#  define USART6_ASSIGNED       1
#elif defined(HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS3_DEV             g_uart7port /* USART7 is ttyS3 */
#  define USART7_ASSIGNED       1
#elif defined(HAVE_USART8) && !defined(USART8_ASSIGNED)
#  define TTYS3_DEV             g_uart8port /* USART8 is ttyS3 */
#  define USART8_ASSIGNED       1
#elif defined(HAVE_USART9) && !defined(USART9_ASSIGNED)
#  define TTYS3_DEV             g_uart9port /* USART9 is ttyS3 */
#  define USART9_ASSIGNED       1
#endif

/* Pick ttys4.  This could be one of USART3-9. It can't be USART0-2 because
 * those are either not enabled OR were already assigned as ttyS0-3.  One
 * of USART 3-9 could also be the console.
 */

#if defined(HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS4_DEV             g_uart3port /* USART3 is ttyS4 */
#  define USART3_ASSIGNED       1
#elif defined(HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS4_DEV             g_uart4port /* USART4 is ttyS4 */
#  define USART4_ASSIGNED       1
#elif defined(HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS4_DEV             g_uart5port /* USART5 is ttyS4 */
#  define USART5_ASSIGNED       1
#elif defined(HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS4_DEV             g_uart6port /* USART6 is ttyS4 */
#  define USART6_ASSIGNED       1
#elif defined(HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS4_DEV             g_uart7port /* USART7 is ttyS4 */
#  define USART7_ASSIGNED       1
#elif defined(HAVE_USART8) && !defined(USART8_ASSIGNED)
#  define TTYS4_DEV             g_uart8port /* USART8 is ttyS4 */
#  define USART8_ASSIGNED       1
#elif defined(HAVE_USART9) && !defined(USART9_ASSIGNED)
#  define TTYS4_DEV             g_uart9port /* USART9 is ttyS4 */
#  define USART9_ASSIGNED       1
#endif

/* Pick ttys5.  This could be one of USART4-9. It can't be USART0-3 because
 * those are either not enabled OR were already assigned as ttyS0-4.  One
 * of USART4-9 could also be the console.
 */

#if defined(HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS5_DEV             g_uart4port /* USART4 is ttyS5 */
#  define USART4_ASSIGNED       1
#elif defined(HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS5_DEV             g_uart5port /* USART5 is ttyS5 */
#  define USART5_ASSIGNED       1
#elif defined(HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS5_DEV             g_uart6port /* USART6 is ttyS5 */
#  define USART6_ASSIGNED       1
#elif defined(HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS5_DEV             g_uart7port /* USART7 is ttyS5 */
#  define USART7_ASSIGNED       1
#elif defined(HAVE_USART8) && !defined(USART8_ASSIGNED)
#  define TTYS5_DEV             g_uart8port /* USART8 is ttyS5 */
#  define USART8_ASSIGNED       1
#elif defined(HAVE_USART9) && !defined(USART9_ASSIGNED)
#  define TTYS5_DEV             g_uart9port /* USART9 is ttyS5 */
#  define USART9_ASSIGNED       1
#endif

/* Pick ttys6.  This could be one of USART5-9. It can't be USART0-4 because
 * those are either not enabled OR were already assigned as ttyS0-5.  One
 * of USART5-9 could also be the console.
 */

#if defined(HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS6_DEV             g_uart5port /* USART5 is ttyS6 */
#  define USART5_ASSIGNED       1
#elif defined(HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS6_DEV             g_uart6port /* USART6 is ttyS6 */
#  define USART6_ASSIGNED       1
#elif defined(HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS6_DEV             g_uart7port /* USART7 is ttyS6 */
#  define USART7_ASSIGNED       1
#elif defined(HAVE_USART8) && !defined(USART8_ASSIGNED)
#  define TTYS6_DEV             g_uart8port /* USART8 is ttyS6 */
#  define USART8_ASSIGNED       1
#elif defined(HAVE_USART9) && !defined(USART9_ASSIGNED)
#  define TTYS6_DEV             g_uart9port /* USART9 is ttyS6 */
#  define USART9_ASSIGNED       1
#endif

/* Pick ttys7.  This could be one of USART6-9. It can't be USART0-5 because
 * those are either not enabled OR were already assigned as ttyS0-6.  One
 * of USART6-9 could also be the console.
 */

#if defined(HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS7_DEV             g_uart6port /* USART6 is ttyS7 */
#  define USART6_ASSIGNED       1
#elif defined(HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS7_DEV             g_uart7port /* USART7 is ttyS7 */
#  define USART7_ASSIGNED       1
#elif defined(HAVE_USART8) && !defined(USART8_ASSIGNED)
#  define TTYS7_DEV             g_uart8port /* USART8 is ttyS7 */
#  define USART8_ASSIGNED       1
#elif defined(HAVE_USART9) && !defined(USART9_ASSIGNED)
#  define TTYS7_DEV             g_uart9port /* USART9 is ttyS7 */
#  define USART9_ASSIGNED       1
#endif

/* Pick ttys8.  This could be one of USART7-9. It can't be USART0-6 because
 * those are either not enabled OR were already assigned as ttyS0-7.  One
 * of USART7-9 could also be the console.
 */

#if defined(HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS8_DEV             g_uart7port /* USART7 is ttyS8 */
#  define USART7_ASSIGNED       1
#elif defined(HAVE_USART8) && !defined(USART8_ASSIGNED)
#  define TTYS8_DEV             g_uart8port /* USART8 is ttyS8 */
#  define USART8_ASSIGNED       1
#elif defined(HAVE_USART9) && !defined(USART9_ASSIGNED)
#  define TTYS8_DEV             g_uart9port /* USART9 is ttyS8 */
#  define USART9_ASSIGNED       1
#endif

/* Event sets */

#define CCR_RX_EVENTS       (USART_FIFOINT_RXLVL | USART_FIFOINT_RXERR)
#define CCR_TX_EVENTS       (USART_FIFOINT_TXLVL | USART_FIFOINT_TXERR)
#define CCR_ERROR_EVENTS    (USART_FIFOINT_RXERR | USART_FIFOINT_TXERR | \
                             USART_FIFOINTSTAT_PERINT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one USART device */

struct lpc54_dev_s
{
  uintptr_t uartbase;  /* Base address of USART registers */
  uint8_t   irq;       /* IRQ associated with this USART */

  /* USART configuration */

  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  lpc54_setup(struct uart_dev_s *dev);
static void lpc54_shutdown(struct uart_dev_s *dev);
static int  lpc54_attach(struct uart_dev_s *dev);
static void lpc54_detach(struct uart_dev_s *dev);
static int  lpc54_interrupt(int irq, void *context, void *arg);
static int  lpc54_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  lpc54_receive(struct uart_dev_s *dev, unsigned int *status);
static void lpc54_rxint(struct uart_dev_s *dev, bool enable);
static bool lpc54_rxavailable(struct uart_dev_s *dev);
static void lpc54_send(struct uart_dev_s *dev, int ch);
static void lpc54_txint(struct uart_dev_s *dev, bool enable);
static bool lpc54_txready(struct uart_dev_s *dev);
static bool lpc54_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = lpc54_setup,
  .shutdown       = lpc54_shutdown,
  .attach         = lpc54_attach,
  .detach         = lpc54_detach,
  .ioctl          = lpc54_ioctl,
  .receive        = lpc54_receive,
  .rxint          = lpc54_rxint,
  .rxavailable    = lpc54_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = lpc54_send,
  .txint          = lpc54_txint,
  .txready        = lpc54_txready,
  .txempty        = lpc54_txempty,
};

/* I/O buffers */

#ifdef HAVE_USART0
static char g_uart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef HAVE_USART1
static char g_uart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef HAVE_USART2
static char g_uart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef HAVE_USART3
static char g_uart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif
#ifdef HAVE_USART4
static char g_uart4rxbuffer[CONFIG_USART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_USART4_TXBUFSIZE];
#endif
#ifdef HAVE_USART5
static char g_uart5rxbuffer[CONFIG_USART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_USART5_TXBUFSIZE];
#endif

/* This describes the state of the LPC54xx USART0 port. */

#ifdef HAVE_USART0
static struct lpc54_dev_s g_uart0priv =
{
  .uartbase       = LPC54_FLEXCOMM0_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM0,
  .config         =
  {
    .baud         = CONFIG_USART0_BAUD,
    .fclk         = BOARD_FLEXCOMM0_FCLK,
    .parity       = CONFIG_USART0_PARITY,
    .bits         = CONFIG_USART0_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART0_2STOP,
#ifdef CONFIG_USART0_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART0_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

/* This describes the state of the LPC54xx USART1 port. */

#ifdef HAVE_USART1
static struct lpc54_dev_s g_uart1priv =
{
  .uartbase       = LPC54_FLEXCOMM1_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM1,
  .config         =
  {
    .baud         = CONFIG_USART1_BAUD,
    .fclk         = BOARD_FLEXCOMM1_FCLK,
    .parity       = CONFIG_USART1_PARITY,
    .bits         = CONFIG_USART1_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART1_2STOP,
#ifdef CONFIG_USART1_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART1_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_USART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif

/* This describes the state of the LPC54xx USART2 port. */

#ifdef HAVE_USART2
static struct lpc54_dev_s g_uart2priv =
{
  .uartbase       = LPC54_FLEXCOMM2_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM2,
  .config         =
  {
    .baud         = CONFIG_USART2_BAUD,
    .fclk         = BOARD_FLEXCOMM2_FCLK,
    .parity       = CONFIG_USART2_PARITY,
    .bits         = CONFIG_USART2_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART2_2STOP,
#ifdef CONFIG_USART2_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART2_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart2port =
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

/* This describes the state of the LPC54xx USART3 port. */

#ifdef HAVE_USART3
static struct lpc54_dev_s g_uart3priv =
{
  .uartbase       = LPC54_FLEXCOMM3_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM3,
  .config         =
  {
    .baud         = CONFIG_USART3_BAUD,
    .fclk         = BOARD_FLEXCOMM3_FCLK,
    .parity       = CONFIG_USART3_PARITY,
    .bits         = CONFIG_USART3_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART3_2STOP,
#ifdef CONFIG_USART3_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART3_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};
#endif

/* This describes the state of the LPC54xx USART4 port. */

#ifdef HAVE_USART4
static struct lpc54_dev_s g_uart4priv =
{
  .uartbase       = LPC54_FLEXCOMM4_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM4,
  .config         =
  {
    .baud         = CONFIG_USART4_BAUD,
    .fclk         = BOARD_FLEXCOMM4_FCLK,
    .parity       = CONFIG_USART4_PARITY,
    .bits         = CONFIG_USART4_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART4_2STOP,
#ifdef CONFIG_USART4_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART4_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart4port =
{
  .recv     =
  {
    .size   = CONFIG_USART4_RXBUFSIZE,
    .buffer = g_uart4rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART4_TXBUFSIZE,
    .buffer = g_uart4txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart4priv,
};
#endif

/* This describes the state of the LPC54xx USART5 port. */

#ifdef HAVE_USART5
static struct lpc54_dev_s g_uart5priv =
{
  .uartbase       = LPC54_FLEXCOMM5_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM5,
  .config         =
  {
    .baud         = CONFIG_USART5_BAUD,
    .fclk         = BOARD_FLEXCOMM5_FCLK,
    .parity       = CONFIG_USART5_PARITY,
    .bits         = CONFIG_USART5_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART5_2STOP,
#ifdef CONFIG_USART5_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART5_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart5port =
{
  .recv     =
  {
    .size   = CONFIG_USART5_RXBUFSIZE,
    .buffer = g_uart5rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART5_TXBUFSIZE,
    .buffer = g_uart5txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart5priv,
};
#endif

/* This describes the state of the LPC54xx USART6 port. */

#ifdef HAVE_USART6
static struct lpc54_dev_s g_uart6priv =
{
  .uartbase       = LPC54_FLEXCOMM6_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM6,
  .config         =
  {
    .baud         = CONFIG_USART6_BAUD,
    .fclk         = BOARD_FLEXCOMM6_FCLK,
    .parity       = CONFIG_USART6_PARITY,
    .bits         = CONFIG_USART6_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART6_2STOP,
#ifdef CONFIG_USART6_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART6_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart6port =
{
  .recv     =
  {
    .size   = CONFIG_USART6_RXBUFSIZE,
    .buffer = g_uart6rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART6_TXBUFSIZE,
    .buffer = g_uart6txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart6priv,
};
#endif

/* This describes the state of the LPC54xx USART7 port. */

#ifdef HAVE_USART7
static struct lpc54_dev_s g_uart7priv =
{
  .uartbase       = LPC54_FLEXCOMM7_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM7,
  .config         =
  {
    .baud         = CONFIG_USART7_BAUD,
    .fclk         = BOARD_FLEXCOMM7_FCLK,
    .parity       = CONFIG_USART7_PARITY,
    .bits         = CONFIG_USART7_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART7_2STOP,
#ifdef CONFIG_USART7_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART7_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart7port =
{
  .recv     =
  {
    .size   = CONFIG_USART7_RXBUFSIZE,
    .buffer = g_uart7rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART7_TXBUFSIZE,
    .buffer = g_uart7txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart7priv,
};
#endif

/* This describes the state of the LPC54xx USART8 port. */

#ifdef HAVE_USART8
static struct lpc54_dev_s g_uart8priv =
{
  .uartbase       = LPC54_FLEXCOMM8_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM8,
  .config         =
  {
    .baud         = CONFIG_USART8_BAUD,
    .fclk         = BOARD_FLEXCOMM8_FCLK,
    .parity       = CONFIG_USART8_PARITY,
    .bits         = CONFIG_USART8_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART8_2STOP,
#ifdef CONFIG_USART8_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART8_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart8port =
{
  .recv     =
  {
    .size   = CONFIG_USART8_RXBUFSIZE,
    .buffer = g_uart8rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART8_TXBUFSIZE,
    .buffer = g_uart8txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart8priv,
};
#endif

/* This describes the state of the LPC54xx USART9 port. */

#ifdef HAVE_USART9
static struct lpc54_dev_s g_uart9priv =
{
  .uartbase       = LPC54_FLEXCOMM9_BASE,
  .irq            = LPC54_IRQ_FLEXCOMM9,
  .config         =
  {
    .baud         = CONFIG_USART9_BAUD,
    .fclk         = BOARD_FLEXCOMM9_FCLK,
    .parity       = CONFIG_USART9_PARITY,
    .bits         = CONFIG_USART9_BITS,
    .txlevel      = LPC54_USART_FIFO_DEPTH / 2,
    .rxlevel      = 0,
    .stopbits2    = CONFIG_USART9_2STOP,
#ifdef CONFIG_USART9_IFLOWCONTROL
    .iflow        = true,
#endif
#ifdef CONFIG_USART9_OFLOWCONTROL
    .oflow        = true,
#endif
  }
};

static uart_dev_t g_uart9port =
{
  .recv     =
  {
    .size   = CONFIG_USART9_RXBUFSIZE,
    .buffer = g_uart9rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART9_TXBUFSIZE,
    .buffer = g_uart9txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart9priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_serialin
 ****************************************************************************/

static inline uint32_t lpc54_serialin(struct lpc54_dev_s *priv,
                                      unsigned int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: lpc54_serialout
 ****************************************************************************/

static inline void lpc54_serialout(struct lpc54_dev_s *priv,
                                   unsigned int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: lpc54_modifyreg
 ****************************************************************************/

static inline void lpc54_modifyreg(struct lpc54_dev_s *priv,
                                   unsigned int offset, uint32_t setbits,
                                   uint32_t clrbits)
{
  irqstate_t flags;
  uintptr_t regaddr = priv->uartbase + offset;
  uint32_t regval;

  flags   = enter_critical_section();

  regval  = getreg32(regaddr);
  regval &= ~clrbits;
  regval |= setbits;
  putreg32(regval, regaddr);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc54_fifoint_enable
 ****************************************************************************/

static inline void lpc54_fifoint_enable(struct lpc54_dev_s *priv,
                                        uint32_t intset)
{
  lpc54_serialout(priv, LPC54_USART_FIFOINTENSET_OFFSET, intset);
}

/****************************************************************************
 * Name: lpc54_fifoint_disable
 ****************************************************************************/

static inline void lpc54_fifoint_disable(struct lpc54_dev_s *priv,
                                         uint32_t intset)
{
  lpc54_serialout(priv, LPC54_USART_FIFOINTENCLR_OFFSET, intset);
}

/****************************************************************************
 * Name: lpc54_fifoint_disableall
 ****************************************************************************/

static void lpc54_fifoint_disableall(struct lpc54_dev_s *priv,
                                     uint32_t *intset)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (intset)
    {
      *intset = lpc54_serialin(priv, LPC54_USART_FIFOINTENCLR_OFFSET);
    }

  lpc54_serialout(priv, LPC54_USART_FIFOINTENCLR_OFFSET, USART_FIFOINT_ALL);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc54_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int lpc54_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_USART_CONFIG
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;

  /* Configure the USART as an RS-232 USART */

  lpc54_usart_configure(priv->uartbase, &priv->config);
#endif

  /* Make sure that all interrupts are disabled */

  lpc54_fifoint_disableall(priv, NULL);
  return OK;
}

/****************************************************************************
 * Name: lpc54_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed.
 *
 ****************************************************************************/

static void lpc54_shutdown(struct uart_dev_s *dev)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;

  /* Disable interrupts */

  lpc54_fifoint_disableall(priv, NULL);

  /* Reset hardware and disable Rx and Tx */

  lpc54_usart_disable(priv->uartbase);
}

/****************************************************************************
 * Name: lpc54_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method
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

static int lpc54_attach(struct uart_dev_s *dev)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(priv->irq, lpc54_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: lpc54_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port
 *   is closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void lpc54_detach(struct uart_dev_s *dev)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;

  /* Disable interrupts */

  lpc54_fifoint_disableall(priv, NULL);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: lpc54_interrupt
 *
 * Description:
 *   This is the USART status interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int lpc54_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct lpc54_dev_s *priv;
  int passes;
  uint32_t regval;
  bool handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct lpc54_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Read and clear FIFO interrupt status */

      regval = lpc54_serialin(priv, LPC54_USART_FIFOINTSTAT_OFFSET);
      lpc54_serialout(priv, LPC54_USART_FIFOINTSTAT_OFFSET, regval);

      /* Handle incoming, receive bytes.
       * Check if the received FIFO is not empty.
       */

      if ((regval & USART_FIFOINT_RXLVL) != 0)
        {
          /* Process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes.
       * Check if the received FIFO is not full.
       */

      if ((regval & USART_FIFOINT_TXLVL) != 0)
        {
          /* Process outgoing bytes */

          uart_xmitchars(dev);
          handled = true;
        }

#ifdef CONFIG_DEBUG_FEATURES
      /* Check for error conditions */

      if ((regval & CCR_ERROR_EVENTS) != 0)
        {
          /* And now do... what?  Should we reset FIFOs on a FIFO error? */
#warning Missing logic
        }
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: lpc54_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int lpc54_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  struct inode      *inode;
  struct uart_dev_s *dev;
  struct lpc54_dev_s   *priv;
  int                   ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv);
  priv = (struct lpc54_dev_s *)dev->priv;

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
 * Name: lpc54_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int lpc54_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;
  uint32_t fiford;

  /* Get input data along with receiver control information */

  fiford = lpc54_serialin(priv, LPC54_USART_FIFORD_OFFSET);

  /* Return receiver control information */

  if (status)
    {
      *status = fiford && ~USART_FIFORD_RXDATA_MASK;
    }

  /* Then return the actual received data. */

  return fiford & USART_FIFORD_RXDATA_MASK;
}

/****************************************************************************
 * Name: lpc54_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void lpc54_rxint(struct uart_dev_s *dev, bool enable)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Receive an interrupt when there is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

      lpc54_fifoint_enable(priv, CCR_RX_EVENTS);
#endif
    }
  else
    {
      lpc54_fifoint_disable(priv, CCR_RX_EVENTS);
    }
}

/****************************************************************************
 * Name: lpc54_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool lpc54_rxavailable(struct uart_dev_s *dev)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the receive buffer/fifo is not "empty." */

  regval = lpc54_serialin(priv, LPC54_USART_FIFOSTAT_OFFSET);
  return ((regval & USART_FIFOSTAT_RXNOTEMPTY) != 0);
}

/****************************************************************************
 * Name: lpc54_send
 *
 * Description:
 *   This method will send one byte on the USART.
 *
 ****************************************************************************/

static void lpc54_send(struct uart_dev_s *dev, int ch)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;
  lpc54_serialout(priv, LPC54_USART_FIFOWR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: lpc54_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void lpc54_txint(struct uart_dev_s *dev, bool enable)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      irqstate_t flags;

      /* Enable the TX interrupt */

      flags = enter_critical_section();
      lpc54_fifoint_enable(priv, CCR_TX_EVENTS);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
      leave_critical_section(flags);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      lpc54_fifoint_disable(priv, CCR_TX_EVENTS);
    }
}

/****************************************************************************
 * Name: lpc54_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool lpc54_txready(struct uart_dev_s *dev)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the transmit FIFO is "not full." */

  regval = lpc54_serialin(priv, LPC54_USART_FIFOSTAT_OFFSET);
  return ((regval & USART_FIFOSTAT_TXNOTFULL) != 0);
}

/****************************************************************************
 * Name: lpc54_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool lpc54_txempty(struct uart_dev_s *dev)
{
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)dev->priv;
  uint32_t regval;

  /* Return true if the transmit FIFO is "empty." */

  regval = lpc54_serialin(priv, LPC54_USART_FIFOSTAT_OFFSET);
  return ((regval & USART_FIFOSTAT_TXEMPTY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before lpc54_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in lpc54_lowsetup() and main clock
 *   initialization performed in lpc54_clockconfig().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void lpc54_earlyserialinit(void)
{
  /* Disable interrupts from all USARTS. */

  lpc54_fifoint_disableall(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  lpc54_fifoint_disableall(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  lpc54_fifoint_disableall(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  lpc54_fifoint_disableall(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  lpc54_fifoint_disableall(TTYS4_DEV.priv, NULL);
#endif
#ifdef TTYS5_DEV
  lpc54_fifoint_disableall(TTYS5_DEV.priv, NULL);
#endif
#ifdef TTYS6_DEV
  lpc54_fifoint_disableall(TTYS6_DEV.priv, NULL);
#endif
#ifdef TTYS7_DEV
  lpc54_fifoint_disableall(TTYS7_DEV.priv, NULL);
#endif
#ifdef TTYS8_DEV
  lpc54_fifoint_disableall(TTYS8_DEV.priv, NULL);
#endif
#ifdef TTYS9_DEV
  lpc54_fifoint_disableall(TTYS9_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console. */

#ifdef HAVE_USART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  lpc54_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that lpc54_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef HAVE_USART_CONSOLE
  /* Register the serial console */

  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
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
#ifdef TTYS8_DEV
  uart_register("/dev/ttyS8", &TTYS8_DEV);
#endif
#ifdef TTYS9_DEV
  uart_register("/dev/ttyS9", &TTYS9_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes.
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_USART_CONSOLE
  struct lpc54_dev_s *priv = (struct lpc54_dev_s *)CONSOLE_DEV.priv;
  uint32_t intset;

  lpc54_fifoint_disableall(priv, &intset);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  lpc54_fifoint_enable(priv, intset);
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
#ifdef HAVE_USART_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  return ch;
}
#endif

#endif /* HAVE_USART_DEVICE && USE_SERIALDRIVER */
