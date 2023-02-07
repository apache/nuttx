/****************************************************************************
 * arch/arm/src/tiva/common/tiva_serial.c
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
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "tiva_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we
 * still must provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Some sanity checks *******************************************************/

/* Is there a UART enabled? */

#if !defined(CONFIG_TIVA_UART0) && !defined(CONFIG_TIVA_UART1) && !defined(CONFIG_TIVA_UART2) && \
    !defined(CONFIG_TIVA_UART3) && !defined(CONFIG_TIVA_UART4) && !defined(CONFIG_TIVA_UART5) && \
    !defined(CONFIG_TIVA_UART6) && !defined(CONFIG_TIVA_UART7)
#  error "No UARTs enabled"
#endif

/* Which UART will be tty0/console and which tty1-7?  The console will always
 * be ttyS0.  If there is no console then we'll use the lowest numbered UART.
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
#    define TTYS0_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart6port /* UART6 is console */
#    define TTYS0_DEV           g_uart6port /* UART6 is ttyS0 */
#    define UART6_ASSIGNED      1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart7port /* UART7 is console */
#    define TTYS0_DEV           g_uart7port /* UART7 is ttyS0 */
#    define UART7_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_TIVA_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_TIVA_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_TIVA_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(CONFIG_TIVA_UART3)
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  elif defined(CONFIG_TIVA_UART4)
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  elif defined(CONFIG_TIVA_UART5)
#    define TTYS0_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#  elif defined(CONFIG_TIVA_UART6)
#    define TTYS0_DEV           g_uart6port /* UART6 is ttyS0 */
#    define UART6_ASSIGNED      1
#  elif defined(CONFIG_TIVA_UART7)
#    define TTYS0_DEV           g_uart7port /* UART7 is ttyS0 */
#    define UART7_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-7 excluding the console UART. */

#if defined(CONFIG_TIVA_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS1_DEV           g_uart5port /* UART5 is ttyS1 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS1_DEV           g_uart6port /* UART6 is ttyS1 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS1_DEV           g_uart7port /* UART7 is ttyS1 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART1-7. It can't be UART0 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-7 could also be the
 * console.
 */

#if defined(CONFIG_TIVA_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS2_DEV           g_uart5port /* UART5 is ttyS2 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS2_DEV           g_uart6port /* UART6 is ttyS2 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS2_DEV           g_uart7port /* UART7 is ttyS2 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART2-7. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART 2-7 could also be the console.
 */

#if defined(CONFIG_TIVA_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS3_DEV           g_uart2port /* UART2 is ttyS3 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS3_DEV           g_uart5port /* UART5 is ttyS3 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS3_DEV           g_uart6port /* UART6 is ttyS3 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS3_DEV           g_uart7port /* UART7 is ttyS3 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys4. This could be one of UART3-7. It can't be UART0-2 because
 * those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * UART 3-7 could also be the console.
 */

#if defined(CONFIG_TIVA_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS4_DEV           g_uart3port /* UART3 is ttyS4 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS4_DEV           g_uart5port /* UART5 is ttyS4 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS4_DEV           g_uart6port /* UART6 is ttyS4 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS4_DEV           g_uart7port /* UART7 is ttyS4 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys5. This could be one of UART4-7. It can't be UART0-3 because
 * those have already been assigned to ttsyS0, 1, 2, 3 or 4.  One of
 * UART 4-7 could also be the console.
 */

#if defined(CONFIG_TIVA_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS5_DEV           g_uart4port /* UART4 is ttyS5 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS5_DEV           g_uart5port /* UART5 is ttyS5 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS5_DEV           g_uart6port /* UART6 is ttyS5 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS5_DEV           g_uart7port /* UART7 is ttyS5 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys6. This could be one of UART5-7. It can't be UART0-4 because
 * those have already been assigned to ttsyS0, 1, 2, 3, 4, or 5.  One of
 * UART 5-7 could also be the console.
 */

#if defined(CONFIG_TIVA_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS6_DEV           g_uart5port /* UART5 is ttyS6 */
#  define UART5_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS6_DEV           g_uart6port /* UART6 is ttyS6 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS6_DEV           g_uart7port /* UART7 is ttyS6 */
#  define UART7_ASSIGNED      1
#endif

/* Pick ttys7. This could be one of UART6-7. It can't be UART0-5 because
 * those have already been assigned to ttsyS0, 1, 2, 3, 4, 5, or 6.  One
 * of UART 6-7 could also be the console.
 */

#if defined(CONFIG_TIVA_UART6) && !defined(UART6_ASSIGNED)
#  define TTYS7_DEV           g_uart6port /* UART6 is ttyS7 */
#  define UART6_ASSIGNED      1
#elif defined(CONFIG_TIVA_UART7) && !defined(UART7_ASSIGNED)
#  define TTYS7_DEV           g_uart7port /* UART7 is ttyS7 */
#  define UART7_ASSIGNED      1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t im;        /* Saved IM value */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool     iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool     oflow;     /* output flow control (CTS) enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
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

#ifdef CONFIG_TIVA_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_TIVA_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_TIVA_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif
#ifdef CONFIG_TIVA_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif
#ifdef CONFIG_TIVA_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif
#ifdef CONFIG_TIVA_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
#endif
#ifdef CONFIG_TIVA_UART6
static char g_uart6rxbuffer[CONFIG_UART6_RXBUFSIZE];
static char g_uart6txbuffer[CONFIG_UART6_TXBUFSIZE];
#endif
#ifdef CONFIG_TIVA_UART7
static char g_uart7rxbuffer[CONFIG_UART7_RXBUFSIZE];
static char g_uart7txbuffer[CONFIG_UART7_TXBUFSIZE];
#endif

/* This describes the state of the Stellaris uart0 port. */

#ifdef CONFIG_TIVA_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = TIVA_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = TIVA_IRQ_UART0,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .stopbits2      = CONFIG_UART0_2STOP,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART0_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART0_OFLOWCONTROL)
  .oflow          = true,
#endif
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

/* This describes the state of the Stellaris uart1 port. */

#ifdef CONFIG_TIVA_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = TIVA_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = TIVA_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART1_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART1_OFLOWCONTROL)
  .oflow          = true,
#endif
};

static uart_dev_t g_uart1port =
{
  .recv       =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },
  .ops        = &g_uart_ops,
  .priv       = &g_uart1priv,
};
#endif

/* This describes the state of the Stellaris uart2 port. */

#ifdef CONFIG_TIVA_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = TIVA_UART2_BASE,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = TIVA_IRQ_UART2,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
  .stopbits2      = CONFIG_UART2_2STOP,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART2_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART2_OFLOWCONTROL)
  .oflow          = true,
#endif
};

static uart_dev_t g_uart2port =
{
  .recv       =
    {
      .size   = CONFIG_UART2_RXBUFSIZE,
      .buffer = g_uart2rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_UART2_TXBUFSIZE,
      .buffer = g_uart2txbuffer,
    },
  .ops        = &g_uart_ops,
  .priv       = &g_uart2priv,
};
#endif

/* This describes the state of the Stellaris uart3 port. */

#ifdef CONFIG_TIVA_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = TIVA_UART3_BASE,
  .baud           = CONFIG_UART3_BAUD,
  .irq            = TIVA_IRQ_UART3,
  .parity         = CONFIG_UART3_PARITY,
  .bits           = CONFIG_UART3_BITS,
  .stopbits2      = CONFIG_UART3_2STOP,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART3_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART3_OFLOWCONTROL)
  .oflow          = true,
#endif
};

static uart_dev_t g_uart3port =
{
  .recv       =
    {
      .size   = CONFIG_UART3_RXBUFSIZE,
      .buffer = g_uart3rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_UART3_TXBUFSIZE,
      .buffer = g_uart3txbuffer,
    },
  .ops        = &g_uart_ops,
  .priv       = &g_uart3priv,
};
#endif

/* This describes the state of the Stellaris uart4 port. */

#ifdef CONFIG_TIVA_UART4
static struct up_dev_s g_uart4priv =
{
  .uartbase       = TIVA_UART4_BASE,
  .baud           = CONFIG_UART4_BAUD,
  .irq            = TIVA_IRQ_UART4,
  .parity         = CONFIG_UART4_PARITY,
  .bits           = CONFIG_UART4_BITS,
  .stopbits2      = CONFIG_UART4_2STOP,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART4_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART4_OFLOWCONTROL)
  .oflow          = true,
#endif
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

/* This describes the state of the Stellaris uart5 port. */

#ifdef CONFIG_TIVA_UART5
static struct up_dev_s g_uart5priv =
{
  .uartbase       = TIVA_UART5_BASE,
  .baud           = CONFIG_UART5_BAUD,
  .irq            = TIVA_IRQ_UART5,
  .parity         = CONFIG_UART5_PARITY,
  .bits           = CONFIG_UART5_BITS,
  .stopbits2      = CONFIG_UART5_2STOP,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART5_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART5_OFLOWCONTROL)
  .oflow          = true,
#endif
};

static uart_dev_t g_uart5port =
{
  .recv       =
    {
      .size   = CONFIG_UART5_RXBUFSIZE,
      .buffer = g_uart5rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_UART5_TXBUFSIZE,
      .buffer = g_uart5txbuffer,
    },
  .ops        = &g_uart_ops,
  .priv       = &g_uart5priv,
};
#endif

/* This describes the state of the Stellaris uart6 port. */

#ifdef CONFIG_TIVA_UART6
static struct up_dev_s g_uart6priv =
{
  .uartbase       = TIVA_UART6_BASE,
  .baud           = CONFIG_UART6_BAUD,
  .irq            = TIVA_IRQ_UART6,
  .parity         = CONFIG_UART6_PARITY,
  .bits           = CONFIG_UART6_BITS,
  .stopbits2      = CONFIG_UART6_2STOP,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART6_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART6_OFLOWCONTROL)
  .oflow          = true,
#endif
};

static uart_dev_t g_uart6port =
{
  .recv       =
    {
      .size   = CONFIG_UART6_RXBUFSIZE,
      .buffer = g_uart6rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_UART6_TXBUFSIZE,
      .buffer = g_uart6txbuffer,
    },
  .ops        = &g_uart_ops,
  .priv       = &g_uart6priv,
};
#endif

/* This describes the state of the Stellaris uart7 port. */

#ifdef CONFIG_TIVA_UART7
static struct up_dev_s g_uart7priv =
{
  .uartbase       = TIVA_UART7_BASE,
  .baud           = CONFIG_UART7_BAUD,
  .irq            = TIVA_IRQ_UART7,
  .parity         = CONFIG_UART7_PARITY,
  .bits           = CONFIG_UART7_BITS,
  .stopbits2      = CONFIG_UART7_2STOP,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART7_IFLOWCONTROL)
  .iflow          = true,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART7_OFLOWCONTROL)
  .oflow          = true,
#endif
};

static uart_dev_t g_uart7port =
{
  .recv       =
    {
      .size   = CONFIG_UART7_RXBUFSIZE,
      .buffer = g_uart7rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_UART7_TXBUFSIZE,
      .buffer = g_uart7txbuffer,
    },
  .ops        = &g_uart_ops,
  .priv       = &g_uart7priv,
};
#endif

/****************************************************************************
 * Private Functions
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

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *im)
{
  /* Return the current interrupt mask value */

  if (im)
    {
      *im = priv->im;
    }

  /* Disable all interrupts */

  priv->im = 0;
  up_serialout(priv, TIVA_UART_IM_OFFSET, 0);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t im)
{
  priv->im = im;
  up_serialout(priv, TIVA_UART_IM_OFFSET, im);
}

/****************************************************************************
 * Name: up_waittxnotfull
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
static inline void up_waittxnotfull(struct up_dev_s *priv)
{
  volatile int tmp;

  /* Limit how long we will wait for the TX available condition */

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Check Tx FIFO is full */

      if ((up_serialin(priv, TIVA_UART_FR_OFFSET) & UART_FR_TXFF) == 0)
        {
          /* The Tx FIFO is not full... return */

          break;
        }
    }

  /* If we get here, then the wait has timed out and the Tx FIFO remains
   * full.
   */
}
#endif

/****************************************************************************
 * Name: up_set_format
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc.
 *
 ****************************************************************************/

static void up_set_format(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t den;
  uint32_t brdi;
  uint32_t remainder;
  uint32_t divfrac;
  uint32_t lcrh;
  uint32_t ctl;
  bool     was_active;

  /* Note:  The logic here depends on the fact that that the UART module
   * was enabled and the GPIOs were configured in tiva_lowsetup().
   */

  /* Disable the UART by clearing the UARTEN bit in the UART CTL register */

  ctl = up_serialin(priv, TIVA_UART_CTL_OFFSET);
  was_active = (ctl & UART_CTL_UARTEN) != 0;

  if (was_active)
    {
      ctl &= ~UART_CTL_UARTEN;
      up_serialout(priv, TIVA_UART_CTL_OFFSET, ctl);
    }

  /* Calculate BAUD rate from the SYS clock:
   *
   * "The baud-rate divisor is a 22-bit number consisting of a 16-bit integer
   *  and a 6-bit fractional part. The number formed by these two values is
   *  used by the baud-rate generator to determine the bit period. Having a
   *  fractional baud-rate divider allows the UART to generate all the
   *  standard baud rates.
   *
   * "The 16-bit integer is loaded through the UART Integer Baud-Rate Divisor
   *  (UARTIBRD) register ... and the 6-bit fractional part is loaded with
   *  the UART Fractional Baud-Rate Divisor (UARTFBRD) register... The
   *  baud-rate divisor (BRD) has the following relationship to the system
   *  clock (where BRDI is the integer part of the BRD and BRDF is the
   *  fractional part, separated by a decimal place.):
   *
   *    "BRD = BRDI + BRDF = UARTSysClk / (16 * Baud Rate)
   *
   * "where UARTSysClk is the system clock connected to the UART. The 6-bit
   *  fractional number (that is to be loaded into the DIVFRAC bit field in
   *  the UARTFBRD register) can be calculated by taking the fractional part
   *  of the baud-rate divisor, multiplying it by 64, and adding 0.5 to
   *  account for rounding errors:
   *
   *    "UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5)
   *
   * "The UART generates an internal baud-rate reference clock at 16x the
   *  baud-rate (referred to as Baud16). This reference clock is divided by
   *  16 to generate the transmit clock, and is used for error detection
   *  during receive operations.
   *
   * "Along with the UART Line Control, High Byte (UARTLCRH) register ...,
   *  the UARTIBRD and UARTFBRD registers form an internal 30-bit register.
   *  This internal register is only updated when a write operation to
   *  UARTLCRH is performed, so any changes to the baud-rate divisor must be
   *  followed by a write to the UARTLCRH register for the changes to take
   *  effect. ..."
   */

  den       = priv->baud << 4;
  brdi      = SYSCLK_FREQUENCY / den;
  remainder = SYSCLK_FREQUENCY - den * brdi;
  divfrac   = ((remainder << 6) + (den >> 1)) / den;

  up_serialout(priv, TIVA_UART_IBRD_OFFSET, brdi);
  up_serialout(priv, TIVA_UART_FBRD_OFFSET, divfrac);

  /* Set up the LCRH register */

  lcrh = 0;
  switch (priv->bits)
    {
      case 5:
          lcrh |= UART_LCRH_WLEN_5BITS;
          break;

      case 6:
          lcrh |= UART_LCRH_WLEN_6BITS;
          break;

      case 7:
          lcrh |= UART_LCRH_WLEN_7BITS;
          break;

      case 8:
      default:
          lcrh |= UART_LCRH_WLEN_8BITS;
          break;
    }

  switch (priv->parity)
    {
      case 0:
      default:
          break;
      case 1:
          lcrh |= UART_LCRH_PEN;
          break;
      case 2:
          lcrh |= UART_LCRH_PEN | UART_LCRH_EPS;
          break;
    }

  if (priv->stopbits2)
    {
      lcrh |= UART_LCRH_STP2;
    }

  up_serialout(priv, TIVA_UART_LCRH_OFFSET, lcrh);

  /* Enable or disable CTS/RTS, if applicable */

#if defined(CONFIG_SERIAL_IFLOWCONTROL)
  if (priv->iflow)
    {
      ctl |= UART_CTL_RTSEN;
    }
  else
    {
      ctl &= ~UART_CTL_RTSEN;
    }
#endif

#if defined(CONFIG_SERIAL_OFLOWCONTROL)
  if (priv->oflow)
    {
      ctl |= UART_CTL_CTSEN;
    }
  else
    {
      ctl &= ~UART_CTL_CTSEN;
    }
#endif

  if (was_active)
    {
      ctl |= UART_CTL_UARTEN;
    }

  up_serialout(priv, TIVA_UART_CTL_OFFSET, ctl);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t lcrh;
  uint32_t ctl;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  up_set_format(dev);
#endif

  /* Set the UART to interrupt whenever the TX FIFO is almost empty or when
   * any character is received.
   */

  up_serialout(priv, TIVA_UART_IFLS_OFFSET,
               UART_IFLS_TXIFLSEL_18TH | UART_IFLS_RXIFLSEL_18TH);

  /* Flush the Rx and Tx FIFOs -- How do you do that? */

  /* Enable Rx interrupts from the UART except for Tx interrupts.  We don't
   * want TX interrupts until we have something to send.  We will check for
   * serial errors as part of Rx interrupt processing (no interrupts will be
   * received yet because the interrupt is still disabled at the interrupt
   * controller.
   */

  up_serialout(priv, TIVA_UART_IM_OFFSET, UART_IM_RXIM | UART_IM_RTIM);

  /* Enable the FIFOs */

  lcrh = up_serialin(priv, TIVA_UART_LCRH_OFFSET);
  lcrh |= UART_LCRH_FEN;
  up_serialout(priv, TIVA_UART_LCRH_OFFSET, lcrh);

  /* Enable Rx, Tx, and the UART */

  ctl = up_serialin(priv, TIVA_UART_CTL_OFFSET);
  ctl |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);

  up_serialout(priv, TIVA_UART_CTL_OFFSET, ctl);

  /* Set up the cache IM value */

  priv->im = up_serialin(priv, TIVA_UART_IM_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
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
 *   Configure the UART to operate in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, dev);
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
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t           mis;
  int                passes;
  bool               handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the masked UART status and clear the pending interrupts. */

       mis = up_serialin(priv, TIVA_UART_MIS_OFFSET);
       up_serialout(priv, TIVA_UART_ICR_OFFSET, mis);

      /* Handle incoming, receive bytes (with or without timeout) */

      if ((mis & (UART_MIS_RXMIS | UART_MIS_RTMIS)) != 0)
        {
          /* Rx buffer not empty ... process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((mis & UART_MIS_TXMIS) != 0)
        {
          /* Tx FIFO not full ... process outgoing bytes */

          uart_xmitchars(dev);
          handled = true;
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
#if defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || defined(CONFIG_SERIAL_TERMIOS)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
#if defined(CONFIG_SERIAL_TERMIOS)
  struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
#endif
  int                ret   = OK;

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

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        tcflag_t ccflag = 0;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        if (priv->bits >= 5 && priv->bits <= 8)
          {
            ccflag |= (CS5 + (priv->bits - 5));
          }

        if (priv->stopbits2)
          {
            ccflag |= CSTOPB;
          }

        if (priv->parity == 1)
          {
            ccflag |= PARENB;
          }
        else if (priv->parity == 2)
          {
            ccflag |= PARENB | PARODD;
          }

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        if (priv->oflow)
          {
            ccflag |= CCTS_OFLOW;
          }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
        if (priv->iflow)
          {
            ccflag |= CRTS_IFLOW;
          }
#endif

        /* TODO append support for HUPCL and CLOCAL as well as os-compliant
         * break sequence
         */

        termiosp->c_cflag = ccflag;

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Perform some sanity checks before accepting any changes */

#ifndef CONFIG_SERIAL_OFLOWCONTROL
        if (termiosp->c_cflag & CCTS_OFLOW)
          {
            /* CTS not supported in this build, so report error */

            ret = -EINVAL;
            break;
          }
#endif /* !CONFIG_SERIAL_OFLOWCONTROL */

#ifndef CONFIG_SERIAL_IFLOWCONTROL
        if (termiosp->c_cflag & CRTS_IFLOW)
          {
            /* RTS not supported in this build, so report error */

            ret = -EINVAL;
            break;
          }
#endif /* !CONFIG_SERIAL_IFLOWCONTROL */

        if (termiosp->c_cflag & PARENB)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
        priv->bits      = (termiosp->c_cflag & CSIZE) + 5;
        priv->baud      = cfgetispeed(termiosp);

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

        /* Effect the changes immediately - note that we do not implement
         * TCSADRAIN / TCSAFLUSH
         */

        up_set_format(dev);
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

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

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rxd;

  /* Get the Rx byte + 4 bits of error information.  Return those in status */

  rxd     = up_serialin(priv, TIVA_UART_DR_OFFSET);
  *status = rxd;

  /* The lower 8bits of the Rx data is the actual received byte */

  return rxd & 0xff;
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
      /* Receive an interrupt when there is anything in the Rx FIFO (or an Rx
       * timeout occurs.
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= (UART_IM_RXIM | UART_IM_RTIM);
#endif
    }
  else
    {
      priv->im &= ~(UART_IM_RXIM | UART_IM_RTIM);
    }

  up_serialout(priv, TIVA_UART_IM_OFFSET, priv->im);
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
  return ((up_serialin(priv, TIVA_UART_FR_OFFSET) & UART_FR_RXFE) == 0);
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
  up_serialout(priv, TIVA_UART_DR_OFFSET, (uint32_t)ch);
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
      /* Set to receive an interrupt when the TX fifo is half emptied */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_IM_TXIM;
      up_serialout(priv, TIVA_UART_IM_OFFSET, priv->im);

      /* The serial driver wants an interrupt here, but will not get
       * one unless we "prime the pump."  I believe that this is because
       * behave like a level interrupt and the Stellaris interrupts behave
       * (at least by default) like edge interrupts.
       *
       * In any event, faking a TX interrupt here solves the problem;
       * Call uart_xmitchars() just as would have been done if we received
       * the TX interrupt.
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~UART_IM_TXIM;
      up_serialout(priv, TIVA_UART_IM_OFFSET, priv->im);
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
  return ((up_serialin(priv, TIVA_UART_FR_OFFSET) & UART_FR_TXFF) == 0);
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
  return ((up_serialin(priv, TIVA_UART_FR_OFFSET) & UART_FR_TXFE) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the UARTs was performed in
   * tiva_lowsetup
   */

  /* Disable all UARTS */

  up_disableuartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableuartint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableuartint(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  up_disableuartint(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  up_disableuartint(TTYS4_DEV.priv, NULL);
#endif
#ifdef TTYS5_DEV
  up_disableuartint(TTYS5_DEV.priv, NULL);
#endif
#ifdef TTYS6_DEV
  up_disableuartint(TTYS6_DEV.priv, NULL);
#endif
#ifdef TTYS7_DEV
  up_disableuartint(TTYS7_DEV.priv, NULL);
#endif

  /* Configure whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif /* !USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
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
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t im;

  up_disableuartint(priv, &im);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxnotfull(priv);
      up_serialout(priv, TIVA_UART_DR_OFFSET, (uint32_t)'\r');
    }

  up_waittxnotfull(priv);
  up_serialout(priv, TIVA_UART_DR_OFFSET, (uint32_t)ch);

  up_waittxnotfull(priv);
  up_restoreuartint(priv, im);
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
#ifdef HAVE_SERIAL_CONSOLE
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
