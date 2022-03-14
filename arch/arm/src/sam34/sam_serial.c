/****************************************************************************
 * arch/arm/src/sam34/sam_serial.c
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

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4CM) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  include "hardware/sam_uart.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4L)
#  include "hardware/sam4l_usart.h"
#else
#  error Unknown UART
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* If the USART is not being used as a UART, then it really isn't enabled
 * for our purposes.
 */

#ifndef CONFIG_USART0_SERIALDRIVER
#  undef CONFIG_SAM34_USART0
#endif
#ifndef CONFIG_USART1_SERIALDRIVER
#  undef CONFIG_SAM34_USART1
#endif
#ifndef CONFIG_USART2_SERIALDRIVER
#  undef CONFIG_SAM34_USART2
#endif
#ifndef CONFIG_USART3_SERIALDRIVER
#  undef CONFIG_SAM34_USART3
#endif

/* Is there a USART/USART enabled? */

#if !defined(CONFIG_SAM34_UART0)  && !defined(CONFIG_SAM34_UART1) && \
    !defined(CONFIG_SAM34_USART0) && !defined(CONFIG_SAM34_USART1) && \
    !defined(CONFIG_SAM34_USART2) && !defined(CONFIG_SAM34_USART3)
#  error "No USARTs enabled"
#endif

#if defined(CONFIG_SAM34_USART0) || defined(CONFIG_SAM34_USART1) ||\
    defined(CONFIG_SAM34_USART2) || defined(CONFIG_SAM34_USART3)
#  define HAVE_USART
#endif

/* Hardware flow control requires using the PDC or DMAC channel for
 * reception
 */

#ifdef CONFIG_SERIAL_IFLOWCONTROL
#  warning PDC or DMAC support is required for RTS hardware flow control
#  undef CONFIG_SERIAL_IFLOWCONTROL
#  undef CONFIG_USART0_IFLOWCONTROL
#  undef CONFIG_USART1_IFLOWCONTROL
#  undef CONFIG_USART2_IFLOWCONTROL
#  undef CONFIG_USART3_IFLOWCONTROL
#endif

/* Is there a serial console?  It could be on UART0-1 or USART0-3 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_SAM34_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_SAM34_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_SAM34_USART0)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_SAM34_USART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_SAM34_USART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_SAM34_USART3)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  ifndef CONFIG_NO_SERIAL_CONSOLE
#    warning "No valid CONFIG_USARTn_SERIAL_CONSOLE Setting"
#  endif

#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

#ifdef USE_SERIALDRIVER

/* Which UART/USART with be tty0/console and which tty1? tty2? tty3?
 * tty4? tty5?
 */

/* First pick the console and ttys0.  This could be any of UART0-1,
 * USART0-3
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port  /* UART0 is console */
#    define TTYS0_DEV           g_uart0port  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port  /* UART1 is console */
#    define TTYS0_DEV           g_uart1port  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE)
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
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart3port /* USART3 is console */
#    define TTYS5_DEV           g_usart3port /* USART3 is ttyS0 */
#else
#  undef CONSOLE_DEV                         /* No console */
#  if defined(CONFIG_SAM34_UART0)
#    define TTYS0_DEV           g_uart0port  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_SAM34_UART1)
#    define TTYS0_DEV           g_uart1port  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_SAM34_USART0)
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#  elif defined(CONFIG_SAM34_USART1)
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED     1
#  elif defined(CONFIG_SAM34_USART2)
#    define TTYS0_DEV           g_usart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#  elif defined(CONFIG_SAM34_USART3)
#    define TTYS0_DEV           g_usart3port /* USART3 is ttyS0 */
#    define USART3_ASSIGNED     1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-1, USART0-3 excluding the
 * console UART.
 */

#if defined(CONFIG_SAM34_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port  /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_SAM34_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port  /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_SAM34_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV           g_usart0port /* USART0 is ttyS1 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS1_DEV           g_usart1port /* USART1 is ttyS1 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS1_DEV           g_usart2port /* USART2 is ttyS1 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS1_DEV           g_usart3port /* USART3 is ttyS1 */
#  define USART3_ASSIGNED     1
#endif

/* Pick ttys2.  This could be one of UART1 or USART0-3. It can't be UART0
 * because that was either assigned as ttyS0 or ttys1.  One of these
 * could also be the console.
 */

#if defined(CONFIG_SAM34_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port  /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_SAM34_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS2_DEV           g_usart0port /* USART0 is ttyS2 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS2_DEV           g_usart1port /* USART1 is ttyS2 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS2_DEV           g_usart2port /* USART2 is ttyS2 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS2_DEV           g_usart3port /* USART3 is ttyS2 */
#  define USART3_ASSIGNED     1
#endif

/* Pick ttys3. This could be one of USART0-3. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * USART0-3 could also be the console.
 */

#if defined(CONFIG_SAM34_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS3_DEV           g_usart0port /* USART0 is ttyS3 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS3_DEV           g_usart1port /* USART1 is ttyS3 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS3_DEV           g_usart2port /* USART2 is ttyS3 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_SAM34_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS3_DEV           g_usart3port /* USART3 is ttyS3 */
#  define USART3_ASSIGNED     1
#endif

/* Pick ttys4. This could be one of USART1-3. It can't be UART0-1 or USART0
 * because those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * USART1-3 could also be the console.
 */

#if defined(CONFIG_SAM34_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS4_DEV           g_usart1port /* USART1 is ttyS4 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAM34_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS4_DEV           g_usart2port /* USART2 is ttyS4 */
#  define USART2_ASSIGNED      1
#elif defined(CONFIG_SAM34_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS4_DEV           g_usart3port /* USART3 is ttyS4 */
#  define USART3_ASSIGNED      1
#endif

/* Pick ttys5. This could be one of USART2-3. It can't be UART0-1 or
 * USART0-1 because those have already been assigned to ttsyS0, 1, 2,
 * 3 or 4.  One of USART2-3 could also be the console.
 */

#if defined(CONFIG_SAM34_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS5_DEV           g_usart2port /* USART2 is ttyS5 */
#  define USART2_ASSIGNED      1
#elif defined(CONFIG_SAM34_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS5_DEV           g_usart3port /* USART3 is ttyS5 */
#  define USART3_ASSIGNED      1
#endif

/* Select MCU-specific settings
 *
 * For the SAM3U, SAM3A, SAM3X, SAM4E and SAM4S the USARTs are driven by the
 *   main clock.  (This could also be the MCK/8 or an external clock but
 *   those options have not yet been necessary).
 * For the SAM4L, the USARTs are driven by CLK_USART (undivided) which is
 *   selected by the PBADIVMASK register.
 */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4CM) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_MR_USCLKS    UART_MR_USCLKS_MCK   /* Source = Main clock */
#  define SAM_USART_CLOCK  BOARD_MCK_FREQUENCY  /* Frequency of the main clock */
#elif defined(CONFIG_ARCH_CHIP_SAM4L)
#  define SAM_MR_USCLKS    UART_MR_USCLKS_USART /* Source = USART_CLK (undefined) */
#  define SAM_USART_CLOCK  BOARD_PBA_FREQUENCY  /* PBA frequency is undivided */
#else
#  error Unrecognized SAM architecture
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  const uint32_t usartbase;     /* Base address of USART registers */
  uint32_t baud;                /* Configured baud */
  uint32_t sr;                  /* Saved status bits */
  uint8_t  irq;                 /* IRQ associated with this USART */
  uint8_t  parity;              /* 0=none, 1=odd, 2=even */
  uint8_t  bits;                /* Number of bits (5-9) */
  bool     stopbits2;           /* true: Configure with 2 stop bits instead of 1 */
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  bool     flowc;               /* input flow control (RTS) enabled */
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

#ifdef CONFIG_SAM34_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM34_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM34_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM34_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM34_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM34_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif

/* This describes the state of the UART0 port. */

#ifdef CONFIG_SAM34_UART0
static struct up_dev_s g_uart0priv =
{
  .usartbase      = SAM_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = SAM_IRQ_UART0,
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

/* This describes the state of the UART1 port. */

#ifdef CONFIG_SAM34_UART1
static struct up_dev_s g_uart1priv =
{
  .usartbase      = SAM_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = SAM_IRQ_UART1,
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

/* This describes the state of the USART0 port. */

#ifdef CONFIG_SAM34_USART0
static struct up_dev_s g_usart0priv =
{
  .usartbase      = SAM_USART0_BASE,
  .baud           = CONFIG_USART0_BAUD,
  .irq            = SAM_IRQ_USART0,
  .parity         = CONFIG_USART0_PARITY,
  .bits           = CONFIG_USART0_BITS,
  .stopbits2      = CONFIG_USART0_2STOP,
#if defined(CONFIG_USART0_OFLOWCONTROL) || defined(CONFIG_USART0_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_usart0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_usart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_usart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart0priv,
};
#endif

/* This describes the state of the USART1 port. */

#ifdef CONFIG_SAM34_USART1
static struct up_dev_s g_usart1priv =
{
  .usartbase      = SAM_USART1_BASE,
  .baud           = CONFIG_USART1_BAUD,
  .irq            = SAM_IRQ_USART1,
  .parity         = CONFIG_USART1_PARITY,
  .bits           = CONFIG_USART1_BITS,
  .stopbits2      = CONFIG_USART1_2STOP,
#if defined(CONFIG_USART1_OFLOWCONTROL) || defined(CONFIG_USART1_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_usart1port =
{
  .recv     =
  {
    .size   = CONFIG_USART1_RXBUFSIZE,
    .buffer = g_usart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART1_TXBUFSIZE,
    .buffer = g_usart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart1priv,
};
#endif

/* This describes the state of the USART2 port. */

#ifdef CONFIG_SAM34_USART2
static struct up_dev_s g_usart2priv =
{
  .usartbase      = SAM_USART2_BASE,
  .baud           = CONFIG_USART2_BAUD,
  .irq            = SAM_IRQ_USART2,
  .parity         = CONFIG_USART2_PARITY,
  .bits           = CONFIG_USART2_BITS,
  .stopbits2      = CONFIG_USART2_2STOP,
#if defined(CONFIG_USART2_OFLOWCONTROL) || defined(CONFIG_USART2_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_usart2port =
{
  .recv       =
    {
      .size   = CONFIG_USART2_RXBUFSIZE,
      .buffer = g_usart2rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_USART2_TXBUFSIZE,
      .buffer = g_usart2txbuffer,
    },
  .ops        = &g_uart_ops,
  .priv       = &g_usart2priv,
};
#endif

/* This describes the state of the USART3 port. */

#ifdef CONFIG_SAM34_USART3
static struct up_dev_s g_usart3priv =
{
  .usartbase      = SAM_USART3_BASE,
  .baud           = CONFIG_USART3_BAUD,
  .irq            = SAM_IRQ_USART3,
  .parity         = CONFIG_USART3_PARITY,
  .bits           = CONFIG_USART3_BITS,
  .stopbits2      = CONFIG_USART3_2STOP,
#if defined(CONFIG_USART3_OFLOWCONTROL) || defined(CONFIG_USART3_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_usart3port =
{
  .recv       =
    {
      .size   = CONFIG_USART3_RXBUFSIZE,
      .buffer = g_usart3rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_USART3_TXBUFSIZE,
      .buffer = g_usart3txbuffer,
    },
  .ops        = &g_uart_ops,
  .priv       = &g_usart3priv,
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
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static inline void up_restoreusartint(struct up_dev_s *priv, uint32_t imr)
{
  /* Re-enable previously disabled interrupts state (assuming all interrupts
   * disabled)
   */

  up_serialout(priv, SAM_UART_IER_OFFSET, imr);
}

/****************************************************************************
 * Name: up_disableallints
 ****************************************************************************/

static void up_disableallints(struct up_dev_s *priv, uint32_t *imr)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = enter_critical_section();
  if (imr)
    {
      /* Return the current interrupt mask */

      *imr = up_serialin(priv, SAM_UART_IMR_OFFSET);
    }

  /* Disable all interrupts */

  up_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_ALLINTS);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;
  uint32_t imr;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled and the pins were configured in sam_lowsetup().
   */

  /* The shutdown method will put the UART in a known, disabled state */

  up_disableallints(priv, &imr);
  up_shutdown(dev);

  /* Set up the mode register.  Start with normal UART mode and the MCK
   * as the timing source
   */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  /* "Setting the USART to operate with hardware handshaking is performed by
   *  writing the USART_MODE field in the Mode Register (US_MR) to the value
   *  0x2. ... Using this mode requires using the PDC or DMAC channel for
   *  reception. The transmitter can handle hardware handshaking in any
   *  case."
   */

  if (priv->flowc)
    {
      /* Enable hardware flow control and MCK as the timing source */

      regval = (UART_MR_MODE_HWHS | SAM_MR_USCLKS);
    }
  else
#endif
    {
      /* Set up the mode register.  Start with normal UART mode and the MCK
       * as the timing source
       */

      regval = (UART_MR_MODE_NORMAL | SAM_MR_USCLKS);
    }

  /* OR in settings for the selected number of bits */

  if (priv->bits == 5)
    {
      regval |= UART_MR_CHRL_5BITS; /* 5 bits */
    }
  else if (priv->bits == 6)
    {
      regval |= UART_MR_CHRL_6BITS;  /* 6 bits */
    }
  else if (priv->bits == 7)
    {
      regval |= UART_MR_CHRL_7BITS; /* 7 bits */
    }
#ifdef HAVE_USART
  else if (priv->bits == 9
#if defined(CONFIG_SAM34_UART0)
           && priv->usartbase != SAM_UART0_BASE
#endif
#if defined(CONFIG_SAM34_UART1)
           && priv->usartbase != SAM_UART1_BASE
#endif
          )
    {
      regval |= UART_MR_MODE9; /* 9 bits */
    }
#endif
  else /* if (priv->bits == 8) */
    {
      regval |= UART_MR_CHRL_8BITS; /* 8 bits (default) */
    }

  /* OR in settings for the selected parity */

  if (priv->parity == 1)
    {
      regval |= UART_MR_PAR_ODD;
    }
  else if (priv->parity == 2)
    {
      regval |= UART_MR_PAR_EVEN;
    }
  else
    {
      regval |= UART_MR_PAR_NONE;
    }

  /* OR in settings for the number of stop bits */

  if (priv->stopbits2)
    {
      regval |= UART_MR_NBSTOP_2;
    }
  else
    {
      regval |= UART_MR_NBSTOP_1;
    }

#ifdef CONFIG_SAM34_UART1_OPTICAL
  if (priv == &g_uart1priv)
    {
      /* Enable optical mode. */

      regval |= UART_MR_OPT_EN;
    }
#endif

  /* And save the new mode register value */

  up_serialout(priv, SAM_UART_MR_OFFSET, regval);

  /* Configure the console baud:
   *
   *   Fbaud   = USART_CLOCK / (16 * divisor)
   *   divisor = USART_CLOCK / (16 * Fbaud)
   *
   * NOTE: Oversampling by 8 is not supported. This may limit BAUD rates
   * for lower USART clocks.
   */

  regval  = (SAM_USART_CLOCK + (priv->baud << 3)) / (priv->baud << 4);
  up_serialout(priv, SAM_UART_BRGR_OFFSET, regval);

  /* Enable receiver & transmitter */

  up_serialout(priv, SAM_UART_CR_OFFSET, (UART_CR_RXEN | UART_CR_TXEN));
  up_restoreusartint(priv, imr);

#endif

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Reset and disable receiver and transmitter */

  up_serialout(priv, SAM_UART_CR_OFFSET,
               (UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS |
                UART_CR_TXDIS));

  /* Disable all interrupts */

  up_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
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

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the USART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
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
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t           pending;
  uint32_t           imr;
  int                passes;
  bool               handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or, until we have
   * been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the UART/USART status (we are only interested in the unmasked
       * interrupts).
       */

      priv->sr = up_serialin(priv, SAM_UART_SR_OFFSET);  /* Save for error reporting */
      imr      = up_serialin(priv, SAM_UART_IMR_OFFSET); /* Interrupt mask */
      pending  = priv->sr & imr;                         /* Mask out disabled interrupt sources */

      /* Handle an incoming, receive byte.  RXRDY: At least one complete
       * character has been received and US_RHR has not yet been read.
       */

      if ((pending & UART_INT_RXRDY) != 0)
        {
          /* Received data ready... process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes. TXRDY: There is no character in the
       * US_THR.
       */

      if ((pending & UART_INT_TXRDY) != 0)
        {
          /* Transmit data register empty ... process outgoing bytes */

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
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
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

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct up_dev_s *priv     = (struct up_dev_s *)dev->priv;

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

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        termiosp->c_cflag |= (priv->flowc) ? (CCTS_OFLOW | CRTS_IFLOW): 0;
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
        struct up_dev_s *priv     = (struct up_dev_s *)dev->priv;
        uint32_t baud;
        uint32_t imr;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        bool flowc;
#endif

        if (!termiosp)
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

        /* Decode flow control */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        flowc = (termiosp->c_cflag & (CCTS_OFLOW | CRTS_IFLOW)) != 0;
#endif
        /* Verify that all settings are valid before committing */

        if (ret == OK)
          {
            /* Commit */

            priv->baud      = baud;
            priv->parity    = parity;
            priv->bits      = nbits;
            priv->stopbits2 = stop2;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
            priv->flowc     = flowc;
#endif
            /* effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            up_disableallints(priv, &imr);
            ret = up_setup(dev);

            /* Restore the interrupt state */

            up_restoreusartint(priv, imr);
          }
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
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return the error information in the saved status */

  *status  = priv->sr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return (int)(up_serialin(priv, SAM_UART_RHR_OFFSET) & 0xff);
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an RX timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_serialout(priv, SAM_UART_IER_OFFSET, UART_INT_RXRDY);
#endif
    }
  else
    {
      up_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_RXRDY);
    }
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_RXRDY) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART/USART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, SAM_UART_THR_OFFSET, (uint32_t)ch);
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
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_serialout(priv, SAM_UART_IER_OFFSET, UART_INT_TXRDY);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);

#endif
    }
  else
    {
      /* Disable the TX interrupt */

      up_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_TXRDY);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_TXRDY) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_TXEMPTY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the USARTs was performed in
   * sam_lowsetup
   */

  /* Disable all USARTS */

  up_disableallints(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableallints(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableallints(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  up_disableallints(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  up_disableallints(TTYS4_DEV.priv, NULL);
#endif
#ifdef TTYS5_DEV
  up_disableallints(TTYS5_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

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

#ifdef HAVE_CONSOLE
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
}

#endif /* USE_SERIALDRIVER */
