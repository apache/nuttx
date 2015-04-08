/****************************************************************************
 * arch/arm/src/samv7/sam_serial.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/serial.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "sam_config.h"
#include "chip/sam_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/* Which UART/USART with be tty0/console and which tty1-7? */

/* First pick the console and ttys0.  This could be any of UART0-4, USART0-2 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port  /* UART0 is console */
#    define TTYS0_DEV           g_uart0port  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port  /* UART1 is console */
#    define TTYS0_DEV           g_uart1port  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart2port  /* UART2 is console */
#    define TTYS0_DEV           g_uart2port  /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart3port  /* UART3 is console */
#    define TTYS0_DEV           g_uart3port  /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart4port  /* UART4 is console */
#    define TTYS0_DEV           g_uart4port  /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
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
#else
#  undef CONSOLE_DEV                         /* No console */
#  if defined(CONFIG_SAMV7_UART0)
#    define TTYS0_DEV           g_uart0port  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_SAMV7_UART1)
#    define TTYS0_DEV           g_uart1port  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_SAMV7_UART2)
#    define TTYS0_DEV           g_uart2port  /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(CONFIG_SAMV7_UART3)
#    define TTYS0_DEV           g_uart3port  /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  elif defined(CONFIG_SAMV7_UART4)
#    define TTYS0_DEV           g_uart4port  /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  elif defined(CONFIG_SAMV7_USART0)
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#  elif defined(CONFIG_SAMV7_USART1)
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED     1
#  elif defined(CONFIG_SAMV7_USART2)
#    define TTYS0_DEV           g_usart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-4, USART0-2 excluding the console
 * UART.
 */

#if defined(CONFIG_SAMV7_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port  /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port  /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port  /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port  /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port  /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV           g_usart0port /* USART0 is ttyS1 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS1_DEV           g_usart1port /* USART1 is ttyS1 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS1_DEV           g_usart2port /* USART2 is ttyS1 */
#  define USART2_ASSIGNED     1
#endif

/* Pick ttys2.  This could be one of UART1-4 or USART0-1. It can't be UART0
 * because that was either assigned as ttyS0 or ttys1.  One of these
 * could also be the console.
 */

#if defined(CONFIG_SAMV7_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port  /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port  /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port  /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port  /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS2_DEV           g_usart0port /* USART0 is ttyS2 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS2_DEV           g_usart1port /* USART1 is ttyS2 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS2_DEV           g_usart2port /* USART2 is ttyS2 */
#  define USART2_ASSIGNED     1
#endif

/* Pick ttys3. This could be one of UART2-4 or USART0-2. It can't be UART0-1
 * because those have already been assigned to ttsyS0, 1, or 2.  One of
 * these could also be the console.
 */

#if defined(CONFIG_SAMV7_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS3_DEV           g_uart2port  /* UART2 is ttyS3 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port  /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port  /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS3_DEV           g_usart0port /* USART0 is ttyS3 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS3_DEV           g_usart1port /* USART1 is ttyS3 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS3_DEV           g_usart2port /* USART2 is ttyS3 */
#  define USART2_ASSIGNED     1
#endif

/* Pick ttys4. This could be one of UART3-4 or USART0-2. It can't be UART0-2
 * because those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * these could also be the console.
 */

#if defined(CONFIG_SAMV7_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS4_DEV           g_uart3port  /* UART3 is ttyS4 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_SAMV7_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port  /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS4_DEV           g_usart0port /* USART0 is ttyS4 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS4_DEV           g_usart1port /* USART1 is ttyS4 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS4_DEV           g_usart2port /* USART2 is ttyS4 */
#  define USART2_ASSIGNED      1
#endif

/* Pick ttys5. This could be one of UART4 or USART0-2. It can't be UART0-3
 * because those have already been assigned to ttsyS0, 1, 2, 3 or 4.  One
 * of these could also be the console.
 */

#if defined(CONFIG_SAMV7_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS5_DEV           g_uart4port  /* UART4 is ttyS5 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS5_DEV           g_usart0port /* USART0 is ttyS5 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS5_DEV           g_usart1port /* USART1 is ttyS5 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS5_DEV           g_usart2port /* USART2 is ttyS5 */
#  define USART2_ASSIGNED      1
#endif

/* Pick ttys6. This could be one of USART0-2. It can't be UART0-4
 * because those have already been assigned to ttsyS0-5.  One of
 * One of USART0-2 could also be the console.
 */

#if defined(CONFIG_SAMV7_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS6_DEV           g_usart0port /* USART0 is ttyS6 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS6_DEV           g_usart1port /* USART1 is ttyS6 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS6_DEV           g_usart2port /* USART2 is ttyS6 */
#  define USART2_ASSIGNED      1
#endif

/* Pick ttys7. This could be one of USART1-2. It can't be UART0-4
 * or USART 1 because those have already been assigned to ttsyS0-6.
 * One of of USART1-2 could also be the console.
 */

#if defined(CONFIG_SAMV7_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS7_DEV           g_usart1port /* USART1 is ttyS7 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS7_DEV           g_usart2port /* USART2 is ttyS7 */
#  define USART2_ASSIGNED      1
#endif

/* BAUD definitions
 *
 * The source clock is selectable and could be one of:
 *
 *   - The peripheral clock
 *   - A division of the peripheral clock, where the divider is product-
 *     dependent, but generally set to 8
 *   - A processor/peripheral independent clock source fully programmable
 *      provided by PMC (PCK)
 *   - The external clock, available on the SCK pin
 *
 * Only the first two options are supported by this driver.  The divided
 * peripheral clock is only used for very low BAUD selections.
 */

#define FAST_USART_CLOCK   BOARD_MCK_FREQUENCY
#define SLOW_USART_CLOCK   (BOARD_MCK_FREQUENCY >> 3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_dev_s
{
  const uint32_t usartbase;     /* Base address of USART registers */
  xcpt_t   handler;             /* Interrupt handler */
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

static int  sam_setup(struct uart_dev_s *dev);
static void sam_shutdown(struct uart_dev_s *dev);
static int  sam_attach(struct uart_dev_s *dev);
static void sam_detach(struct uart_dev_s *dev);
static int sam_interrupt(struct uart_dev_s *dev);
#ifdef CONFIG_SAMV7_UART0
static int  sam_uart0_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMV7_UART1
static int  sam_uart1_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMV7_UART2
static int  sam_uart2_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMV7_UART3
static int  sam_uart3_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMV7_UART4
static int  sam_uart4_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMV7_USART0
static int  sam_usart0_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMV7_USART1
static int  sam_usart1_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMV7_USART2
static int  sam_usart2_interrupt(int irq, void *context);
#endif
static int  sam_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  sam_receive(struct uart_dev_s *dev, uint32_t *status);
static void sam_rxint(struct uart_dev_s *dev, bool enable);
static bool sam_rxavailable(struct uart_dev_s *dev);
static void sam_send(struct uart_dev_s *dev, int ch);
static void sam_txint(struct uart_dev_s *dev, bool enable);
static bool sam_txready(struct uart_dev_s *dev);
static bool sam_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = sam_setup,
  .shutdown       = sam_shutdown,
  .attach         = sam_attach,
  .detach         = sam_detach,
  .ioctl          = sam_ioctl,
  .receive        = sam_receive,
  .rxint          = sam_rxint,
  .rxavailable    = sam_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = sam_send,
  .txint          = sam_txint,
  .txready        = sam_txready,
  .txempty        = sam_txempty,
};

/* I/O buffers */

#ifdef CONFIG_SAMV7_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMV7_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMV7_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMV7_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMV7_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMV7_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMV7_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMV7_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif

/* This describes the state of the UART0 port. */

#ifdef CONFIG_SAMV7_UART0
static struct sam_dev_s g_uart0priv =
{
  .usartbase      = SAM_UART0_BASE,
  .handler        = sam_uart0_interrupt,
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

#ifdef CONFIG_SAMV7_UART1
static struct sam_dev_s g_uart1priv =
{
  .usartbase      = SAM_UART1_BASE,
  .handler        = sam_uart1_interrupt,
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

/* This describes the state of the UART2 port. */

#ifdef CONFIG_SAMV7_UART2
static struct sam_dev_s g_uart2priv =
{
  .usartbase      = SAM_UART2_BASE,
  .handler        = sam_uart2_interrupt,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = SAM_IRQ_UART2,
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

/* This describes the state of the UART3 port. */

#ifdef CONFIG_SAMV7_UART3
static struct sam_dev_s g_uart3priv =
{
  .usartbase      = SAM_UART3_BASE,
  .handler        = sam_uart3_interrupt,
  .baud           = CONFIG_UART3_BAUD,
  .irq            = SAM_IRQ_UART3,
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

/* This describes the state of the UART4 port. */

#ifdef CONFIG_SAMV7_UART4
static struct sam_dev_s g_uart4priv =
{
  .usartbase      = SAM_UART4_BASE,
  .handler        = sam_uart4_interrupt,
  .baud           = CONFIG_UART4_BAUD,
  .irq            = SAM_IRQ_UART4,
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

/* This describes the state of the USART0 port. */

#ifdef CONFIG_SAMV7_USART0
static struct sam_dev_s g_usart0priv =
{
  .usartbase      = SAM_USART0_BASE,
  .handler        = sam_usart0_interrupt,
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

#ifdef CONFIG_SAMV7_USART1
static struct sam_dev_s g_usart1priv =
{
  .usartbase      = SAM_USART1_BASE,
  .handler        = sam_usart1_interrupt,
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

#ifdef CONFIG_SAMV7_USART2
static struct sam_dev_s g_usart2priv =
{
  .usartbase      = SAM_USART2_BASE,
  .handler        = sam_usart2_interrupt,
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
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_usart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_usart2txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_serialin
 ****************************************************************************/

static inline uint32_t sam_serialin(struct sam_dev_s *priv, int offset)
{
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: sam_serialout
 ****************************************************************************/

static inline void sam_serialout(struct sam_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: sam_restoreusartint
 ****************************************************************************/

static inline void sam_restoreusartint(struct sam_dev_s *priv, uint32_t imr)
{
  /* Restore the previous interrupt state (assuming all interrupts disabled) */

  sam_serialout(priv, SAM_UART_IER_OFFSET, imr);
}

/****************************************************************************
 * Name: sam_disableallints
 ****************************************************************************/

static void sam_disableallints(struct sam_dev_s *priv, uint32_t *imr)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = irqsave();
  if (imr)
    {
      /* Return the current interrupt mask */

      *imr = sam_serialin(priv, SAM_UART_IMR_OFFSET);
    }

  /* Disable all interrupts */

  sam_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_ALLINTS);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int sam_setup(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t divb3;
  uint32_t intpart;
  uint32_t fracpart;
  uint32_t regval;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled and the pins were configured in sam_lowsetup().
   */

  /* The shutdown method will put the UART in a known, disabled state */

  sam_shutdown(dev);

  /* Set up the mode register.  Start with normal UART mode and the MCK
   * as the timing source
   */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  /* "Setting the USART to operate with hardware handshaking is performed by
   *  writing the USART_MODE field in the Mode Register (US_MR) to the value
   *  0x2. ... Using this mode requires using the PDC or DMAC channel for
   *  reception. The transmitter can handle hardware handshaking in any case."
   */

  if (priv->flowc)
    {
      /* Enable hardware flow control and MCK as the timing source
       * (the divided clock source may be reselected below).
       */

      regval = (UART_MR_MODE_HWHS | UART_MR_USCLKS_MCK);
    }
  else
#endif
    {
      /* Set up the mode register.  Start with normal UART mode and the MCK
       * as the timing source (the divided clock source may be reselected
       * below).
       */

      regval = (UART_MR_MODE_NORMAL | UART_MR_USCLKS_MCK);
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
#ifdef HAVE_UART_DEVICE
  else if (priv->bits == 9
#if defined(CONFIG_SAMV7_UART0)
           && priv->usartbase != SAM_UART0_BASE
#endif
#if defined(CONFIG_SAMV7_UART1)
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

  /* And save the new mode register value */

  sam_serialout(priv, SAM_UART_MR_OFFSET, regval);

  /* Configure the console baud:
   *
   *   Fbaud   = USART_CLOCK / (16 * divisor)
   *   divisor = USART_CLOCK / (16 * Fbaud)
   *
   * NOTE: Oversampling by 8 is not supported. This may limit BAUD rates
   * for lower USART clocks.
   */

  divb3    = ((FAST_USART_CLOCK + (priv->baud << 3)) << 3) /
             (priv->baud << 4);
  intpart  = divb3 >> 3;
  fracpart = divb3 & 7;

  /* Retain the fast MR peripheral clock UNLESS unless using that clock
   * would result in an excessively large divider.
   *
   * REVISIT: The fractional divider is not used.
   */

  if ((intpart & ~UART_BRGR_CD_MASK) != 0)
    {
      /* Use the divided USART clock */

      divb3    = ((SLOW_USART_CLOCK + (priv->baud << 3)) << 3) /
                 (priv->baud << 4);
      intpart  = divb3 >> 3;
      fracpart = divb3 & 7;

      /* Re-select the clock source */

      regval  = sam_serialin(priv, SAM_UART_MR_OFFSET);
      regval &= ~UART_MR_USCLKS_MASK;
      regval |= UART_MR_USCLKS_MCKDIV;
      sam_serialout(priv, SAM_UART_MR_OFFSET, regval);
    }

  /* Save the BAUD divider (the fractional part is not used for UARTs) */

  regval = UART_BRGR_CD(intpart) | UART_BRGR_FP(fracpart);
  sam_serialout(priv, SAM_UART_BRGR_OFFSET, regval);

  /* Enable receiver & transmitter */

  sam_serialout(priv, SAM_UART_CR_OFFSET, (UART_CR_RXEN|UART_CR_TXEN));
#endif

  return OK;
}

/****************************************************************************
 * Name: sam_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void sam_shutdown(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;

  /* Reset and disable receiver and transmitter */

  sam_serialout(priv, SAM_UART_CR_OFFSET,
                (UART_CR_RSTRX|UART_CR_RSTTX|UART_CR_RXDIS|UART_CR_TXDIS));

  /* Disable all interrupts */

  sam_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: sam_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int sam_attach(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, priv->handler);
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
 * Name: sam_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void sam_detach(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: sam_interrupt
 *
 * Description:
 *   This is the common UART/USART interrupt handler.  It will be invoked
 *   when an interrupt received on the device.  It should call
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ****************************************************************************/

static int sam_interrupt(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv;
  uint32_t          pending;
  uint32_t          imr;
  int               passes;
  bool              handled;

  DEBUGASSERT(dev && dev->priv);
  priv = (struct sam_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or, until we have
   * been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the UART/USART status (we are only interested in the unmasked interrupts). */

      priv->sr = sam_serialin(priv, SAM_UART_SR_OFFSET);
      imr      = sam_serialin(priv, SAM_UART_IMR_OFFSET);
      pending  = priv->sr & imr;

      /* Handle an incoming, receive byte.  RXRDY: At least one complete character
       * has been received and US_RHR has not yet been read.
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
 * Name: sam_uart[n]_interrupt
 *
 * Description:
 *   UART interrupt handlers
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_UART0
static int  sam_uart0_interrupt(int irq, void *context)
{
  return sam_interrupt(&g_uart0port);
}
#endif
#ifdef CONFIG_SAMV7_UART1
static int  sam_uart1_interrupt(int irq, void *context)
{
  return sam_interrupt(&g_uart1port);
}
#endif
#ifdef CONFIG_SAMV7_UART2
static int  sam_uart2_interrupt(int irq, void *context)
{
  return sam_interrupt(&g_uart2port);
}
#endif
#ifdef CONFIG_SAMV7_UART3
static int  sam_uart3_interrupt(int irq, void *context)
{
  return sam_interrupt(&g_uart3port);
}
#endif
#ifdef CONFIG_SAMV7_UART4
static int  sam_uart4_interrupt(int irq, void *context)
{
  return sam_interrupt(&g_uart4port);
}
#endif

/****************************************************************************
 * Name: sam_usart[n]_interrupt
 *
 * Description:
 *   USART interrupt handlers
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USART0
static int  sam_usart0_interrupt(int irq, void *context)
{
  return sam_interrupt(&g_usart0port);
}
#endif
#ifdef CONFIG_SAMV7_USART1
static int  sam_usart1_interrupt(int irq, void *context)
{
  return sam_interrupt(&g_usart1port);
}
#endif
#ifdef CONFIG_SAMV7_USART2
static int  sam_usart2_interrupt(int irq, void *context)
{
  return sam_interrupt(&g_usart2port);
}
#endif

/****************************************************************************
 * Name: sam_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int sam_ioctl(struct file *filep, int cmd, unsigned long arg)
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
         struct sam_dev_s *user = (struct sam_dev_s*)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct sam_dev_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios*)arg;
        struct sam_dev_s *priv     = (struct sam_dev_s *)dev->priv;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return baud */

        cfsetispeed(termiosp, priv->baud);

        /* Return parity */

        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= (priv->stopbits2) ? CSTOPB : 0;

        /* Return flow control */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        termiosp->c_cflag |= (priv->flowc) ? (CCTS_OFLOW | CRTS_IFLOW): 0;
#endif
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
        struct termios  *termiosp = (struct termios*)arg;
        struct sam_dev_s *priv     = (struct sam_dev_s *)dev->priv;
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

            sam_disableallints(priv, &imr);
            ret = sam_setup(dev);

            /* Restore the interrupt state */

            sam_restoreusartint(priv, imr);
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
 * Name: sam_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int sam_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;

  /* Return the error information in the saved status */

  *status  = priv->sr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return (int)(sam_serialin(priv, SAM_UART_RHR_OFFSET) & 0xff);
}

/****************************************************************************
 * Name: sam_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void sam_rxint(struct uart_dev_s *dev, bool enable)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      sam_serialout(priv, SAM_UART_IER_OFFSET, UART_INT_RXRDY);
#endif
    }
  else
    {
      sam_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_RXRDY);
    }
}

/****************************************************************************
 * Name: sam_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool sam_rxavailable(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;
  return ((sam_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_RXRDY) != 0);
}

/****************************************************************************
 * Name: sam_send
 *
 * Description:
 *   This method will send one byte on the UART/USART
 *
 ****************************************************************************/

static void sam_send(struct uart_dev_s *dev, int ch)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;
  sam_serialout(priv, SAM_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: sam_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void sam_txint(struct uart_dev_s *dev, bool enable)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      sam_serialout(priv, SAM_UART_IER_OFFSET, UART_INT_TXRDY);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);

#endif
    }
  else
    {
      /* Disable the TX interrupt */

      sam_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_TXRDY);
    }

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool sam_txready(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;
  return ((sam_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_TXRDY) != 0);
 }

/****************************************************************************
 * Name: sam_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool sam_txempty(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev->priv;
  return ((sam_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_TXEMPTY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the USARTs was performed in
   * sam_lowsetup
   */

  /* Disable all USARTS */

  sam_disableallints(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  sam_disableallints(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  sam_disableallints(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  sam_disableallints(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  sam_disableallints(TTYS4_DEV.priv, NULL);
#endif
#ifdef TTYS5_DEV
  sam_disableallints(TTYS5_DEV.priv, NULL);
#endif
#ifdef TTYS6_DEV
  sam_disableallints(TTYS6_DEV.priv, NULL);
#endif
#ifdef TTYS7_DEV
  sam_disableallints(TTYS7_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  sam_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */

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
#ifdef TTYS6_DEV
  (void)uart_register("/dev/ttyS6", &TTYS6_DEV);
#endif
#ifdef TTYS7_DEV
  (void)uart_register("/dev/ttyS7", &TTYS7_DEV);
#endif
}

#endif /* USE_SERIALDRIVER */
