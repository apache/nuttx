/****************************************************************************
 * arch/arm/src/sama5/sam_serial.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "chip/sam_uart.h"
#include "sam_dbgu.h"
#include "sam_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* If the USART is not being used as a UART, then it really isn't enabled
 * for our purposes.
 */

#ifndef CONFIG_USART0_ISUART
#  undef CONFIG_SAMA5_USART0
#endif
#ifndef CONFIG_USART1_ISUART
#  undef CONFIG_SAMA5_USART1
#endif
#ifndef CONFIG_USART2_ISUART
#  undef CONFIG_SAMA5_USART2
#endif
#ifndef CONFIG_USART3_ISUART
#  undef CONFIG_SAMA5_USART3
#endif
#ifndef CONFIG_USART4_ISUART
#  undef CONFIG_SAMA5_USART4
#endif

/* Is there a USART/USART enabled? */

#if defined(CONFIG_SAMA5_UART0) || defined(CONFIG_SAMA5_UART1)
#  define HAVE_UART
#endif

#if defined(CONFIG_SAMA5_USART0) || defined(CONFIG_SAMA5_USART1) || \
    defined(CONFIG_SAMA5_USART2) || defined(CONFIG_SAMA5_USART3) || \
    defined(CONFIG_SAMA5_USART4)
#  define HAVE_USART
#endif

/* Is there a serial console?  It could be on UART0-1 or USART0-3 */

#if defined(CONFIG_SAMA5_DBGU_CONSOLE) && defined(CONFIG_SAMA5_DBGU)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef HAVE_UART_CONSOLE
#elif defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_UART0)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_UART1)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_USART0)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_USART1)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_USART2)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_SAMA5_USART3)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_USART4_SERIAL_CONSOLE) && defined(CONFIG_SAMA4_USART4)
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#else
#  warning "No valid CONFIG_USARTn_SERIAL_CONSOLE Setting"
#  undef CONFIG_SAMA5_DBGU_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef HAVE_UART_CONSOLE
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

#undef TTYS1_DEV
#undef TTYS2_DEV
#undef TTYS3_DEV
#undef TTYS4_DEV
#undef TTYS5_DEV
#undef TTYS6_DEV

#if defined(HAVE_UART) || defined(HAVE_USART)

/* Which UART/USART with be tty0/console and which tty1? tty2? tty3? tty4? tty5? tty6? */

/* First pick the console and ttyS0.  This could be any of UART0-1, USART0-4 */

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
#    define TTYS0_DEV           g_usart3port /* USART3 is ttyS0 */
#    define USART3_ASSIGNED     1
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart4port /* USART4 is console */
#    define TTYS0_DEV           g_usart4port /* USART4 is ttyS0 */
#    define USART4_ASSIGNED     1
#else
#  undef CONSOLE_DEV                         /* No console */
#  if defined(CONFIG_SAMA5_UART0)
#    define TTYS0_DEV           g_uart0port  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_SAMA5_UART1)
#    define TTYS0_DEV           g_uart1port  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_SAMA5_USART0)
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#  elif defined(CONFIG_SAMA5_USART1)
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED     1
#  elif defined(CONFIG_SAMA5_USART2)
#    define TTYS0_DEV           g_usart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#  elif defined(CONFIG_SAMA5_USART3)
#    define TTYS0_DEV           g_usart3port /* USART3 is ttyS0 */
#    define USART3_ASSIGNED     1
#  elif defined(CONFIG_SAMA5_USART4)
#    define TTYS0_DEV           g_usart4port /* USART4 is ttyS0 */
#    define USART4_ASSIGNED     4
#  endif
#endif

/* Pick ttyS1.  This could be any of UART0-1, USART0-4 excluding the console UART. */

#if defined(CONFIG_SAMA5_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port  /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_SAMA5_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port  /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_SAMA5_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV           g_usart0port /* USART0 is ttyS1 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS1_DEV           g_usart1port /* USART1 is ttyS1 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS1_DEV           g_usart2port /* USART2 is ttyS1 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS1_DEV           g_usart3port /* USART3 is ttyS1 */
#  define USART3_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS1_DEV           g_usart4port /* USART4 is ttyS1 */
#  define USART4_ASSIGNED     1
#endif

/* Pick ttys2.  This could be one of UART1 or USART0-4. It can't be UART0
 * because that was either assigned as ttyS0 or ttyS1.  One of these
 * could also be the console.
 */

#if defined(CONFIG_SAMA5_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port  /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_SAMA5_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS2_DEV           g_usart0port /* USART0 is ttyS2 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS2_DEV           g_usart1port /* USART1 is ttyS2 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS2_DEV           g_usart2port /* USART2 is ttyS2 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS2_DEV           g_usart3port /* USART3 is ttyS2 */
#  define USART3_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS2_DEV           g_usart4port /* USART4 is ttyS2 */
#  define USART4_ASSIGNED     1
#endif

/* Pick ttys3. This could be one of USART0-4. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * USART0-4 could also be the console.
 */

#if defined(CONFIG_SAMA5_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS3_DEV           g_usart0port /* USART0 is ttyS3 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS3_DEV           g_usart1port /* USART1 is ttyS3 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS3_DEV           g_usart2port /* USART2 is ttyS3 */
#  define USART2_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS3_DEV           g_usart3port /* USART3 is ttyS3 */
#  define USART3_ASSIGNED     1
#elif defined(CONFIG_SAMA5_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS3_DEV           g_usart4port /* USART4 is ttyS3 */
#  define USART4_ASSIGNED     1
#endif

/* Pick ttyS4. This could be one of USART1-4. It can't be UART0-1 or USART0
 * because those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * USART1-4 could also be the console.
 */

#if defined(CONFIG_SAMA5_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS4_DEV           g_usart1port /* USART1 is ttyS4 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMA5_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS4_DEV           g_usart2port /* USART2 is ttyS4 */
#  define USART2_ASSIGNED      1
#elif defined(CONFIG_SAMA5_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS4_DEV           g_usart3port /* USART3 is ttyS4 */
#  define USART3_ASSIGNED      1
#elif defined(CONFIG_SAMA5_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS4_DEV           g_usart4port /* USART4 is ttyS4 */
#  define USART4_ASSIGNED      1
#endif

/* Pick ttyS5. This could be one of USART2-4. It can't be UART0-1 or
 * USART0-1 because those have already been assigned to ttsyS0, 1, 2,
 * 3 or 4.  One of USART2-4 could also be the console.
 */

#if defined(CONFIG_SAMA5_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS5_DEV           g_usart2port /* USART2 is ttyS5 */
#  define USART2_ASSIGNED      1
#elif defined(CONFIG_SAMA5_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS5_DEV           g_usart3port /* USART3 is ttyS5 */
#  define USART3_ASSIGNED      1
#elif defined(CONFIG_SAMA5_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS5_DEV           g_usart3port /* USART4 is ttyS5 */
#  define USART4_ASSIGNED      1
#endif

/* Pick ttyS6. This could be one of USART3-4. It can't be UART0-1 or
 * USART0-2 because those have already been assigned to ttsyS0, 1, 2,
 * 3, 4, or 5.  One of USART3-4 could also be the console.
 */

#if defined(CONFIG_SAMA5_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS6_DEV           g_usart3port /* USART3 is ttyS6 */
#  define USART3_ASSIGNED      1
#elif defined(CONFIG_SAMA5_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS6_DEV           g_usart3port /* USART4 is ttyS6 */
#  define USART4_ASSIGNED      1
#endif

/* The UART/USART modules are driven by the peripheral clock (MCK or MCK2). */

#define SAM_USART_CLOCK  BOARD_USART_FREQUENCY /* Frequency of the USART clock */
#define SAM_MR_USCLKS    UART_MR_USCLKS_MCK    /* Source = Main clock */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t usartbase; /* Base address of USART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t sr;        /* Saved status bits */
  uint8_t  irq;       /* IRQ associated with this USART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  bool     flowc;     /* input flow control (RTS) enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
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

#ifdef CONFIG_SAMA5_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMA5_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMA5_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMA5_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMA5_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMA5_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMA5_USART4
static char g_usart4rxbuffer[CONFIG_USART4_RXBUFSIZE];
static char g_usart4txbuffer[CONFIG_USART4_TXBUFSIZE];
#endif

/* This describes the state of the UART0 port. */

#ifdef CONFIG_SAMA5_UART0
static struct up_dev_s g_uart0priv =
{
  .usartbase      = SAM_UART0_VBASE,
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

#ifdef CONFIG_SAMA5_UART1
static struct up_dev_s g_uart1priv =
{
  .usartbase      = SAM_UART1_VBASE,
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

#ifdef CONFIG_SAMA5_USART0
static struct up_dev_s g_usart0priv =
{
  .usartbase      = SAM_USART0_VBASE,
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

#ifdef CONFIG_SAMA5_USART1
static struct up_dev_s g_usart1priv =
{
  .usartbase      = SAM_USART1_VBASE,
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

#ifdef CONFIG_SAMA5_USART2
static struct up_dev_s g_usart2priv =
{
  .usartbase      = SAM_USART2_VBASE,
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

/* This describes the state of the USART3 port. */

#ifdef CONFIG_SAMA5_USART3
static struct up_dev_s g_usart3priv =
{
  .usartbase      = SAM_USART3_VBASE,
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
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_usart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_usart3txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart3priv,
};
#endif

/* This describes the state of the USART4 port. */

#ifdef CONFIG_SAMA5_USART4
static struct up_dev_s g_usart4priv =
{
  .usartbase      = SAM_USART4_VBASE,
  .baud           = CONFIG_USART4_BAUD,
  .irq            = SAM_IRQ_USART4,
  .parity         = CONFIG_USART4_PARITY,
  .bits           = CONFIG_USART4_BITS,
  .stopbits2      = CONFIG_USART4_2STOP,
#if defined(CONFIG_USART4_OFLOWCONTROL) || defined(CONFIG_USART4_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_usart4port =
{
  .recv     =
  {
    .size   = CONFIG_USART4_RXBUFSIZE,
    .buffer = g_usart4rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART4_TXBUFSIZE,
    .buffer = g_usart4txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart4priv,
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

static inline void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static inline void up_restoreusartint(struct up_dev_s *priv, uint32_t imr)
{
  /* Restore the previous interrupt state (assuming all interrupts disabled) */

  up_serialout(priv, SAM_UART_IER_OFFSET, imr);
}

/****************************************************************************
 * Name: up_disableallints
 ****************************************************************************/

static void up_disableallints(struct up_dev_s *priv, uint32_t *imr)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = irqsave();

  /* Return the current interrupt state */

  if (imr)
    {
      *imr = up_serialin(priv, SAM_UART_IMR_OFFSET);
    }

  /* Disable all interrupts */

  up_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_ALLINTS);
  irqrestore(flags);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t regval;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled and the pins were configured in sam_lowsetup().
   */

  /* The shutdown method will put the UART in a known, disabled state */

  up_shutdown(dev);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  /* "Setting the USART to operate with hardware handshaking is performed by
   *  writing the USART_MODE field in the Mode Register (US_MR) to the value
   *  0x2. ... Using this mode requires using the PDC or DMAC channel for
   *  reception. The transmitter can handle hardware handshaking in any case."
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
#if defined(CONFIG_SAMA5_UART0)
           && priv->usartbase != SAM_UART0_VBASE
#endif
#if defined(CONFIG_SAMA5_UART1)
           && priv->usartbase != SAM_UART1_VBASE
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

  up_serialout(priv, SAM_UART_MR_OFFSET, regval);

  /* Configure the console baud.  NOTE: Oversampling by 8 is not supported.
   * This may limit BAUD rates for lower USART clocks.
   */

  regval  = (SAM_USART_CLOCK + (priv->baud << 3))/(priv->baud << 4);
  up_serialout(priv, SAM_UART_BRGR_OFFSET, regval);

  /* Enable receiver & transmitter */

  up_serialout(priv, SAM_UART_CR_OFFSET, (UART_CR_RXEN|UART_CR_TXEN));
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Reset and disable receiver and transmitter */

  up_serialout(priv, SAM_UART_CR_OFFSET,
               (UART_CR_RSTRX|UART_CR_RSTTX|UART_CR_RXDIS|UART_CR_TXDIS));

  /* Disable all interrupts */

  up_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
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

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
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
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
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

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  uint32_t           pending;
  uint32_t           imr;
  int                passes;
  bool               handled;

#ifdef CONFIG_SAMA5_UART0
  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_SAMA5_UART1
  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else
#endif
#ifdef CONFIG_SAMA5_USART0
  if (g_usart0priv.irq == irq)
    {
      dev = &g_usart0port;
    }
  else
#endif
#ifdef CONFIG_SAMA5_USART1
  if (g_usart1priv.irq == irq)
    {
      dev = &g_usart1port;
    }
  else
#endif
#ifdef CONFIG_SAMA5_USART2
  if (g_usart2priv.irq == irq)
    {
      dev = &g_usart2port;
    }
  else
#endif
#ifdef CONFIG_SAMA5_USART3
  if (g_usart3priv.irq == irq)
    {
      dev = &g_usart3port;
    }
  else
#endif
#ifdef CONFIG_SAMA5_USART4
  if (g_usart4priv.irq == irq)
    {
      dev = &g_usart4port;
    }
  else
#endif
    {
      PANIC();
    }

  priv = (struct up_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or, until we have
   * been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the UART/USART status (we are only interested in the unmasked interrupts). */

      priv->sr = up_serialin(priv, SAM_UART_SR_OFFSET);  /* Save for error reporting */
      imr      = up_serialin(priv, SAM_UART_IMR_OFFSET); /* Interrupt mask */
      pending  = priv->sr & imr;                         /* Mask out disabled interrupt sources */

      /* Handle an incoming, receive byte.  RXRDY: At least one complete character
       * has been received and US_RHR has not yet been read.
       */

      if ((pending & UART_INT_RXRDY) != 0)
        {
           /* Received data ready... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes. XRDY: There is no character in the
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
         struct up_dev_s *user = (struct up_dev_s*)arg;
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
        struct termios  *termiosp = (struct termios*)arg;
        struct up_dev_s *priv     = (struct up_dev_s *)dev->priv;

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
            /* Effect the changes immediately - note that we do not
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

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
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

  irqrestore(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_TXEMPTY) != 0);
}

#endif /* HAVE_UART || HAVE_USART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

void sam_earlyserialinit(void)
{
  /* NOTE:  All PIO configuration for the USARTs was performed in
   * sam_lowsetup
   */

  /* Disable all USARTS */

#ifdef TTYS0_DEV
  up_disableallints(TTYS0_DEV.priv, NULL);
#endif
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
#ifdef TTYS6_DEV
  up_disableallints(TTYS6_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

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

#ifdef HAVE_UART_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs/USARTs */

#ifdef TTYS0_DEV
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
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

/* Register the DBGU as well */

#ifdef CONFIG_SAMA5_DBGU
  sam_dbgu_register();
#endif
}

#endif /* USE_SERIALDRIVER */
