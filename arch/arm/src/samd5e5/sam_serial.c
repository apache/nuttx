/****************************************************************************
 * arch/arm/src/samd5e5/sam_serial.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <arch/serial.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "sam_config.h"
#include "sam_usart.h"
#include "sam_lowputc.h"
#include "sam_serial.h"

#ifdef SAMD5E5_HAVE_USART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which USART with be tty0/console and which tty1? tty2? tty3? ...tty7? */

/* First pick the console and ttys0.  This could be any of USART0-5 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart0port /* USART0 is console */
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED      1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart1port /* USART1 is console */
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED      1
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
#elif defined(CONFIG_USART5_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart5port /* USART5 is console */
#    define TTYS0_DEV           g_usart5port /* USART5 is ttyS0 */
#    define USART5_ASSIGNED     1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart6port /* USART6 is console */
#    define TTYS0_DEV           g_usart6port /* USART6 is ttyS0 */
#    define USART6_ASSIGNED     1
#elif defined(CONFIG_USART7_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart7port /* USART7 is console */
#    define TTYS0_DEV           g_usart7port /* USART7 is ttyS0 */
#    define USART7_ASSIGNED     1
#else
#  undef CONSOLE_DEV                         /* No console */
#  if defined(SAMD5E5_HAVE_USART0)
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED      1
#  elif defined(SAMD5E5_HAVE_USART1)
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED      1
#  elif defined(SAMD5E5_HAVE_USART2)
#    define TTYS0_DEV           g_usart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#  elif defined(SAMD5E5_HAVE_USART3)
#    define TTYS0_DEV           g_usart3port /* USART3 is ttyS0 */
#    define USART3_ASSIGNED     1
#  elif defined(SAMD5E5_HAVE_USART4)
#    define TTYS0_DEV           g_usart4port /* USART4 is ttyS0 */
#    define USART4_ASSIGNED     1
#  elif defined(SAMD5E5_HAVE_USART5)
#    define TTYS0_DEV           g_usart5port /* USART5 is ttyS0 */
#    define USART5_ASSIGNED     1
#  elif defined(SAMD5E5_HAVE_USART6)
#    define TTYS0_DEV           g_usart6port /* USART6 is ttyS0 */
#    define USART5_ASSIGNED     1
#  elif defined(SAMD5E5_HAVE_USART7)
#    define TTYS0_DEV           g_usart7port /* USART7 is ttyS0 */
#    define USART5_ASSIGNED     1
#  endif
#endif

/* Pick ttys1.  This could be any of USART0-7 excluding the console USART. */

#if defined(SAMD5E5_HAVE_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV           g_usart0port /* USART0 is ttyS1 */
#  define USART0_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS1_DEV           g_usart1port /* USART1 is ttyS1 */
#  define USART1_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS1_DEV           g_usart2port /* USART2 is ttyS1 */
#  define USART2_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS1_DEV           g_usart3port /* USART3 is ttyS1 */
#  define USART3_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS1_DEV           g_usart4port /* USART4 is ttyS1 */
#  define USART4_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS1_DEV           g_usart5port /* USART5 is ttyS1 */
#  define USART5_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS1_DEV           g_usart6port /* USART6 is ttyS1 */
#  define USART6_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS1_DEV           g_usart7port /* USART7 is ttyS1 */
#  define USART7_ASSIGNED     1
#endif

/* Pick ttys2.  This could be one of USART1-7. It can't be USART0
 * because that was either assigned as ttyS0 or ttys1.  One of these
 * could also be the console.
 */

#if defined(SAMD5E5_HAVE_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS2_DEV           g_usart1port /* USART1 is ttyS2 */
#  define USART1_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS2_DEV           g_usart2port /* USART2 is ttyS2 */
#  define USART2_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS2_DEV           g_usart3port /* USART3 is ttyS2 */
#  define USART3_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS2_DEV           g_usart4port /* USART4 is ttyS2 */
#  define USART4_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS2_DEV           g_usart5port /* USART5 is ttyS2 */
#  define USART5_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS2_DEV           g_usart6port /* USART6 is ttyS2 */
#  define USART6_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS2_DEV           g_usart7port /* USART7 is ttyS2 */
#  define USART7_ASSIGNED     1
#endif

/* Pick ttys3. This could be one of USART2-7. It can't be USART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * USART2-5 could also be the console.
 */

#if defined(SAMD5E5_HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS3_DEV           g_usart2port /* USART2 is ttyS3 */
#  define USART2_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS3_DEV           g_usart3port /* USART3 is ttyS3 */
#  define USART3_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS3_DEV           g_usart4port /* USART4 is ttyS3 */
#  define USART4_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS3_DEV           g_usart5port /* USART5 is ttyS3 */
#  define USART5_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS3_DEV           g_usart6port /* USART6 is ttyS3 */
#  define USART6_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS3_DEV           g_usart7port /* USART7 is ttyS3 */
#  define USART7_ASSIGNED     1
#endif

/* Pick ttys4. This could be one of USART3-7. It can't be USART0-2
 * because those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * USART3-5 could also be the console.
 */

#if defined(SAMD5E5_HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS4_DEV           g_usart3port /* USART3 is ttyS4 */
#  define USART3_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS4_DEV           g_usart4port /* USART4 is ttyS4 */
#  define USART4_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS4_DEV           g_usart5port /* USART5 is ttyS4 */
#  define USART5_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS4_DEV           g_usart6port /* USART6 is ttyS4 */
#  define USART6_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS4_DEV           g_usart7port /* USART7 is ttyS4 */
#  define USART7_ASSIGNED     1
#endif

/* Pick ttys5. This could be one of USART4-7. It can't be USART0-3
 * because those have already been assigned to ttsyS0, 1, 2, 3 or 4.
 * One of USART4-5 could also be the console.
 */

#if defined(SAMD5E5_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS5_DEV           g_usart4port /* USART4 is ttyS5 */
#  define USART4_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS5_DEV           g_usart5port /* USART5 is ttyS5 */
#  define USART5_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS5_DEV           g_usart6port /* USART6 is ttyS5 */
#  define USART6_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS5_DEV           g_usart7port /* USART7 is ttyS5 */
#  define USART7_ASSIGNED     1
#endif

/* Pick ttys6. This could be one of USART5-7. It can't be USART0-4
 * because those have already been assigned to ttsyS0, 1, 2, 3, 4 or 5.
 * One of USART4-5 could also be the console.
 */

#if defined(SAMD5E5_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS6_DEV           g_usart5port /* USART5 is ttyS6 */
#  define USART5_ASSIGNED      1
#elif defined(SAMD5E5_HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS6_DEV           g_usart6port /* USART6 is ttyS6 */
#  define USART6_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS6_DEV           g_usart7port /* USART7 is ttyS6 */
#  define USART7_ASSIGNED     1
#endif

/* Pick ttys7. This could be one of USART6-7. It can't be USART0-5
 * because those have already been assigned to ttsyS0, 1, 2, 3, 4, 5, or 6.
 * One of USART4-5 could also be the console.
 */

#if defined(SAMD5E5_HAVE_USART6) && !defined(USART6_ASSIGNED)
#  define TTYS7_DEV           g_usart6port /* USART6 is ttyS7 */
#  define USART6_ASSIGNED     1
#elif defined(SAMD5E5_HAVE_USART7) && !defined(USART7_ASSIGNED)
#  define TTYS7_DEV           g_usart7port /* USART7 is ttyS7 */
#  define USART7_ASSIGNED     1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_dev_s
{
  /* Common USART configuration */

  const struct sam_usart_config_s * const config;

  /* Information unique to the serial driver */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Support functions */

static inline uint8_t
            sam_serialin8(struct sam_dev_s *priv, int offset);
static inline void
            sam_serialout8(struct sam_dev_s *priv, int offset,
              uint8_t regval);
static inline uint16_t
            sam_serialin16(struct sam_dev_s *priv, int offset);
static inline void
            sam_serialout16(struct sam_dev_s *priv, int offset,
              uint16_t regval);
static void sam_disableallints(struct sam_dev_s *priv);
static int  sam_interrupt(int irq, void *context, FAR void *arg);

/* UART methods */

static int  sam_setup(struct uart_dev_s *dev);
static void sam_shutdown(struct uart_dev_s *dev);
static int  sam_attach(struct uart_dev_s *dev);
static void sam_detach(struct uart_dev_s *dev);
static int  sam_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  sam_receive(struct uart_dev_s *dev, uint32_t *status);
static void sam_rxint(struct uart_dev_s *dev, bool enable);
static bool sam_rxavailable(struct uart_dev_s *dev);
static void sam_send(struct uart_dev_s *dev, int ch);
static void sam_txint(struct uart_dev_s *dev, bool enable);
static bool sam_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
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
  .txready        = sam_txempty,
  .txempty        = sam_txempty,
};

/* I/O buffers */

#ifdef SAMD5E5_HAVE_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef SAMD5E5_HAVE_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef SAMD5E5_HAVE_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef SAMD5E5_HAVE_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif
#ifdef SAMD5E5_HAVE_USART4
static char g_usart4rxbuffer[CONFIG_USART4_RXBUFSIZE];
static char g_usart4txbuffer[CONFIG_USART4_TXBUFSIZE];
#endif
#ifdef SAMD5E5_HAVE_USART5
static char g_usart5rxbuffer[CONFIG_USART5_RXBUFSIZE];
static char g_usart5txbuffer[CONFIG_USART5_TXBUFSIZE];
#endif
#ifdef SAMD5E5_HAVE_USART6
static char g_usart6rxbuffer[CONFIG_USART6_RXBUFSIZE];
static char g_usart6txbuffer[CONFIG_USART6_TXBUFSIZE];
#endif
#ifdef SAMD5E5_HAVE_USART7
static char g_usart7rxbuffer[CONFIG_USART7_RXBUFSIZE];
static char g_usart7txbuffer[CONFIG_USART7_TXBUFSIZE];
#endif

/* This describes the state of the USART0 port. */

#ifdef SAMD5E5_HAVE_USART0
static struct sam_dev_s g_usart0priv =
{
  .config   = &g_usart0config,
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

#ifdef SAMD5E5_HAVE_USART1
static struct sam_dev_s g_usart1priv =
{
  .config   = &g_usart1config,
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

#ifdef SAMD5E5_HAVE_USART2
static struct sam_dev_s g_usart2priv =
{
  .config   = &g_usart2config,
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

#ifdef SAMD5E5_HAVE_USART3
static struct sam_dev_s g_usart3priv =
{
  .config   = &g_usart3config,
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

#ifdef SAMD5E5_HAVE_USART4
static struct sam_dev_s g_usart4priv =
{
  .config   = &g_usart4config,
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

/* This describes the state of the USART5 port. */

#ifdef SAMD5E5_HAVE_USART5
static struct sam_dev_s g_usart5priv =
{
  .config   = &g_usart5config,
};

static uart_dev_t g_usart5port =
{
  .recv     =
  {
    .size   = CONFIG_USART5_RXBUFSIZE,
    .buffer = g_usart5rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART5_TXBUFSIZE,
    .buffer = g_usart5txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart5priv,
};
#endif

/* This describes the state of the USART6 port. */

#ifdef SAMD5E5_HAVE_USART6
static struct sam_dev_s g_usart6priv =
{
  .config   = &g_usart6config,
};

static uart_dev_t g_usart6port =
{
  .recv     =
  {
    .size   = CONFIG_USART6_RXBUFSIZE,
    .buffer = g_usart6rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART6_TXBUFSIZE,
    .buffer = g_usart6txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart6priv,
};
#endif

/* This describes the state of the USART7 port. */

#ifdef SAMD5E5_HAVE_USART7
static struct sam_dev_s g_usart7priv =
{
  .config   = &g_usart7config,
};

static uart_dev_t g_usart7port =
{
  .recv     =
  {
    .size   = CONFIG_USART7_RXBUFSIZE,
    .buffer = g_usart7rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART7_TXBUFSIZE,
    .buffer = g_usart7txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart7priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_serialin8
 ****************************************************************************/

static inline uint8_t sam_serialin8(struct sam_dev_s *priv, int offset)
{
  return getreg8(priv->config->base + offset);
}

/****************************************************************************
 * Name: sam_serialout8
 ****************************************************************************/

static inline void sam_serialout8(struct sam_dev_s *priv, int offset,
                                  uint8_t regval)
{
  putreg8(regval, priv->config->base + offset);
}

/****************************************************************************
 * Name: sam_serialin16
 ****************************************************************************/

static inline uint16_t sam_serialin16(struct sam_dev_s *priv, int offset)
{
  return getreg16(priv->config->base + offset);
}

/****************************************************************************
 * Name: sam_serialout16
 ****************************************************************************/

static inline void sam_serialout16(struct sam_dev_s *priv, int offset,
                                   uint16_t regval)
{
  putreg16(regval, priv->config->base + offset);
}

/****************************************************************************
 * Name: sam_disableallints
 ****************************************************************************/

static void sam_disableallints(struct sam_dev_s *priv)
{
  /* Disable all interrupts */

  sam_serialout8(priv, SAM_USART_INTENCLR_OFFSET, USART_INT_ALL);
}

/****************************************************************************
 * Name: sam_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int sam_interrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct sam_dev_s *priv;
  uint8_t pending;
  uint8_t intflag;
  uint8_t inten;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct sam_dev_s *)dev->priv;

  /* Get the set of pending USART interrupts (we are only interested in the
   * unmasked interrupts).
   */

  intflag = sam_serialin8(priv, SAM_USART_INTFLAG_OFFSET);
  inten   = sam_serialin8(priv, SAM_USART_INTENCLR_OFFSET);
  pending = intflag & inten;

  /* Handle an incoming, receive byte.  The RXC flag is set when there is
   * unread data in DATA register.  This flag is cleared by reading the DATA
   * register (or by disabling the receiver).
   */

  if ((pending & USART_INT_RXC) != 0)
    {
      /* Received data ready... process incoming bytes */

       uart_recvchars(dev);
    }

  /* Handle outgoing, transmit bytes. The DRE flag is set when the DATA
   * register is empty and ready to be written.  This flag is cleared by
   * writing new data to the DATA register.  If there is no further data to
   * be transmitted,  the serial driver will disable TX interrupts, prohibit
   * further interrupts until TX interrupts are re-enabled.
   */

  if ((pending & USART_INT_DRE) != 0)
    {
      /* Transmit data register empty ... process outgoing bytes */

      uart_xmitchars(dev);
    }

  return OK;
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
  int ret = 0;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* Configure the SERCOM as a USART.  Don't reconfigure the console UART;
   * that was already done in sam_lowputc.c.
   */

  if (!dev->isconsole)
    {
      ret = sam_usart_initialize(priv->config);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: sam_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial port is
 *   closed.  The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void sam_shutdown(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* Resetting the SERCOM restores all registers to the reget state and
   * disables the SERCOM.  Ignore any requests to shutown the console
   * device (shouldn't happen).
   */

  if (!dev->isconsole)
    {
      sam_usart_reset(priv->config);
    }
}

/****************************************************************************
 * Name: sam_attach
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

static int sam_attach(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  const struct sam_usart_config_s * const config = priv->config;
  int ret;

  /* Attach and enable the Tx and Rx IRQs */

  ret = irq_attach(config->txirq, sam_interrupt, dev);
  if (ret == OK)
    {
      ret = irq_attach(config->rxirq, sam_interrupt, dev);
      if (ret == OK)
        {
          /* Enable the interrupt (RX and TX interrupts are still disabled
           * in the USART
           */

          up_enable_irq(config->txirq);
          up_enable_irq(config->rxirq);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sam_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void sam_detach(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  const struct sam_usart_config_s * const config = priv->config;

  /* Disable interrupts at the SERCOM device and at the NVIC */

  sam_disableallints(priv);
  up_disable_irq(config->txirq);
  up_disable_irq(config->rxirq);

  /* Detach the interrupt handler */

  irq_detach(config->txirq);
  irq_detach(config->rxirq);
}

/****************************************************************************
 * Name: sam_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int sam_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct sam_dev_s *user = (struct sam_dev_s *)arg;
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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* Return read status */

  *status = (uint32_t)sam_serialin16(priv, SAM_USART_STATUS_OFFSET);

  /* Then return the actual received byte */

  return (int)sam_serialin16(priv, SAM_USART_DATA_OFFSET);
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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Receive an interrupt when their is anything in the Rx data register */

      sam_serialout8(priv, SAM_USART_INTENSET_OFFSET, USART_INT_RXC);
#endif
    }
  else
    {
      sam_serialout8(priv, SAM_USART_INTENCLR_OFFSET, USART_INT_RXC);
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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  return ((sam_serialin8(priv, SAM_USART_INTFLAG_OFFSET) & USART_INT_RXC) != 0);
}

/****************************************************************************
 * Name: sam_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void sam_send(struct uart_dev_s *dev, int ch)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  sam_serialout16(priv, SAM_USART_DATA_OFFSET, (uint16_t)ch);
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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

      sam_serialout8(priv, SAM_USART_INTENSET_OFFSET, USART_INT_DRE);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      sam_serialout8(priv, SAM_USART_INTENCLR_OFFSET, USART_INT_DRE);
    }

  leave_critical_section(flags);
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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  return ((sam_serialin8(priv, SAM_USART_INTFLAG_OFFSET) & USART_INT_DRE) != 0);
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
 *   before sam_serialinit.
 *
 *   NOTE: On this platform up_earlyserialinit() does not really do
 *   anything of consequence and probably could be eliminated with little
 *   effort.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Disable all USARTS */

  sam_disableallints(TTYS0_DEV.priv);
#ifdef TTYS1_DEV
  sam_disableallints(TTYS1_DEV.priv);
#endif
#ifdef TTYS2_DEV
  sam_disableallints(TTYS2_DEV.priv);
#endif
#ifdef TTYS3_DEV
  sam_disableallints(TTYS3_DEV.priv);
#endif
#ifdef TTYS4_DEV
  sam_disableallints(TTYS4_DEV.priv);
#endif
#ifdef TTYS5_DEV
  sam_disableallints(TTYS5_DEV.priv);
#endif
#ifdef TTYS6_DEV
  sam_disableallints(TTYS6_DEV.priv);
#endif
#ifdef TTYS7_DEV
  sam_disableallints(TTYS7_DEV.priv);
#endif

#ifdef HAVE_SERIAL_CONSOLE
  /* Mark the serial console (if any) */

  CONSOLE_DEV.isconsole = true;
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that sam_earlyserialinit was called previously.
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
  irqstate_t flags;

  /* All interrupts must be disabled to prevent re-entrancy and to prevent
   * interrupts from firing in the serial driver code.
   */

  flags = enter_critical_section();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      sam_lowputc('\r');
    }

  sam_lowputc(ch);
  leave_critical_section(flags);
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

      sam_lowputc('\r');
    }

  sam_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
#endif /* SAMD5E5_HAVE_USART */
