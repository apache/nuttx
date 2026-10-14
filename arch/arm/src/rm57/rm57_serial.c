/****************************************************************************
 * arch/arm/src/rm57/rm57_serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Adapted from tms570_serial.c, using RM57's SCI register layout.
 * Supports SCI1-SCI4. Lower-half operations use the `up_` prefix rather
 * than `rm57_`, matching the convention used by e.g. stm32h7's
 * stm32_serial.c, since these are arch-private callback implementations
 * of the common serial upper half's uart_ops_s vtable rather than public
 * RM57 API entry points (those - rm57_lowsetup(), rm57_sci_configure() -
 * keep the `rm57_` prefix and live in rm57_lowputc.c).
 *
 * SCI1/LIN1 and SCI2/LIN2 are dual-role SCI/LIN modules; this driver
 * always runs them in SCI-compatibility mode (GCR1 LIN_MODE=0,
 * MBUF_MODE=0), which is why every DMA/interrupt request in this file is
 * generated on a byte-per-byte basis (TRM SPNU562A 28.2.3) rather than
 * the multi-buffered response DMA used by full LIN mode. SCI3/SCI4 are
 * SCI-only and behave identically in this mode.
 *
 * DMA support (CONFIG_RM57_SCIn_RXDMA/TXDMA) follows the pattern used by
 * stm32h7's stm32_serial.c: up to four uart_ops_s tables (no-DMA,
 * RX-only, TX-only, RX+TX), a circular RX buffer drained via
 * rm57_dmaresidual(), and one-shot TX transfers kicked off by
 * uart_xmitchars_dma(). See TRM 28.2.3 for the SCI-side DMA request
 * semantics this depends on: with MBUF_MODE=0, a DMA request is
 * generated for every single byte, so the DMA channel's "block" is
 * simply the RX ring buffer / TX chunk, transferred one byte (one frame,
 * TTYPE=0) per hardware request.
 */

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
#include <nuttx/debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_sci.h"
#include "hardware/rm57_dma.h"
#include "rm57_lowputc.h"
#include "rm57_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/* Which SCI will be console, ttyS0, ttyS1, ttyS2, ttyS3? The console (if
 * any) takes priority; remaining enabled SCIs are then assigned to
 * ttyS0..ttyS3 in SCI1..SCI4 order. This is the same SCIn_ASSIGNED-guard
 * ladder idiom used throughout NuttX's other multi-UART serial drivers
 * (e.g. stm32h7's stm32_serial.c).
 */

/* Console */

#if defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI1)
#  define CONSOLE_DEV     g_sci1priv.dev
#  define SCI1_ASSIGNED   1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI2)
#  define CONSOLE_DEV     g_sci2priv.dev
#  define SCI2_ASSIGNED   1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI3)
#  define CONSOLE_DEV     g_sci3priv.dev
#  define SCI3_ASSIGNED   1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI4)
#  define CONSOLE_DEV     g_sci4priv.dev
#  define SCI4_ASSIGNED   1
#else
#  undef CONSOLE_DEV
#endif

/* ttyS0 */

#if defined(CONFIG_RM57_SCI1) && !defined(SCI1_ASSIGNED)
#  define TTYS0_DEV       g_sci1priv.dev
#  define SCI1_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI2) && !defined(SCI2_ASSIGNED)
#  define TTYS0_DEV       g_sci2priv.dev
#  define SCI2_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI3) && !defined(SCI3_ASSIGNED)
#  define TTYS0_DEV       g_sci3priv.dev
#  define SCI3_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI4) && !defined(SCI4_ASSIGNED)
#  define TTYS0_DEV       g_sci4priv.dev
#  define SCI4_ASSIGNED   1
#endif

/* ttyS1 */

#if defined(CONFIG_RM57_SCI1) && !defined(SCI1_ASSIGNED)
#  define TTYS1_DEV       g_sci1priv.dev
#  define SCI1_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI2) && !defined(SCI2_ASSIGNED)
#  define TTYS1_DEV       g_sci2priv.dev
#  define SCI2_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI3) && !defined(SCI3_ASSIGNED)
#  define TTYS1_DEV       g_sci3priv.dev
#  define SCI3_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI4) && !defined(SCI4_ASSIGNED)
#  define TTYS1_DEV       g_sci4priv.dev
#  define SCI4_ASSIGNED   1
#endif

/* ttyS2 */

#if defined(CONFIG_RM57_SCI1) && !defined(SCI1_ASSIGNED)
#  define TTYS2_DEV       g_sci1priv.dev
#  define SCI1_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI2) && !defined(SCI2_ASSIGNED)
#  define TTYS2_DEV       g_sci2priv.dev
#  define SCI2_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI3) && !defined(SCI3_ASSIGNED)
#  define TTYS2_DEV       g_sci3priv.dev
#  define SCI3_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI4) && !defined(SCI4_ASSIGNED)
#  define TTYS2_DEV       g_sci4priv.dev
#  define SCI4_ASSIGNED   1
#endif

/* ttyS3 */

#if defined(CONFIG_RM57_SCI1) && !defined(SCI1_ASSIGNED)
#  define TTYS3_DEV       g_sci1priv.dev
#  define SCI1_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI2) && !defined(SCI2_ASSIGNED)
#  define TTYS3_DEV       g_sci2priv.dev
#  define SCI2_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI3) && !defined(SCI3_ASSIGNED)
#  define TTYS3_DEV       g_sci3priv.dev
#  define SCI3_ASSIGNED   1
#elif defined(CONFIG_RM57_SCI4) && !defined(SCI4_ASSIGNED)
#  define TTYS3_DEV       g_sci4priv.dev
#  define SCI4_ASSIGNED   1
#endif

/* Is RX/TX DMA available on any (enabled) SCI? */

#undef SERIAL_HAVE_RXDMA
#if defined(CONFIG_RM57_SCI1_RXDMA) || defined(CONFIG_RM57_SCI2_RXDMA) || \
    defined(CONFIG_RM57_SCI3_RXDMA) || defined(CONFIG_RM57_SCI4_RXDMA)
#  define SERIAL_HAVE_RXDMA 1
#endif

#undef SERIAL_HAVE_TXDMA
#if defined(CONFIG_RM57_SCI1_TXDMA) || defined(CONFIG_RM57_SCI2_TXDMA) || \
    defined(CONFIG_RM57_SCI3_TXDMA) || defined(CONFIG_RM57_SCI4_TXDMA)
#  define SERIAL_HAVE_TXDMA 1
#endif

/* Is RX/TX DMA used on the console SCI? */

#undef SERIAL_HAVE_CONSOLE_RXDMA
#if defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI1_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI2_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI3_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI4_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#endif

#undef SERIAL_HAVE_CONSOLE_TXDMA
#if defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI1_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI2_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI3_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RM57_SCI4_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#endif

/* Which of the (up to) four uart_ops_s tables are actually needed? */

#undef SERIAL_HAVE_NODMA_OPS
#if defined(CONFIG_RM57_SCI1) && !defined(CONFIG_RM57_SCI1_RXDMA) && \
    !defined(CONFIG_RM57_SCI1_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS 1
#elif defined(CONFIG_RM57_SCI2) && !defined(CONFIG_RM57_SCI2_RXDMA) && \
    !defined(CONFIG_RM57_SCI2_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS 1
#elif defined(CONFIG_RM57_SCI3) && !defined(CONFIG_RM57_SCI3_RXDMA) && \
    !defined(CONFIG_RM57_SCI3_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS 1
#elif defined(CONFIG_RM57_SCI4) && !defined(CONFIG_RM57_SCI4_RXDMA) && \
    !defined(CONFIG_RM57_SCI4_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS 1
#endif

#undef SERIAL_HAVE_RXTXDMA_OPS
#if defined(CONFIG_RM57_SCI1_RXDMA) && defined(CONFIG_RM57_SCI1_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS 1
#elif defined(CONFIG_RM57_SCI2_RXDMA) && defined(CONFIG_RM57_SCI2_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS 1
#elif defined(CONFIG_RM57_SCI3_RXDMA) && defined(CONFIG_RM57_SCI3_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS 1
#elif defined(CONFIG_RM57_SCI4_RXDMA) && defined(CONFIG_RM57_SCI4_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS 1
#endif

#undef SERIAL_HAVE_RXDMA_OPS
#if defined(CONFIG_RM57_SCI1_RXDMA) && !defined(CONFIG_RM57_SCI1_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS 1
#elif defined(CONFIG_RM57_SCI2_RXDMA) && !defined(CONFIG_RM57_SCI2_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS 1
#elif defined(CONFIG_RM57_SCI3_RXDMA) && !defined(CONFIG_RM57_SCI3_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS 1
#elif defined(CONFIG_RM57_SCI4_RXDMA) && !defined(CONFIG_RM57_SCI4_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS 1
#endif

#undef SERIAL_HAVE_TXDMA_OPS
#if !defined(CONFIG_RM57_SCI1_RXDMA) && defined(CONFIG_RM57_SCI1_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS 1
#elif !defined(CONFIG_RM57_SCI2_RXDMA) && defined(CONFIG_RM57_SCI2_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS 1
#elif !defined(CONFIG_RM57_SCI3_RXDMA) && defined(CONFIG_RM57_SCI3_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS 1
#elif !defined(CONFIG_RM57_SCI4_RXDMA) && defined(CONFIG_RM57_SCI4_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS 1
#endif

#undef SERIAL_HAVE_RXINT_METHODS
#if defined(SERIAL_HAVE_NODMA_OPS) || defined(SERIAL_HAVE_TXDMA_OPS)
#  define SERIAL_HAVE_RXINT_METHODS 1
#endif

#undef SERIAL_HAVE_TXINT_METHODS
#if defined(SERIAL_HAVE_NODMA_OPS) || defined(SERIAL_HAVE_RXDMA_OPS)
#  define SERIAL_HAVE_TXINT_METHODS 1
#endif

/* Sentinel for a port's [rt]xdma_req field when that direction's DMA is
 * not configured for it (the struct field still exists whenever *any*
 * port in the file uses that direction of DMA).
 */

#define RM57_DMA_NOREQ ((unsigned int)-1)

#ifdef SERIAL_HAVE_RXDMA
/* The Cortex-R5 D-cache line size on this device is 32 bytes; the RX DMA
 * buffer must be a multiple of, and aligned to, that size so that
 * up_invalidate_dcache() calls on sub-ranges of it never touch a byte
 * shared with unrelated data.
 */

#  define RXDMA_BUFFER_MASK  31
#  define RXDMA_BUFFER_SIZE  ((CONFIG_RM57_SERIAL_RXDMA_BUFFER_SIZE + \
                               RXDMA_BUFFER_MASK) & ~RXDMA_BUFFER_MASK)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  struct uart_dev_s dev;         /* Generic UART device state */
  const uint32_t scibase;        /* Base address of SCI registers */
  struct sci_config_s config;    /* SCI configuration */
  const uint8_t irq;             /* IRQ associated with this SCI */

#ifdef SERIAL_HAVE_TXDMA
  const unsigned int txdma_req;  /* DMA request line, or RM57_DMA_NOREQ */
  DMA_HANDLE txdma;              /* Currently-open transmit DMA channel */
  bool txdmabusy;                /* A transmit block is still in flight */
#endif

#ifdef SERIAL_HAVE_RXDMA
  const unsigned int rxdma_req;  /* DMA request line, or RM57_DMA_NOREQ */
  DMA_HANDLE rxdma;              /* Currently-open receive DMA channel */
  bool rxenable;                 /* DMA-based reception en/disable */
  uint32_t rxdmanext;            /* Next byte in the DMA buffer to read */
  uint32_t rxdmaavail;           /* Bytes available without invalidating
                                   * the D-cache */
  char *const rxfifo;            /* Receive DMA buffer */
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
#ifdef SERIAL_HAVE_RXINT_METHODS
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
#endif
static void up_send(struct uart_dev_s *dev, int ch);
#ifdef SERIAL_HAVE_TXINT_METHODS
static void up_txint(struct uart_dev_s *dev, bool enable);
#endif
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_RXDMA
static int  up_dma_nextrx(struct up_dev_s *priv);
static int  up_dma_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool up_dma_rxavailable(struct uart_dev_s *dev);
static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status,
                               void *arg);
#endif

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_send(struct uart_dev_s *dev);
static void up_dma_txint(struct uart_dev_s *dev, bool enable);
static void up_dma_txavailable(struct uart_dev_s *dev);
static void up_dma_txcallback(DMA_HANDLE handle, uint8_t status,
                               void *arg);
#endif

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int  up_dma_setup(struct uart_dev_s *dev);
static void up_dma_shutdown(struct uart_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef SERIAL_HAVE_NODMA_OPS
static const struct uart_ops_s g_sci_ops =
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
#endif

#ifdef SERIAL_HAVE_RXTXDMA_OPS
static const struct uart_ops_s g_sci_rxtxdma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_dma_receive,
  .rxint          = up_dma_rxint,
  .rxavailable    = up_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_dma_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
  .dmatxavail     = up_dma_txavailable,
  .dmasend        = up_dma_send,
};
#endif

#ifdef SERIAL_HAVE_RXDMA_OPS
static const struct uart_ops_s g_sci_rxdma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_dma_receive,
  .rxint          = up_dma_rxint,
  .rxavailable    = up_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};
#endif

#ifdef SERIAL_HAVE_TXDMA_OPS
static const struct uart_ops_s g_sci_txdma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
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
  .txint          = up_dma_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
  .dmatxavail     = up_dma_txavailable,
  .dmasend        = up_dma_send,
};
#endif

/* RX DMA buffers. Must be a multiple of, and aligned to, the D-cache line
 * size (see RXDMA_BUFFER_SIZE above) and positioned in DMA-able memory.
 */

#ifdef CONFIG_RM57_SCI1_RXDMA
static char g_sci1rxfifo[RXDMA_BUFFER_SIZE] aligned_data(32);
#endif

#ifdef CONFIG_RM57_SCI2_RXDMA
static char g_sci2rxfifo[RXDMA_BUFFER_SIZE] aligned_data(32);
#endif

#ifdef CONFIG_RM57_SCI3_RXDMA
static char g_sci3rxfifo[RXDMA_BUFFER_SIZE] aligned_data(32);
#endif

#ifdef CONFIG_RM57_SCI4_RXDMA
static char g_sci4rxfifo[RXDMA_BUFFER_SIZE] aligned_data(32);
#endif

/* I/O buffers */

#ifdef CONFIG_RM57_SCI1
static char g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif

#ifdef CONFIG_RM57_SCI2
static char g_sci2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_sci2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#endif

#ifdef CONFIG_RM57_SCI3
static char g_sci3rxbuffer[CONFIG_SCI3_RXBUFSIZE];
static char g_sci3txbuffer[CONFIG_SCI3_TXBUFSIZE];
#endif

#ifdef CONFIG_RM57_SCI4
static char g_sci4rxbuffer[CONFIG_SCI4_RXBUFSIZE];
static char g_sci4txbuffer[CONFIG_SCI4_TXBUFSIZE];
#endif

/* This describes the state of the SCI1/LIN1 port. */

#ifdef CONFIG_RM57_SCI1
static struct up_dev_s g_sci1priv =
{
  .dev            =
  {
    .recv     =
    {
      .size   = CONFIG_SCI1_RXBUFSIZE,
      .buffer = g_sci1rxbuffer,
    },
    .xmit     =
    {
      .size   = CONFIG_SCI1_TXBUFSIZE,
      .buffer = g_sci1txbuffer,
    },
#if defined(CONFIG_RM57_SCI1_RXDMA) && defined(CONFIG_RM57_SCI1_TXDMA)
    .ops      = &g_sci_rxtxdma_ops,
#elif defined(CONFIG_RM57_SCI1_RXDMA) && !defined(CONFIG_RM57_SCI1_TXDMA)
    .ops      = &g_sci_rxdma_ops,
#elif !defined(CONFIG_RM57_SCI1_RXDMA) && defined(CONFIG_RM57_SCI1_TXDMA)
    .ops      = &g_sci_txdma_ops,
#else
    .ops      = &g_sci_ops,
#endif
    .priv     = &g_sci1priv,
  },
  .scibase        = RM57_SCI1_BASE,
  .config         =
  {
    .baud         = CONFIG_SCI1_BAUD,
    .parity       = 0,
    .bits         = 8,
    .stopbits2    = CONFIG_SCI1_2STOP,
  },
  .irq            = RM57_REQ_LIN1HIGH,
#ifdef SERIAL_HAVE_TXDMA
#  ifdef CONFIG_RM57_SCI1_TXDMA
  .txdma_req      = DMA_REQ_LIN1TX,
#  else
  .txdma_req      = RM57_DMA_NOREQ,
#  endif
#endif
#ifdef SERIAL_HAVE_RXDMA
#  ifdef CONFIG_RM57_SCI1_RXDMA
  .rxdma_req      = DMA_REQ_LIN1RX,
  .rxfifo         = g_sci1rxfifo,
#  else
  .rxdma_req      = RM57_DMA_NOREQ,
  .rxfifo         = NULL,
#  endif
#endif
};
#endif

/* This describes the state of the SCI2/LIN2 port. */

#ifdef CONFIG_RM57_SCI2
static struct up_dev_s g_sci2priv =
{
  .dev            =
  {
    .recv     =
    {
      .size   = CONFIG_SCI2_RXBUFSIZE,
      .buffer = g_sci2rxbuffer,
    },
    .xmit     =
    {
      .size   = CONFIG_SCI2_TXBUFSIZE,
      .buffer = g_sci2txbuffer,
    },
#if defined(CONFIG_RM57_SCI2_RXDMA) && defined(CONFIG_RM57_SCI2_TXDMA)
    .ops      = &g_sci_rxtxdma_ops,
#elif defined(CONFIG_RM57_SCI2_RXDMA) && !defined(CONFIG_RM57_SCI2_TXDMA)
    .ops      = &g_sci_rxdma_ops,
#elif !defined(CONFIG_RM57_SCI2_RXDMA) && defined(CONFIG_RM57_SCI2_TXDMA)
    .ops      = &g_sci_txdma_ops,
#else
    .ops      = &g_sci_ops,
#endif
    .priv     = &g_sci2priv,
  },
  .scibase        = RM57_SCI2_BASE,
  .config         =
  {
    .baud         = CONFIG_SCI2_BAUD,
    .parity       = 0,
    .bits         = 8,
    .stopbits2    = CONFIG_SCI2_2STOP,
#ifdef CONFIG_RM57_SCI2_LOOPBACK
    .loopback     = true,
#endif
  },
  .irq            = RM57_REQ_LIN2HIGH,
#ifdef SERIAL_HAVE_TXDMA
#  ifdef CONFIG_RM57_SCI2_TXDMA
  .txdma_req      = DMA_REQ_LIN2TX,
#  else
  .txdma_req      = RM57_DMA_NOREQ,
#  endif
#endif
#ifdef SERIAL_HAVE_RXDMA
#  ifdef CONFIG_RM57_SCI2_RXDMA
  .rxdma_req      = DMA_REQ_LIN2RX,
  .rxfifo         = g_sci2rxfifo,
#  else
  .rxdma_req      = RM57_DMA_NOREQ,
  .rxfifo         = NULL,
#  endif
#endif
};
#endif

/* This describes the state of the SCI3 port. */

#ifdef CONFIG_RM57_SCI3
static struct up_dev_s g_sci3priv =
{
  .dev            =
  {
    .recv     =
    {
      .size   = CONFIG_SCI3_RXBUFSIZE,
      .buffer = g_sci3rxbuffer,
    },
    .xmit     =
    {
      .size   = CONFIG_SCI3_TXBUFSIZE,
      .buffer = g_sci3txbuffer,
    },
#if defined(CONFIG_RM57_SCI3_RXDMA) && defined(CONFIG_RM57_SCI3_TXDMA)
    .ops      = &g_sci_rxtxdma_ops,
#elif defined(CONFIG_RM57_SCI3_RXDMA) && !defined(CONFIG_RM57_SCI3_TXDMA)
    .ops      = &g_sci_rxdma_ops,
#elif !defined(CONFIG_RM57_SCI3_RXDMA) && defined(CONFIG_RM57_SCI3_TXDMA)
    .ops      = &g_sci_txdma_ops,
#else
    .ops      = &g_sci_ops,
#endif
    .priv     = &g_sci3priv,
  },
  .scibase        = RM57_SCI3_BASE,
  .config         =
  {
    .baud         = CONFIG_SCI3_BAUD,
    .parity       = 0,
    .bits         = 8,
    .stopbits2    = CONFIG_SCI3_2STOP,
#ifdef CONFIG_RM57_SCI3_LOOPBACK
    .loopback     = true,
#endif
  },
  .irq            = RM57_REQ_SCI3HIGH,
#ifdef SERIAL_HAVE_TXDMA
#  ifdef CONFIG_RM57_SCI3_TXDMA
  .txdma_req      = DMA_REQ_SCI3TX,
#  else
  .txdma_req      = RM57_DMA_NOREQ,
#  endif
#endif
#ifdef SERIAL_HAVE_RXDMA
#  ifdef CONFIG_RM57_SCI3_RXDMA
  .rxdma_req      = DMA_REQ_SCI3RX,
  .rxfifo         = g_sci3rxfifo,
#  else
  .rxdma_req      = RM57_DMA_NOREQ,
  .rxfifo         = NULL,
#  endif
#endif
};
#endif

/* This describes the state of the SCI4 port. */

#ifdef CONFIG_RM57_SCI4
static struct up_dev_s g_sci4priv =
{
  .dev            =
  {
    .recv     =
    {
      .size   = CONFIG_SCI4_RXBUFSIZE,
      .buffer = g_sci4rxbuffer,
    },
    .xmit     =
    {
      .size   = CONFIG_SCI4_TXBUFSIZE,
      .buffer = g_sci4txbuffer,
    },
#if defined(CONFIG_RM57_SCI4_RXDMA) && defined(CONFIG_RM57_SCI4_TXDMA)
    .ops      = &g_sci_rxtxdma_ops,
#elif defined(CONFIG_RM57_SCI4_RXDMA) && !defined(CONFIG_RM57_SCI4_TXDMA)
    .ops      = &g_sci_rxdma_ops,
#elif !defined(CONFIG_RM57_SCI4_RXDMA) && defined(CONFIG_RM57_SCI4_TXDMA)
    .ops      = &g_sci_txdma_ops,
#else
    .ops      = &g_sci_ops,
#endif
    .priv     = &g_sci4priv,
  },
  .scibase        = RM57_SCI4_BASE,
  .config         =
  {
    .baud         = CONFIG_SCI4_BAUD,
    .parity       = 0,
    .bits         = 8,
    .stopbits2    = CONFIG_SCI4_2STOP,
#ifdef CONFIG_RM57_SCI4_LOOPBACK
    .loopback     = true,
#endif
  },
  .irq            = RM57_REQ_SCI4HIGH,
#ifdef SERIAL_HAVE_TXDMA
#  ifdef CONFIG_RM57_SCI4_TXDMA
  .txdma_req      = DMA_REQ_SCI4TX,
#  else
  .txdma_req      = RM57_DMA_NOREQ,
#  endif
#endif
#ifdef SERIAL_HAVE_RXDMA
#  ifdef CONFIG_RM57_SCI4_RXDMA
  .rxdma_req      = DMA_REQ_SCI4RX,
  .rxfifo         = g_sci4rxfifo,
#  else
  .rxdma_req      = RM57_DMA_NOREQ,
  .rxfifo         = NULL,
#  endif
#endif
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
  return getreg32(priv->scibase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                 uint32_t value)
{
  putreg32(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: up_restoresciint
 ****************************************************************************/

static inline void up_restoresciint(struct up_dev_s *priv, uint32_t ints)
{
  up_serialout(priv, RM57_SCI_SETINT_OFFSET, ints);
}

/****************************************************************************
 * Name: up_disableallints
 ****************************************************************************/

static void up_disableallints(struct up_dev_s *priv, uint32_t *ints)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = enter_critical_section();
  if (ints)
    {
      *ints = up_serialin(priv, RM57_SCI_SETINT_OFFSET);
    }

  up_serialout(priv, RM57_SCI_CLEARINT_OFFSET, SCI_INT_ALLINTS);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_SCI_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return rm57_sci_configure(priv->scibase, &priv->config);
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_serialout(priv, RM57_SCI_GCR1_OFFSET, 0);
  up_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the SCI to operate in interrupt driven mode.  This method
 *   is called when the serial port is opened, normally just after the
 *   setup() method is called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port
 *   is closed normally, just before the shutdown method is called.
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
 *   This is the common SCI interrupt handler.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv;
  uint32_t intvec;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  for (; ; )
    {
      /* Reading INTVECT0 clears the corresponding INTFLAG bit for most
       * interrupt sources.
       */

      intvec = up_serialin(priv, RM57_SCI_INTVECT0_OFFSET) &
               SCI_INTVECT_MASK;

      switch (intvec)
        {
          case SCI_INTVECT_NONE:    /* No interrupt */
            return OK;

          case SCI_INTVECT_WAKEUP:  /* Wake-up interrupt (ignored) */
            break;

          /* SCI errors: ignored for now, since break-detect interrupt
           * is never enabled
           */

          case SCI_INTVECT_PE:
          case SCI_INTVECT_FE:
          case SCI_INTVECT_OE:
          case SCI_INTVECT_BRKDT:
          case SCI_INTVECT_BE:
            break;

          case SCI_INTVECT_RX:      /* Receive interrupt */
            uart_recvchars(dev);
            break;

          case SCI_INTVECT_TX:      /* Transmit interrupt */
            uart_xmitchars(dev);
            break;

          /* LIN mode only.  These should never occur in SCI mode */

          case SCI_INTVECT_ISFE:
          case SCI_INTVECT_ID:
          case SCI_INTVECT_PBE:
          case SCI_INTVECT_CE:
          case SCI_INTVECT_NRE:
          case SCI_INTVECT_TOAWUS:
          case SCI_INTVECT_TOA3WUS:
          case SCI_INTVECT_TIMEOUT:
          default:
            DEBUGPANIC();
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
              memcpy(user, dev->priv, sizeof(struct up_dev_s));
            }
        }
        break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios  *termiosp = (struct termios *)arg;
          struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          termiosp->c_cflag = ((priv->config.parity != 0) ? PARENB : 0) |
                              ((priv->config.parity == 1) ? PARODD : 0);
          termiosp->c_cflag |= (priv->config.stopbits2) ? CSTOPB : 0;
          cfsetispeed(termiosp, priv->config.baud);

          switch (priv->config.bits)
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
            }
        }
        break;

      case TCSETS:
        {
          struct termios  *termiosp = (struct termios *)arg;
          struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
          uint32_t baud;
          uint32_t ints;
          uint8_t parity;
          uint8_t nbits;
          bool stop2;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          ret = OK;
          baud = cfgetispeed(termiosp);

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

              default:
                ret = -EINVAL;
                break;
            }

          if ((termiosp->c_cflag & PARENB) != 0)
            {
              parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              parity = 0;
            }

          stop2 = (termiosp->c_cflag & CSTOPB) != 0;

          if (ret == OK)
            {
              priv->config.baud      = baud;
              priv->config.parity    = parity;
              priv->config.bits      = nbits;
              priv->config.stopbits2 = stop2;

              up_disableallints(priv, &ints);
              ret = rm57_sci_configure(priv->scibase, &priv->config);
              up_restoresciint(priv, ints);
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
 *   Called (usually) from the interrupt level to receive one character
 *   from the SCI.  Error bits associated with the receipt are provided
 *   in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXINT_METHODS
static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  *status = up_serialin(priv, RM57_SCI_FLR_OFFSET);
  return (int)(up_serialin(priv, RM57_SCI_RD_OFFSET) & 0xff);
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
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_serialout(priv, RM57_SCI_SETINT_OFFSET, SCI_INT_RX);
#endif
    }
  else
    {
      up_serialout(priv, RM57_SCI_CLEARINT_OFFSET, SCI_INT_RX);
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

  return ((up_serialin(priv, RM57_SCI_FLR_OFFSET) & SCI_FLR_RXRDY) != 0);
}
#endif

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_serialout(priv, RM57_SCI_TD_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXINT_METHODS
static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_serialout(priv, RM57_SCI_SETINT_OFFSET, SCI_INT_TX);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      up_serialout(priv, RM57_SCI_CLEARINT_OFFSET, SCI_INT_TX);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return ((up_serialin(priv, RM57_SCI_FLR_OFFSET) & SCI_FLR_TXRDY) != 0);
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

  return ((up_serialin(priv, RM57_SCI_FLR_OFFSET) &
           SCI_FLR_TXEMPTY) != 0);
}

/****************************************************************************
 * Name: up_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA engine will write
 *   the next byte, computed from the DMA channel's residual element
 *   count.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int up_dma_nextrx(struct up_dev_s *priv)
{
  size_t dmaresidual = rm57_dmaresidual(priv->rxdma);

  /* The residual can never exceed the size of the ring the channel was
   * programmed with.  Clamp it rather than let an out-of-range reading turn
   * into a negative index: C's % keeps the sign of the dividend, so a
   * residual of RXDMA_BUFFER_SIZE + 1 would silently yield -1 here, which
   * up_dma_rxavailable() would then read as "bytes are waiting" forever.
   */

  if (dmaresidual > RXDMA_BUFFER_SIZE)
    {
      dmaresidual = RXDMA_BUFFER_SIZE;
    }

  return (int)((RXDMA_BUFFER_SIZE - dmaresidual) % RXDMA_BUFFER_SIZE);
}
#endif

/****************************************************************************
 * Name: up_dma_setup
 *
 * Description:
 *   Configure the SCI baud/bits/parity as with up_setup(), then acquire
 *   and arm the RX and/or TX DMA channels configured for this port.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int up_dma_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Do the basic SCI setup first, unless we are the console (which was
   * already configured by rm57_lowsetup()).
   */

  if (!dev->isconsole)
    {
      ret = up_setup(dev);
      if (ret != OK)
        {
          return ret;
        }
    }

#ifdef SERIAL_HAVE_TXDMA
  if (priv->txdma_req != RM57_DMA_NOREQ)
    {
      priv->txdma     = rm57_dmachannel(priv->txdma_req);
      priv->txdmabusy = false;
    }
#endif

#ifdef SERIAL_HAVE_RXDMA
  if (priv->rxdma_req != RM57_DMA_NOREQ)
    {
      struct rm57_dmacfg_s rxdmacfg;

      priv->rxdma = rm57_dmachannel(priv->rxdma_req);

      /* Configure for circular DMA reception into the RX FIFO: one byte
       * (one frame, 8-bit elements) is transferred per hardware request,
       * source address fixed at the SCI receive data register,
       * destination address auto-incrementing through the ring buffer.
       * AIM keeps the channel armed for hardware triggering past the end
       * of a block (see the file-level comment on TRM 20.2.8/20.3.1.5 for
       * why this gives circular behavior for a byte-per-byte hardware
       * request source like this one).
       */

      rxdmacfg.saddr     = priv->scibase + RM57_SCI_RD_DMA_OFFSET;
      rxdmacfg.daddr     = (uint32_t)priv->rxfifo;
      rxdmacfg.nframes   = RXDMA_BUFFER_SIZE;
      rxdmacfg.nelems    = 1;
      rxdmacfg.chctrl    = DMA_CHCTRL_RES_8BIT | DMA_CHCTRL_WES_8BIT |
                            DMA_CHCTRL_ADDMR_CONST |
                            DMA_CHCTRL_ADDMW_INCR | DMA_CHCTRL_AIM;
      rxdmacfg.parassign = DMA_PAR_AB_B_RD_A_WR;
      rm57_dmasetup(priv->rxdma, &rxdmacfg);

      /* Reset our DMA shadow pointer and Rx data availability count to
       * match the buffer just programmed above.
       */

      priv->rxdmanext  = 0;
      priv->rxdmaavail = 0;

      up_invalidate_dcache((uintptr_t)priv->rxfifo,
                            (uintptr_t)priv->rxfifo + RXDMA_BUFFER_SIZE);

      /* Enable RX DMA requests at the SCI (byte-per-byte, since
       * MBUF_MODE=0) and start the DMA channel, arranging for callbacks
       * at the half and full points in the FIFO so there is always half
       * a FIFO worth of time to claim bytes before they are
       * overwritten.
       *
       * SET RX INT must be set as well as SET RX DMA: per TRM 29.4.1 it
       * is SET RX INT that arms the RXRDY event, while SET RX DMA only
       * steers the resulting request to the DMA controller instead of the
       * CPU.  With SET RX DMA set no CPU interrupt is generated (TRM
       * 28.2.2.2: both DMA bits must be *cleared* to select interrupt
       * functionality), so this does not disturb up_interrupt().
       *
       * The transmit side needs no equivalent because TXRDY is level-high
       * whenever SCITD is empty, so arming SET TX DMA is enough to make
       * the first request fire.
       */

      up_serialout(priv, RM57_SCI_SETINT_OFFSET,
                   SCI_INT_RX | SCI_INT_RXDMA | SCI_INT_RXDMAALL);

      rm57_dmastart(priv->rxdma, up_dma_rxcallback, priv, true);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: up_dma_shutdown
 *
 * Description:
 *   Disable the SCI and release any DMA channels acquired by
 *   up_dma_setup().
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static void up_dma_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_serialout(priv, RM57_SCI_CLEARINT_OFFSET,
               SCI_INT_RX | SCI_INT_TXDMA | SCI_INT_RXDMA |
               SCI_INT_RXDMAALL);

  up_shutdown(dev);

#ifdef SERIAL_HAVE_TXDMA
  if (priv->txdma != NULL)
    {
      rm57_dmastop(priv->txdma);
      rm57_dmafree(priv->txdma);
      priv->txdma     = NULL;
      priv->txdmabusy = false;
    }
#endif

#ifdef SERIAL_HAVE_RXDMA
  if (priv->rxdma != NULL)
    {
      rm57_dmastop(priv->rxdma);
      rm57_dmafree(priv->rxdma);
      priv->rxdma = NULL;
    }
#endif
}
#endif

/****************************************************************************
 * Name: up_dma_receive
 *
 * Description:
 *   Called (usually) from the DMA callback level to receive one character
 *   from the SCI's RX DMA ring buffer.  Error status is not tracked per
 *   byte in DMA mode; 0 is always returned.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int up_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int nextrx = up_dma_nextrx(priv);
  int c = 0;

  *status = 0;

  /* Check if more data is available */

  if (nextrx != (int)priv->rxdmanext)
    {
      /* If the data cache is enabled, then we also need to manage cache
       * coherency.  Are any bytes available in the currently coherent
       * region of the data cache?
       */

      if (priv->rxdmaavail == 0)
        {
          uint32_t rxdmaavail;
          uintptr_t addr;

          /* No.. then we will have to invalidate additional space in the
           * Rx DMA buffer.
           */

          if (nextrx > (int)priv->rxdmanext)
            {
              rxdmaavail = nextrx - priv->rxdmanext;
            }
          else
            {
              rxdmaavail = RXDMA_BUFFER_SIZE - priv->rxdmanext;
            }

          addr = (uintptr_t)&priv->rxfifo[priv->rxdmanext];
          up_invalidate_dcache(addr, addr + rxdmaavail);

          priv->rxdmaavail = rxdmaavail;
        }

      priv->rxdmaavail--;

      /* Now read from the DMA buffer */

      c = priv->rxfifo[priv->rxdmanext];

      priv->rxdmanext++;
      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
          priv->rxdmanext = 0;
        }
    }

  /* NOTE: If no data is available, we return 0, which is of course valid
   * binary data.  The protocol is that the upper half driver must call
   * up_dma_rxavailable() prior to calling this function to assure that
   * this never happens.
   */

  return c;
}
#endif

/****************************************************************************
 * Name: up_dma_rxint
 *
 * Description:
 *   Call to enable or disable RX reception via DMA
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void up_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Note that it is not safe to check for available bytes and
   * immediately pass them to uart_recvchars() as that could potentially
   * recurse back into us.  Instead, bytes wait until the next DMA
   * callback or rm57_serial_dma_poll() call.
   */

  priv->rxenable = enable;
}
#endif

/****************************************************************************
 * Name: up_dma_rxavailable
 *
 * Description:
 *   Return true if there are unclaimed bytes in the RX DMA ring buffer
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static bool up_dma_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return (up_dma_nextrx(priv) != (int)priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: up_dma_rxcallback
 *
 * Description:
 *   DMA HBC/BTC callback: pass any newly available bytes up to the
 *   generic serial layer.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct up_dev_s *priv = (struct up_dev_s *)arg;

  if (priv->rxenable && up_dma_rxavailable(&priv->dev))
    {
      uart_recvchars(&priv->dev);
    }
}
#endif

/****************************************************************************
 * Name: up_dma_send
 *
 * Description:
 *   Called (usually) from the interrupt level to start a TX DMA transfer.
 *   (Re-)configures the TX DMA channel with the current xmit buffer
 *   contents.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_send(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  struct rm57_dmacfg_s txdmacfg;

  /* Stop DMA before reconfiguration */

  rm57_dmastop(priv->txdma);

  dev->dmatx.nbytes = 0;

  /* Flush the contents of the TX buffer(s) into physical memory before
   * the DMA engine reads them.
   */

  up_clean_dcache((uintptr_t)dev->dmatx.buffer,
                  (uintptr_t)dev->dmatx.buffer + dev->dmatx.length);

  if (dev->dmatx.nbuffer)
    {
      up_clean_dcache((uintptr_t)dev->dmatx.nbuffer,
                      (uintptr_t)dev->dmatx.nbuffer + dev->dmatx.nlength);
    }

  txdmacfg.saddr     = (uint32_t)dev->dmatx.buffer;
  txdmacfg.daddr     = priv->scibase + RM57_SCI_TD_DMA_OFFSET;
  txdmacfg.nframes   = dev->dmatx.length;
  txdmacfg.nelems    = 1;
  txdmacfg.chctrl    = DMA_CHCTRL_RES_8BIT | DMA_CHCTRL_WES_8BIT |
                        DMA_CHCTRL_ADDMR_INCR | DMA_CHCTRL_ADDMW_CONST;
  txdmacfg.parassign = DMA_PAR_AB_A_RD_B_WR;
  rm57_dmasetup(priv->txdma, &txdmacfg);

  up_serialout(priv, RM57_SCI_SETINT_OFFSET, SCI_INT_TXDMA);

  priv->txdmabusy = true;
  rm57_dmastart(priv->txdma, up_dma_txcallback, priv, false);
}
#endif

/****************************************************************************
 * Name: up_dma_txavailable
 *
 * Description:
 *   Informs the DMA engine that TX data is available and ready for
 *   transfer, if it is currently idle.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_txavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

  /* Whether a block is still going out is tracked here rather than asked
   * of the DMA: CTCOUNT in the working control packet is only written back
   * when the channel is arbitrated out of the priority queue (TRM Table
   * 20-108), so a channel that is busy - or merely still resident in the
   * queue - can report a stale count in either direction.  Deciding
   * "transmitter idle" from it risks either dropping the next block or
   * starting it on top of a live one.
   */

  if (!priv->txdmabusy && dev->dmatx.length == 0 && dev->dmatx.nlength == 0)
    {
      uart_xmitchars_dma(dev);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_dma_txcallback
 *
 * Description:
 *   DMA BTC callback for a TX transfer: advances the xmit buffer and
 *   either starts the next (split) transfer or notifies the generic
 *   serial layer that the transfer is done.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_txcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct up_dev_s *priv = (struct up_dev_s *)arg;
  struct uart_dev_s *dev = &priv->dev;

  /* Only a block transfer completion ends a transmit.  Anything else -
   * a half-block notification in particular - must leave the SCI's TX DMA
   * request armed, or the rest of the block never goes out.
   */

  if ((status & RM57_DMA_STATUS_BTC) == 0)
    {
      return;
    }

  dev->dmatx.nbytes += dev->dmatx.length;
  if (dev->dmatx.nlength)
    {
      struct rm57_dmacfg_s txdmacfg;

      txdmacfg.saddr     = (uint32_t)dev->dmatx.nbuffer;
      txdmacfg.daddr     = priv->scibase + RM57_SCI_TD_DMA_OFFSET;
      txdmacfg.nframes   = dev->dmatx.nlength;
      txdmacfg.nelems    = 1;
      txdmacfg.chctrl    = DMA_CHCTRL_RES_8BIT | DMA_CHCTRL_WES_8BIT |
                            DMA_CHCTRL_ADDMR_INCR |
                            DMA_CHCTRL_ADDMW_CONST;
      txdmacfg.parassign = DMA_PAR_AB_A_RD_B_WR;
      rm57_dmasetup(priv->txdma, &txdmacfg);

      dev->dmatx.length  = dev->dmatx.nlength;
      dev->dmatx.nlength = 0;

      rm57_dmastart(priv->txdma, up_dma_txcallback, priv, false);
      return;
    }

  /* Transfer complete and no follow-on buffer: stop generating TX DMA
   * requests until the next up_dma_send() call.
   */

  up_serialout(priv, RM57_SCI_CLEARINT_OFFSET, SCI_INT_TXDMA);

  priv->txdmabusy = false;

  uart_xmitchars_done(dev);

  up_dma_txavailable(dev);
}
#endif

/****************************************************************************
 * Name: up_dma_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.  Nothing to do here: TX DMA
 *   requests replace the TX interrupt entirely, and are activated by
 *   up_dma_send()/up_dma_txcallback() instead.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_txint(struct uart_dev_s *dev, bool enable)
{
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  /* Disable all SCIs */

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

#ifdef CONSOLE_DEV
  /* Configure whichever one is the console.  NOTE: this was already done
   * in rm57_lowsetup().
   */

  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);

  uart_register("/dev/console", &CONSOLE_DEV);

#if defined(SERIAL_HAVE_CONSOLE_RXDMA) || defined(SERIAL_HAVE_CONSOLE_TXDMA)
  /* If DMA is enabled on the console, layer the DMA channel setup on top
   * of the basic register configuration performed just above.
   */

  up_dma_setup(&CONSOLE_DEV);
#endif
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
}

/****************************************************************************
 * Name: rm57_serial_dma_poll
 *
 * Description:
 *   Checks RX DMA buffers for received bytes that have not yet
 *   accumulated to the point where the DMA half/full (HBC/BTC) interrupt
 *   has triggered. Boards using RXDMA and needing low-latency reception
 *   of short, infrequent messages should call this periodically (e.g.
 *   from a timer).
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void rm57_serial_dma_poll(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

#ifdef CONFIG_RM57_SCI1_RXDMA
  if (g_sci1priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_sci1priv.rxdma, 0, &g_sci1priv);
    }
#endif

#ifdef CONFIG_RM57_SCI2_RXDMA
  if (g_sci2priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_sci2priv.rxdma, 0, &g_sci2priv);
    }
#endif

#ifdef CONFIG_RM57_SCI3_RXDMA
  if (g_sci3priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_sci3priv.rxdma, 0, &g_sci3priv);
    }
#endif

#ifdef CONFIG_RM57_SCI4_RXDMA
  if (g_sci4priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_sci4priv.rxdma, 0, &g_sci4priv);
    }
#endif

  leave_critical_section(flags);
}
#endif

#endif /* USE_SERIALDRIVER */
