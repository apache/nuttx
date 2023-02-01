/****************************************************************************
 * arch/arm/src/kinetis/kinetis_lpserial.c
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

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "arm_internal.h"
#include "kinetis_config.h"
#include "chip.h"
#include "hardware/kinetis_lpuart.h"
#include "hardware/kinetis_pinmux.h"
#include "hardware/kinetis_dmamux.h"
#include "kinetis.h"
#include "kinetis_lpuart.h"
#include "kinetis_edma.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there at least one LPUART enabled and configured as a RS-232 device? */

#ifndef HAVE_LPUART_DEVICE
#  warning "No LPUARTs enabled"
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(HAVE_LPUART_DEVICE) && defined(USE_SERIALDRIVER)

/* Assume DMA is not used on the console UART */

#undef SERIAL_HAVE_CONSOLE_DMA

/* Which LPUART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered
 * LPUART.
 */

/* First pick the console and ttys0.  This could be any of LPUART0-4 */

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_lpuart0port /* LPUART0 is console */
#    define TTYS0_DEV           g_lpuart0port /* LPUART0 is ttyS0 */
#    define LPUART0_ASSIGNED    1
#  if defined(CONFIG_KINETIS_LPUART0_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_lpuart1port /* LPUART1 is console */
#    define TTYS0_DEV           g_lpuart1port /* LPUART1 is ttyS0 */
#    define LPUART1_ASSIGNED    1
#  if defined(CONFIG_KINETIS_LPUART1_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_lpuart2port /* LPUART2 is console */
#    define TTYS0_DEV           g_lpuart2port /* LPUART2 is ttyS0 */
#    define LPUART2_ASSIGNED    1
#  if defined(CONFIG_KINETIS_LPUART2_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_LPUART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_lpuart3port /* LPUART3 is console */
#    define TTYS0_DEV           g_lpuart3port /* LPUART3 is ttyS0 */
#    define LPUART3_ASSIGNED    1
#  if defined(CONFIG_KINETIS_LPUART3_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_LPUART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_lpuart4port /* LPUART4 is console */
#    define TTYS0_DEV           g_lpuart4port /* LPUART4 is ttyS0 */
#    define LPUART4_ASSIGNED    1
#  if defined(CONFIG_KINETIS_LPUART4_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#else
#  undef CONSOLE_DEV                          /* No console */
#  if defined(CONFIG_KINETIS_LPUART0)
#    define TTYS0_DEV           g_lpuart0port /* LPUART0 is ttyS0 */
#    define LPUART0_ASSIGNED    1
#  elif defined(CONFIG_KINETIS_LPUART1)
#    define TTYS0_DEV           g_lpuart1port /* LPUART1 is ttyS0 */
#    define LPUART1_ASSIGNED    1
#  elif defined(CONFIG_KINETIS_LPUART2)
#    define TTYS0_DEV           g_lpuart2port /* LPUART2 is ttyS0 */
#    define LPUART2_ASSIGNED    1
#  elif defined(CONFIG_KINETIS_LPUART3)
#    define TTYS0_DEV           g_lpuart3port /* LPUART3 is ttyS0 */
#    define LPUART3_ASSIGNED    1
#  elif defined(CONFIG_KINETIS_LPUART4)
#    define TTYS0_DEV           g_lpuart4port /* LPUART4 is ttyS0 */
#    define LPUART4_ASSIGNED    1
#  endif
#endif

/* Pick ttys1. This could be any of LPUART0-4 excluding the console/ttyS0
 * LPUART.
 */

#if defined(CONFIG_KINETIS_LPUART0) && !defined(LPUART0_ASSIGNED)
#  define TTYS1_DEV             g_lpuart0port /* LPUART0 is ttyS1 */
#  define LPUART0_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART1) && !defined(LPUART1_ASSIGNED)
#  define TTYS1_DEV             g_lpuart1port /* LPUART1 is ttyS1 */
#  define LPUART1_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART2) && !defined(LPUART2_ASSIGNED)
#  define TTYS1_DEV             g_lpuart2port /* LPUART2 is ttyS1 */
#  define LPUART2_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART3) && !defined(LPUART3_ASSIGNED)
#  define TTYS1_DEV             g_lpuart3port /* LPUART3 is ttyS1 */
#  define LPUART3_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART4) && !defined(LPUART4_ASSIGNED)
#  define TTYS1_DEV             g_lpuart4port /* LPUART4 is ttyS1 */
#  define LPUART4_ASSIGNED      1
#endif

/* Pick ttys2. This could be any of LPUART1-4 excluding the console/ttyS0
 * LPUART.
 */

#if defined(CONFIG_KINETIS_LPUART1) && !defined(LPUART1_ASSIGNED)
#  define TTYS2_DEV             g_lpuart1port /* LPUART1 is ttyS2 */
#  define LPUART1_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART2) && !defined(LPUART2_ASSIGNED)
#  define TTYS2_DEV             g_lpuart2port /* LPUART2 is ttyS2 */
#  define LPUART2_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART3) && !defined(LPUART3_ASSIGNED)
#  define TTYS2_DEV             g_lpuart3port /* LPUART3 is ttyS2 */
#  define LPUART3_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART4) && !defined(LPUART4_ASSIGNED)
#  define TTYS2_DEV             g_lpuart4port /* LPUART4 is ttyS2 */
#  define LPUART4_ASSIGNED      1
#endif

/* Pick ttys3. This could be any of LPUART2-4 excluding the console/ttyS0
 * LPUART.
 */

#if defined(CONFIG_KINETIS_LPUART2) && !defined(LPUART2_ASSIGNED)
#  define TTYS3_DEV             g_lpuart2port /* LPUART2 is ttyS3 */
#  define LPUART2_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART3) && !defined(LPUART3_ASSIGNED)
#  define TTYS3_DEV             g_lpuart3port /* LPUART3 is ttyS3 */
#  define LPUART3_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART4) && !defined(LPUART4_ASSIGNED)
#  define TTYS3_DEV             g_lpuart4port /* LPUART4 is ttyS3 */
#  define LPUART4_ASSIGNED      1
#endif

/* Pick ttys3. This could be any of LPUART3-4 excluding the console/ttyS0
 * LPUART.
 */

#if defined(CONFIG_KINETIS_LPUART3) && !defined(LPUART3_ASSIGNED)
#  define TTYS4_DEV             g_lpuart3port /* LPUART3 is ttyS4 */
#  define LPUART3_ASSIGNED      1
#elif defined(CONFIG_KINETIS_LPUART4) && !defined(LPUART4_ASSIGNED)
#  define TTYS4_DEV             g_lpuart4port /* LPUART4 is ttyS4 */
#  define LPUART4_ASSIGNED      1
#endif

#ifdef LPSERIAL_HAVE_DMA

/* The DMA buffer size when using RX DMA to emulate a FIFO.
 *
 * When streaming data, the generic serial layer will be called every time
 * the FIFO receives half this number of bytes.
 *
 * This buffer size should be an even multiple of the Cortex-M7 D-Cache line
 * size, ARMV7M_DCACHE_LINESIZE, so that it can be individually invalidated.
 *
 * Should there be a Cortex-M7 without a D-Cache, ARMV7M_DCACHE_LINESIZE
 * would be zero!
 */

#  if !defined(ARMV7M_DCACHE_LINESIZE) || ARMV7M_DCACHE_LINESIZE == 0
#    undef ARMV7M_DCACHE_LINESIZE
#    define ARMV7M_DCACHE_LINESIZE 32
#  endif

#  if !defined(CONFIG_KINETIS_LPUART_RXDMA_BUFFER_SIZE) || \
      (CONFIG_KINETIS_LPUART_RXDMA_BUFFER_SIZE < ARMV7M_DCACHE_LINESIZE)
#    undef CONFIG_KINETIS_LPUART_RXDMA_BUFFER_SIZE
#    define CONFIG_KINETIS_LPUART_RXDMA_BUFFER_SIZE ARMV7M_DCACHE_LINESIZE
#  endif

#  define RXDMA_BUFFER_MASK   ((uint32_t)(ARMV7M_DCACHE_LINESIZE - 1))
#  define RXDMA_BUFFER_SIZE   ((CONFIG_KINETIS_LPUART_RXDMA_BUFFER_SIZE \
                                + RXDMA_BUFFER_MASK) & ~RXDMA_BUFFER_MASK)

#endif /* LPSERIAL_HAVE_DMA */

#define LPUART_CTRL_ERROR_INTS  (LPUART_CTRL_ORIE | LPUART_CTRL_FEIE | \
                                 LPUART_CTRL_NEIE | LPUART_CTRL_PEIE)

#define LPUART_CTRL_RX_INTS     LPUART_CTRL_RIE

#define LPUART_CTRL_TX_INTS     LPUART_CTRL_TIE

#define LPUART_CTRL_ALL_INTS    (LPUART_CTRL_TX_INTS | LPUART_CTRL_RX_INTS | \
                                 LPUART_CTRL_MA1IE | LPUART_CTRL_MA1IE | \
                                 LPUART_CTRL_ILIE  | LPUART_CTRL_TCIE)

#define LPUART_STAT_ERRORS      (LPUART_STAT_OR | LPUART_STAT_FE | \
                                 LPUART_STAT_PF | LPUART_STAT_NF)

/* The LPUART does not have an common set of aligned bits for the interrupt
 * enable and the status. So map the ctrl to the stat bits
 */

#define LPUART_CTRL_TR_INTS     (LPUART_CTRL_TX_INTS | LPUART_CTRL_RX_INTS)
#define LPUART_CTRL2STAT(c)     ((((c) & LPUART_CTRL_ERROR_INTS) >> 8) | \
                                 ((c) & (LPUART_CTRL_TR_INTS)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kinetis_dev_s
{
  uintptr_t uartbase;  /* Base address of LPUART registers */
  uint32_t  baud;      /* Configured baud */
  uint32_t  clock;     /* Clocking frequency of the LPUART module */
  uint32_t  ie;        /* Interrupts enabled */
  uint8_t   irq;       /* IRQ associated with this LPUART (for enable) */
  uint8_t   parity;    /* 0=none, 1=odd, 2=even */
  uint8_t   bits;      /* Number of bits (8 or 9) */
  uint8_t   stop2;     /* Use 2 stop bits */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool      iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool      oflow;     /* output flow control (CTS) enabled */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint32_t  rts_gpio;  /* UART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint32_t  cts_gpio;  /* UART CTS GPIO pin configuration */
#endif
#ifdef LPSERIAL_HAVE_DMA
  const uint8_t rxdma_reqsrc;
  DMACH_HANDLE      rxdma;     /* currently-open receive DMA stream */
  uint32_t          rxdmanext; /* Next byte in the DMA buffer to be read */
  char      *const  rxfifo;    /* Receive DMA buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  kinetis_setup(struct uart_dev_s *dev);
static void kinetis_shutdown(struct uart_dev_s *dev);
static int  kinetis_attach(struct uart_dev_s *dev);
static void kinetis_detach(struct uart_dev_s *dev);
static int  kinetis_interrupt(int irq, void *context, void *arg);
static int  kinetis_ioctl(struct file *filep, int cmd, unsigned long arg);
static void kinetis_rxint(struct uart_dev_s *dev, bool enable);
#if !defined(LPSERIAL_HAVE_ALL_DMA)
static int  kinetis_receive(struct uart_dev_s *dev, unsigned int *status);
static bool kinetis_rxavailable(struct uart_dev_s *dev);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool kinetis_rxflowcontrol(struct uart_dev_s *dev,
                                  unsigned int nbuffered, bool upper);
#endif
static void kinetis_send(struct uart_dev_s *dev, int ch);
static void kinetis_txint(struct uart_dev_s *dev, bool enable);
static bool kinetis_txready(struct uart_dev_s *dev);

#ifdef LPSERIAL_HAVE_DMA
static int  kinetis_dma_nextrx(struct kinetis_dev_s *priv);
static int  kinetis_dma_setup(struct uart_dev_s *dev);
static void kinetis_dma_shutdown(struct uart_dev_s *dev);
static int  kinetis_dma_receive(struct uart_dev_s *dev,
                                unsigned int *status);
static bool kinetis_dma_rxavailable(struct uart_dev_s *dev);
static uint32_t get_and_clear_uart_status(struct kinetis_dev_s *priv);
static void kinetis_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
#if !defined(LPSERIAL_HAVE_ALL_DMA)
static const struct uart_ops_s g_lpuart_ops =
{
  .setup          = kinetis_setup,
  .shutdown       = kinetis_shutdown,
  .attach         = kinetis_attach,
  .detach         = kinetis_detach,
  .ioctl          = kinetis_ioctl,
  .receive        = kinetis_receive,
  .rxint          = kinetis_rxint,
  .rxavailable    = kinetis_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = kinetis_rxflowcontrol,
#endif
  .send           = kinetis_send,
  .txint          = kinetis_txint,
  .txready        = kinetis_txready,
  .txempty        = kinetis_txready,
};
#endif

#ifdef LPSERIAL_HAVE_DMA
static const struct uart_ops_s g_lpuart_dma_ops =
{
  .setup          = kinetis_dma_setup,
  .shutdown       = kinetis_dma_shutdown,
  .attach         = kinetis_attach,
  .detach         = kinetis_detach,
  .ioctl          = kinetis_ioctl,
  .receive        = kinetis_dma_receive,
  .rxint          = kinetis_rxint,
  .rxavailable    = kinetis_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = kinetis_rxflowcontrol,
#endif
  .send           = kinetis_send,
  .txint          = kinetis_txint,
  .txready        = kinetis_txready,
  .txempty        = kinetis_txready,
};
#endif

/* I/O buffers */

#ifdef CONFIG_KINETIS_LPUART0
static char g_lpuart0rxbuffer[CONFIG_LPUART0_RXBUFSIZE];
static char g_lpuart0txbuffer[CONFIG_LPUART0_TXBUFSIZE];
#  ifdef CONFIG_KINETIS_LPUART0_RXDMA
static char g_lpuart0rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#  endif
#endif
#ifdef CONFIG_KINETIS_LPUART1
static char g_lpuart1rxbuffer[CONFIG_LPUART1_RXBUFSIZE];
static char g_lpuart1txbuffer[CONFIG_LPUART1_TXBUFSIZE];
#  ifdef CONFIG_KINETIS_LPUART1_RXDMA
static char g_lpuart1rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#  endif
#endif
#ifdef CONFIG_KINETIS_LPUART2
static char g_lpuart2rxbuffer[CONFIG_LPUART2_RXBUFSIZE];
static char g_lpuart2txbuffer[CONFIG_LPUART2_TXBUFSIZE];
#  ifdef CONFIG_KINETIS_LPUART2_RXDMA
static char g_lpuart2rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#  endif
#endif
#ifdef CONFIG_KINETIS_LPUART3
static char g_lpuart3rxbuffer[CONFIG_LPUART3_RXBUFSIZE];
static char g_lpuart3txbuffer[CONFIG_LPUART3_TXBUFSIZE];
#  ifdef CONFIG_KINETIS_LPUART3_RXDMA
static char g_lpuart3rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#  endif
#endif
#ifdef CONFIG_KINETIS_LPUART4
static char g_lpuart4rxbuffer[CONFIG_LPUART4_RXBUFSIZE];
static char g_lpuart4txbuffer[CONFIG_LPUART4_TXBUFSIZE];
#  ifdef CONFIG_KINETIS_LPUART4_RXDMA
static char g_lpuart4rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
#  endif
#endif

/* This describes the state of the Kinetis LPUART0 port. */

#ifdef CONFIG_KINETIS_LPUART0
static struct kinetis_dev_s g_lpuart0priv =
{
  .uartbase       = KINETIS_LPUART0_BASE,
  .clock          = BOARD_LPUART0_FREQ,
  .baud           = CONFIG_LPUART0_BAUD,
  .irq            = KINETIS_IRQ_LPUART0,
  .parity         = CONFIG_LPUART0_PARITY,
  .bits           = CONFIG_LPUART0_BITS,
  .stop2          = CONFIG_LPUART0_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART0_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = PIN_LPUART0_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART0_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = PIN_LPUART0_RTS,
#endif
#ifdef CONFIG_KINETIS_LPUART0_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_LPUART0_RX,
  .rxfifo         = g_lpuart0rxfifo,
#endif
};

static uart_dev_t g_lpuart0port =
{
  .recv     =
  {
    .size   = CONFIG_LPUART0_RXBUFSIZE,
    .buffer = g_lpuart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_LPUART0_TXBUFSIZE,
    .buffer = g_lpuart0txbuffer,
  },
#ifdef CONFIG_KINETIS_LPUART0_RXDMA
  .ops        = &g_lpuart_dma_ops,
#else
  .ops        = &g_lpuart_ops,
#endif
  .priv     = &g_lpuart0priv,
};
#endif

/* This describes the state of the Kinetis LPUART1 port. */

#ifdef CONFIG_KINETIS_LPUART1
static struct kinetis_dev_s g_lpuart1priv =
{
  .uartbase       = KINETIS_LPUART1_BASE,
  .clock          = BOARD_CORECLK_FREQ,
  .baud           = BOARD_LPUART1_FREQ,
  .irq            = KINETIS_IRQ_LPUART1,
  .parity         = CONFIG_LPUART1_PARITY,
  .bits           = CONFIG_LPUART1_BITS,
  .stop2          = CONFIG_LPUART1_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART1_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = PIN_LPUART1_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = PIN_LPUART1_RTS,
#endif
#ifdef CONFIG_KINETIS_LPUART1_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_LPUART1_RX,
  .rxfifo         = g_lpuart1rxfifo,
#endif
};

static uart_dev_t g_lpuart1port =
{
  .recv     =
  {
    .size   = CONFIG_LPUART1_RXBUFSIZE,
    .buffer = g_lpuart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_LPUART1_TXBUFSIZE,
    .buffer = g_lpuart1txbuffer,
  },
#ifdef CONFIG_KINETIS_LPUART1_RXDMA
  .ops        = &g_lpuart_dma_ops,
#else
  .ops        = &g_lpuart_ops,
#endif
  .priv     = &g_lpuart1priv,
};
#endif

/* This describes the state of the Kinetis LPUART2 port. */

#ifdef CONFIG_KINETIS_LPUART2
static struct kinetis_dev_s g_lpuart2priv =
{
  .uartbase       = KINETIS_LPUART2_BASE,
  .clock          = BOARD_CORECLK_FREQ,
  .baud           = BOARD_LPUART2_FREQ,
  .irq            = KINETIS_IRQ_LPUART2,
  .parity         = CONFIG_LPUART2_PARITY,
  .bits           = CONFIG_LPUART2_BITS,
  .stop2          = CONFIG_LPUART2_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART2_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = PIN_LPUART2_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = PIN_LPUART2_RTS,
#endif
#ifdef CONFIG_KINETIS_LPUART2_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_LPUART2_RX,
  .rxfifo         = g_lpuart2rxfifo,
#endif
};

static uart_dev_t g_lpuart2port =
{
  .recv     =
  {
    .size   = CONFIG_LPUART2_RXBUFSIZE,
    .buffer = g_lpuart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_LPUART2_TXBUFSIZE,
    .buffer = g_lpuart2txbuffer,
  },
#ifdef CONFIG_KINETIS_LPUART2_RXDMA
  .ops        = &g_lpuart_dma_ops,
#else
  .ops        = &g_lpuart_ops,
#endif
  .priv     = &g_lpuart2priv,
};
#endif

/* This describes the state of the Kinetis LPUART3 port. */

#ifdef CONFIG_KINETIS_LPUART3
static struct kinetis_dev_s g_lpuart3priv =
{
  .uartbase       = KINETIS_LPUART3_BASE,
  .clock          = BOARD_CORECLK_FREQ,
  .baud           = BOARD_LPUART3_FREQ,
  .irq            = KINETIS_IRQ_LPUART3,
  .parity         = CONFIG_LPUART3_PARITY,
  .bits           = CONFIG_LPUART3_BITS,
  .stop2          = CONFIG_LPUART3_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART3_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = PIN_LPUART3_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART3_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = PIN_LPUART3_RTS,
#endif
#ifdef CONFIG_KINETIS_LPUART3_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_LPUART3_RX,
  .rxfifo         = g_lpuart3rxfifo,
#endif
};

static uart_dev_t g_lpuart3port =
{
  .recv     =
  {
    .size   = CONFIG_LPUART3_RXBUFSIZE,
    .buffer = g_lpuart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_LPUART3_TXBUFSIZE,
    .buffer = g_lpuart3txbuffer,
  },
#ifdef CONFIG_KINETIS_LPUART3_RXDMA
  .ops        = &g_lpuart_dma_ops,
#else
  .ops        = &g_lpuart_ops,
#endif
  .priv     = &g_lpuart3priv,
};
#endif

/* This describes the state of the Kinetis LPUART4 port. */

#ifdef CONFIG_KINETIS_LPUART4
static struct kinetis_dev_s g_lpuart4priv =
{
  .uartbase       = KINETIS_LPUART4_BASE,
  .clock          = BOARD_CORECLK_FREQ,
  .baud           = BOARD_LPUART4_FREQ,
  .irq            = KINETIS_IRQ_LPUART4,
  .parity         = CONFIG_LPUART4_PARITY,
  .bits           = CONFIG_LPUART4_BITS,
  .stop2          = CONFIG_LPUART4_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART4_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = PIN_LPUART4_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART4_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = PIN_LPUART4_RTS,
#endif
#ifdef CONFIG_KINETIS_LPUART4_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_LPUART4_RX,
  .rxfifo         = g_lpuart4rxfifo,
#endif
};

static uart_dev_t g_lpuart4port =
{
  .recv     =
  {
    .size   = CONFIG_LPUART4_RXBUFSIZE,
    .buffer = g_lpuart4rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_LPUART4_TXBUFSIZE,
    .buffer = g_lpuart4txbuffer,
  },
#ifdef CONFIG_KINETIS_LPUART4_RXDMA
  .ops        = &g_lpuart_dma_ops,
#else
  .ops        = &g_lpuart_ops,
#endif
  .priv     = &g_lpuart4priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_serialin
 ****************************************************************************/

static inline uint32_t kinetis_serialin(struct kinetis_dev_s *priv,
                                        int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: kinetis_serialout
 ****************************************************************************/

static inline void kinetis_serialout(struct kinetis_dev_s *priv, int offset,
                                     uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: kinetis_setuartint
 ****************************************************************************/

static void kinetis_setuartint(struct kinetis_dev_s *priv)
{
  irqstate_t flags;
  uint32_t regval;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in
   * ie
   */

  flags    = enter_critical_section();
  regval   = kinetis_serialin(priv, KINETIS_LPUART_CTRL_OFFSET);
  regval  &= ~LPUART_CTRL_ALL_INTS;
  regval  |= priv->ie;
  kinetis_serialout(priv, KINETIS_LPUART_CTRL_OFFSET, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: kinetis_restoreuartint
 ****************************************************************************/

static void kinetis_restoreuartint(struct kinetis_dev_s *priv, uint32_t ie)
{
  irqstate_t flags;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in
   * ie
   */

  flags    = enter_critical_section();
  priv->ie = ie & LPUART_CTRL_ALL_INTS;
  kinetis_setuartint(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: kinetis_disableuartint
 ****************************************************************************/

#if defined(HAVE_LPUART_PUTC) && defined(HAVE_LPUART_CONSOLE)
static void kinetis_disableuartint(struct kinetis_dev_s *priv, uint32_t *ie)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (ie)
    {
      *ie = priv->ie;
    }

  kinetis_restoreuartint(priv, 0);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: get_and_clear_uart_status
 *
 * Description:
 *   Clears the error flags of the uart if an error occurred in s1 and
 *   returns the status
 *
 * Input Parameters:
 *   u_dev_s
 *
 * Returns Value:
 *   Uart status s1
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
static uint32_t get_and_clear_uart_status(struct kinetis_dev_s *priv)
{
  uint32_t regval;

  regval = kinetis_serialin(priv, KINETIS_LPUART_STAT_OFFSET);
  regval &= LPUART_STAT_ERRORS;

  if (regval != 0)
    {
      kinetis_serialout(priv, KINETIS_LPUART_STAT_OFFSET, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: kinetis_setup
 *
 * Description:
 *   Configure the LPUART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int kinetis_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_LPUART_CONFIG
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool iflow = priv->iflow;
#else
  bool iflow = false;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool oflow = priv->oflow;
#else
  bool oflow = false;
#endif

  /* Configure the LPUART as an RS-232 UART */

  kinetis_lpuartconfigure(priv->uartbase, priv->baud, priv->clock,
                        priv->parity, priv->bits, priv->stop2,
                        iflow, oflow);
#endif

  /* Make sure that all interrupts are disabled */

  kinetis_restoreuartint(priv, 0);
  return OK;
}

/****************************************************************************
 * Name: kinetis_dma_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
static int kinetis_dma_setup(struct uart_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  int result;
  uint32_t regval;
  DMACH_HANDLE rxdma = NULL;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = kinetis_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

  /* Acquire the DMA channel. */

  rxdma = kinetis_dmach_alloc(priv->rxdma_reqsrc | DMAMUX_CHCFG_ENBL, 0);
  if (rxdma == NULL)
    {
      return -EBUSY;
    }

  /* Configure for circular DMA reception into the RX FIFO */

  struct kinetis_edma_xfrconfig_s config;
  config.saddr  = priv->uartbase + KINETIS_LPUART_DATA_OFFSET;
  config.daddr  = (uint32_t) priv->rxfifo;
  config.soff   = 0;
  config.doff   = 1;
  config.iter   = RXDMA_BUFFER_SIZE;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE | EDMA_CONFIG_LOOPDEST;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = 1;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = NULL;
#endif
  kinetis_dmach_xfrsetup(rxdma, &config);

  /* Reset our DMA shadow pointer to match the address just programmed
   * above.
   */

  priv->rxdmanext = 0;

  up_invalidate_dcache((uintptr_t)priv->rxfifo,
                       (uintptr_t)priv->rxfifo + RXDMA_BUFFER_SIZE);

  /* Enable receive DMA for the UART */

  regval  = kinetis_serialin(priv, KINETIS_LPUART_BAUD_OFFSET);
  regval |= LPUART_BAUD_RDMAE;
  kinetis_serialout(priv, KINETIS_LPUART_BAUD_OFFSET, regval);

  /* Start the DMA channel, and arrange for callbacks at the half and
   * full points in the FIFO.  This ensures that we have half a FIFO
   * worth of time to claim bytes before they are overwritten.
   */

  kinetis_dmach_start(rxdma, kinetis_dma_rxcallback, (void *)dev);
  priv->rxdma = rxdma;
  return OK;
}
#endif

/****************************************************************************
 * Name: kinetis_shutdown
 *
 * Description:
 *   Disable the LPUART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void kinetis_shutdown(struct uart_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;

  /* Disable interrupts */

  kinetis_restoreuartint(priv, 0);

  /* Reset hardware and disable Rx and Tx */

  kinetis_lpuartreset(priv->uartbase);
}

/****************************************************************************
 * Name: kinetis_dma_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
static void kinetis_dma_shutdown(struct uart_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  DMACH_HANDLE rxdma = priv->rxdma;

  /* Perform the normal UART shutdown */

  kinetis_shutdown(dev);

  /* Stop the DMA channel */

  kinetis_dmach_stop(rxdma);

  /* Release the DMA channel */

  kinetis_dmach_free(rxdma);

  priv->rxdma = NULL;
}
#endif

/****************************************************************************
 * Name: kinetis_attach
 *
 * Description:
 *   Configure the LPUART to operation in interrupt driven mode.  This
 *   method is called when the serial port is opened.  Normally, this is
 *   just after the setup() method is called, however, the serial
 *   console may operate in a non-interrupt driven mode during the boot
 *   phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int kinetis_attach(struct uart_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the LPUART_CTRL register.
   */

  ret = irq_attach(priv->irq, kinetis_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: kinetis_detach
 *
 * Description:
 *   Detach LPUART interrupts.  This method is called when the serial port
 *   is closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void kinetis_detach(struct uart_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;

  /* Disable interrupts */

  kinetis_restoreuartint(priv, 0);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: kinetis_interrupts
 *
 * Description:
 *   This is the LPUART status interrupt handler.  It will be invoked when
 *   an interrupt received on the 'irq'  It should call uart_transmitchars
 *   or uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   Appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int kinetis_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct kinetis_dev_s *priv;
  uint32_t stat;
  uint32_t ctrl;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct kinetis_dev_s *)dev->priv;

  /* Read status register and qualify it with STAT bit corresponding CTRL IE
   * bits
   */

  stat = kinetis_serialin(priv, KINETIS_LPUART_STAT_OFFSET);
  ctrl = kinetis_serialin(priv, KINETIS_LPUART_CTRL_OFFSET);
  stat &= LPUART_CTRL2STAT(ctrl);
  do
    {
      /* Handle errors.  This interrupt may be caused by:
       *
       * OR: Receiver Overrun Flag. To clear OR, when STAT read with OR set,
       *     write STAT with OR set;
       * FE: Framing error. To clear FE, when STAT read with FE set, read the
       *     data to discard it and write STAT with FE set;
       * NF: Noise flag. To clear NF, when STAT read with EE set, read the
       *     data to discard it and write STAT with NE set;
       * PF: Parity error flag. To clear PF, when STAT read with PE set, read
       *     the data to discard it and write STAT with PE set;
       */

      if (stat & LPUART_STAT_ERRORS)
        {
          /* Only Overrun error does not need a read operation */

          if ((stat & LPUART_STAT_OR) != LPUART_STAT_OR)
            {
              kinetis_serialin(priv, KINETIS_LPUART_DATA_OFFSET);
            }

          /* Reset any Errors */

          kinetis_serialout(priv, KINETIS_LPUART_STAT_OFFSET,
                            stat & LPUART_STAT_ERRORS);
          return OK;
        }

      /* Handle incoming, receive bytes
       *
       * Check if the receive data register is full (RDRF).
       *
       * The RDRF status indication is cleared when the data is read from
       * the RX data register.
       */

      if (stat & LPUART_STAT_RDRF)
        {
          uart_recvchars(dev);
        }

      /* Handle outgoing, transmit bytes
       *
       * Check if the transmit data register is "empty."
       *
       * The TDRE status indication is cleared when the data is written to
       * the TX data register.
       */

      if (stat & LPUART_STAT_TDRE)
        {
          uart_xmitchars(dev);
        }

      /* Read status register and requalify it with STAT bit corresponding
       * CTRL IE bits
       */

      stat = kinetis_serialin(priv, KINETIS_LPUART_STAT_OFFSET);
      ctrl = kinetis_serialin(priv, KINETIS_LPUART_CTRL_OFFSET);
      stat &= LPUART_CTRL2STAT(ctrl);
    }
  while (stat != 0);

  return OK;
}

/****************************************************************************
 * Name: kinetis_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int kinetis_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || \
    defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  struct inode           *inode;
  struct uart_dev_s      *dev;
  uint32_t regval;
#endif
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  struct kinetis_dev_s   *priv;
  bool                   iflow = false;
  bool                   oflow = false;
#endif
  int                    ret = OK;

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || \
    defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->priv != NULL);
#endif

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  priv  = (struct kinetis_dev_s *)dev->priv;
#endif

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct kinetis_dev_s *user = (struct kinetis_dev_s *)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct kinetis_dev_s));
          }
      }
      break;
#endif

#ifdef CONFIG_KINETIS_UART_SINGLEWIRE
    case TIOCSSINGLEWIRE:
      {
        if ((arg & SER_SINGLEWIRE_PULLUP) != 0)
          {
            ret = -EINVAL; /* Not supported */
            break;
          }

        /* Change to single-wire operation. the RXD pin is disconnected from
         * the UART and the UART implements a half-duplex serial connection.
         * The UART uses the TXD pin for both receiving and transmitting
         */

        regval = kinetis_serialin(priv, KINETIS_LPUART_CTRL_OFFSET);

        if ((arg & SER_SINGLEWIRE_ENABLED) != 0)
          {
            regval |= (LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC);
          }
        else
          {
            regval &= ~(LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC);
          }

        kinetis_serialout(priv, KINETIS_LPUART_CTRL_OFFSET, regval);
      }
     break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        cfsetispeed(termiosp, priv->baud);

        /* Note: CSIZE only supports 5-8 bits. The driver only support 8/9
         * bit modes and therefore is no way to report 9-bit mode, we always
         * claim 8 bit mode.
         */

        termiosp->c_cflag =
          ((priv->parity != 0) ? PARENB : 0) |
          ((priv->parity == 1) ? PARODD : 0) |
          ((priv->stop2) ? CSTOPB : 0) |
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
          ((priv->oflow) ? CCTS_OFLOW : 0) |
#  endif
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
          ((priv->iflow) ? CRTS_IFLOW : 0) |
#  endif
          CS8;

        cfsetispeed(termiosp, priv->baud);

        /* TODO: CRTS_IFLOW, CCTS_OFLOW */
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

        if (((termiosp->c_cflag & CSIZE) != CS8)
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#  endif
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#  endif
           )
          {
            ret = -EINVAL;
            break;
          }

        if (termiosp->c_cflag & PARENB)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        priv->stop2 = (termiosp->c_cflag & CSTOPB) != 0;
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
        priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
        oflow = priv->oflow;
#  endif
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
        priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
        iflow = priv->iflow;
#  endif

        /* Note that since there is no way to request 9-bit mode
         * and no way to support 5/6/7-bit modes, we ignore them
         * all here.
         */

        /* Note that only cfgetispeed is used because we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);

        /* Effect the changes immediately - note that we do not implement
         * TCSADRAIN / TCSAFLUSH
         */

        kinetis_lpuartconfigure(priv->uartbase, priv->baud, priv->clock,
                                priv->parity, priv->bits, priv->stop2,
                                iflow, oflow);
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_KINETIS_UART_BREAKS
    case TIOCSBRK:
      {
        irqstate_t flags;

        flags = enter_critical_section();

        /* Send a longer break signal */

        regval = kinetis_serialin(priv, KINETIS_LPUART_STAT_OFFSET);
        regval &= ~LPUART_STAT_BRK13;
# ifdef CONFIG_KINETIS_UART_EXTEDED_BREAK
        regval |= LPUART_STAT_BRK13;
#  endif
        kinetis_serialout(priv, LPUART_STAT_BRK13, regval);

        /* Send a break signal */

        regval = kinetis_serialin(priv, KINETIS_LPUART_CTRL_OFFSET);
        regval |= LPUART_CTRL_SBK;
        kinetis_serialout(priv, KINETIS_LPUART_CTRL_OFFSET, regval);

#  ifdef CONFIG_KINETIS_SERIALBRK_BSDCOMPAT
        /* BSD compatibility: Turn break on, and leave it on */

        kinetis_txint(dev, false);
#  else
        /* Send a single break character
         * Toggling SBK sends one break character. Per the manual
         * Toggling implies clearing the SBK field before the break
         * character has finished transmitting.
         */

        regval &= ~LPUART_CTRL_SBK;
        kinetis_serialout(priv, KINETIS_LPUART_CTRL_OFFSET, regval);
#endif

        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:
      {
        irqstate_t flags;

        flags = enter_critical_section();

        /* Configure TX back to UART
         * If non BSD compatible: This code has no effect, the SBRK
         * was already cleared.
         * but for BSD compatibility: Turn break off
         */

        regval = kinetis_serialin(priv, KINETIS_LPUART_CTRL_OFFSET);
        regval &= ~LPUART_CTRL_SBK;
        kinetis_serialout(priv, KINETIS_LPUART_CTRL_OFFSET, regval);

#  ifdef CONFIG_KINETIS_SERIALBRK_BSDCOMPAT
        /* Enable further tx activity */

        kinetis_txint(dev, true);
#  endif
        leave_critical_section(flags);
      }
      break;
#endif /* CONFIG_KINETIS_UART_BREAKS */

#ifdef CONFIG_KINETIS_UART_INVERT
    case TIOCSINVERT:
      {
        uint32_t stat;
        uint32_t ctrl;
        irqstate_t flags;

        flags = enter_critical_section();

        stat = kinetis_serialin(priv, KINETIS_LPUART_STAT_OFFSET);
        ctrl = kinetis_serialin(priv, KINETIS_LPUART_CTRL_OFFSET);

        /* {R|T}XINV bit fields can written any time */

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

        kinetis_serialout(priv, KINETIS_LPUART_STAT_OFFSET, stat);
        kinetis_serialout(priv, KINETIS_LPUART_CTRL_OFFSET, ctrl);

        leave_critical_section(flags);
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
 * Name: kinetis_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character from
 *   the LPUART.  Error bits associated with the receipt are provided in the
 *   return 'status'.
 *
 ****************************************************************************/
#if !defined(LPSERIAL_HAVE_ALL_DMA)
static int kinetis_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  uint32_t regval;
  int data;

  /* Get error status information:
   *
   * OR: Receiver Overrun Flag. To clear OR, when STAT read with OR set,
   *     write STAT with OR set;
   * FE: Framing error. To clear FE, when STAT read with FE set, read the
   *     data to discard it and write STAT with FE set;
   * NF: Noise flag. To clear NF, when STAT read with EE set, read the
   *     data to discard it and write STAT with NE set;
   * PF: Parity error flag. To clear PF, when STAT read with PE set, read
   *     the data to discard it and write STAT with PE set;
   */

  regval = kinetis_serialin(priv, KINETIS_LPUART_STAT_OFFSET);

  /* Return status information */

  if (status)
    {
      *status = regval;
    }

  /* Then return the actual received byte.  Read DATA. Then if
   * there were any errors write 1 to them to clear the RX errors.
   */

  data = (int)kinetis_serialin(priv, KINETIS_LPUART_DATA_OFFSET);
  regval &= LPUART_STAT_ERRORS;
  if (regval)
    {
      kinetis_serialout(priv, KINETIS_LPUART_STAT_OFFSET, regval);
    }

  return data;
}
#endif

/****************************************************************************
 * Name: kinetis_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
static int kinetis_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  int c = 0;
  uint32_t stat;

  /* Clear uart errors and return status information */

  stat = get_and_clear_uart_status(priv);
  if (status)
    {
      *status = stat;
    }

  if (kinetis_dma_nextrx(priv) != priv->rxdmanext)
    {
      /* Invalidate the DMA buffer */

      up_invalidate_dcache((uintptr_t)priv->rxfifo,
                           (uintptr_t)priv->rxfifo + RXDMA_BUFFER_SIZE);

      /* Now read from the DMA buffer */

      c = priv->rxfifo[priv->rxdmanext];
      priv->rxdmanext++;
      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
          /* HACK: Skip the first byte since it is duplicate of last one. */

          if (kinetis_dma_nextrx(priv) != 0)
            {
              priv->rxdmanext = 1;
            }
          else
            {
              /* Try to catch race conditions that will spin on the whole
               * buffer again.
               */

              priv->rxdmanext = 0;
            }
        }
    }

  return c;
}
#endif

/****************************************************************************
 * Name: kinetis_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void kinetis_rxint(struct uart_dev_s *dev, bool enable)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx related error occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= (LPUART_CTRL_RX_INTS | LPUART_CTRL_ERROR_INTS);
      kinetis_setuartint(priv);
#endif
    }
  else
    {
      priv->ie &= ~(LPUART_CTRL_RX_INTS | LPUART_CTRL_ERROR_INTS);
      kinetis_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: kinetis_dma_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
static bool kinetis_dma_rxavailable(struct uart_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (kinetis_dma_nextrx(priv) != priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: kinetis_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/
#if !defined(LPSERIAL_HAVE_ALL_DMA)
static bool kinetis_rxavailable(struct uart_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;

  /* Return true if the receive data register is full (RDRF). */

  return (kinetis_serialin(priv, KINETIS_LPUART_STAT_OFFSET) &
          LPUART_STAT_RDRF) != 0;
}
#endif

/****************************************************************************
 * Name: kinetis_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input Parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool kinetis_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS)
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  uint16_t ie;

  if (priv->iflow)
    {
      /* Is the RX buffer full? */

      if (upper)
        {
          /* Disable Rx interrupt to prevent more data being from
           * peripheral.  When hardware RTS is enabled, this will
           * prevent more data from coming in.
           *
           * This function is only called when UART recv buffer is full,
           * that is: "dev->recv.head + 1 == dev->recv.tail".
           *
           * Logic in "uart_read" will automatically toggle Rx interrupts
           * when buffer is read empty and thus we do not have to re-
           * enable Rx interrupts.
           */

          ie = priv->ie;
          ie &= ~LPUART_CTRL_RX_INTS;
          kinetis_restoreuartint(priv, ie);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input
           * is received.
           */

          kinetis_rxint(dev, true);
        }
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: kinetis_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
static int kinetis_dma_nextrx(struct kinetis_dev_s *priv)
{
  size_t dmaresidual;

  dmaresidual = kinetis_dmach_getcount(priv->rxdma);

  return (RXDMA_BUFFER_SIZE - (int)dmaresidual) % RXDMA_BUFFER_SIZE;
}
#endif

/****************************************************************************
 * Name: kinetis_send
 *
 * Description:
 *   This method will send one byte on the LPUART.
 *
 ****************************************************************************/

static void kinetis_send(struct uart_dev_s *dev, int ch)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  kinetis_serialout(priv, KINETIS_LPUART_DATA_OFFSET, ch);
}

/****************************************************************************
 * Name: kinetis_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void kinetis_txint(struct uart_dev_s *dev, bool enable)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_TX_INTS;
      kinetis_setuartint(priv);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->ie &= ~LPUART_CTRL_TX_INTS;
      kinetis_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: kinetis_txready
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool kinetis_txready(struct uart_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev->priv;

  /* Return true if the transmit data register is "empty." */

  return (kinetis_serialin(priv, KINETIS_LPUART_STAT_OFFSET) &
          LPUART_STAT_TDRE) != 0;
}

/****************************************************************************
 * Name: kinetis_dma_rxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
static void kinetis_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;

  if (kinetis_dma_rxavailable(dev))
    {
      uart_recvchars(dev);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_lpuart_earlyserialinit
 *
 * Description:
 *   Performs the low level LPUART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in kinetis_lowsetup() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void kinetis_lpuart_earlyserialinit(void)
{
  /* Disable interrupts from all LPUARTS.  The console is enabled in
   * kinetis_setup()
   */

  kinetis_restoreuartint(TTYS0_DEV.priv, 0);
#ifdef TTYS1_DEV
  kinetis_restoreuartint(TTYS1_DEV.priv, 0);
#endif
#ifdef TTYS2_DEV
  kinetis_restoreuartint(TTYS2_DEV.priv, 0);
#endif
#ifdef TTYS3_DEV
  kinetis_restoreuartint(TTYS3_DEV.priv, 0);
#endif
#ifdef TTYS4_DEV
  kinetis_restoreuartint(TTYS4_DEV.priv, 0);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_LPUART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  kinetis_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: kinetis_lpuart_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   first: - First TTY number to assign
 *
 * Returned Value:
 *   The next TTY number available for assignment
 *
 ****************************************************************************/

unsigned int kinetis_lpuart_serialinit(unsigned int first)
{
#if defined(CONFIG_KINETIS_MERGE_TTY)
  char devname[] = "/dev/ttySx";
#endif

  /* Register the console */

#ifdef HAVE_LPUART_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#  ifdef SERIAL_HAVE_CONSOLE_DMA
  /* If we need to re-initialise the console to enable DMA do that here. */

  kinetis_dma_setup(&CONSOLE_DEV);
#  endif
#endif
#if !defined(CONFIG_KINETIS_MERGE_TTY)
  /* Register all LPUARTs as LPn devices */

  uart_register("/dev/ttyLP0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyLP1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyLP2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyLP3", &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  uart_register("/dev/ttyLP4", &TTYS4_DEV);
#endif

#else

  devname[(sizeof(devname) / sizeof(devname[0])) - 2] = '0' + first++;
  uart_register(devname, &TTYS0_DEV);
#ifdef TTYS1_DEV
  devname[(sizeof(devname) / sizeof(devname[0])) - 2] = '0' + first++;
  uart_register(devname, &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  devname[(sizeof(devname) / sizeof(devname[0])) - 2] = '0' + first++;
  uart_register(devname, &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  devname[(sizeof(devname) / sizeof(devname[0])) - 2] = '0' + first++;
  uart_register(devname, &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  devname[(sizeof(devname) / sizeof(devname[0])) - 2] = '0' + first++;
  uart_register(devname, &TTYS4_DEV);
#endif
#endif

  return first;
}

/****************************************************************************
 * Name: kinetis_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 ****************************************************************************/

#ifdef LPSERIAL_HAVE_DMA
void kinetis_lpserial_dma_poll(void)
{
    irqstate_t flags;

    flags = enter_critical_section();

#ifdef CONFIG_KINETIS_LPUART0_RXDMA
  if (g_lpuart0priv.rxdma != NULL)
    {
      kinetis_dma_rxcallback(g_lpuart0priv.rxdma, (void *)&g_lpuart0port,
                             false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_LPUART1_RXDMA
  if (g_lpuart1priv.rxdma != NULL)
    {
      kinetis_dma_rxcallback(g_lpuart1priv.rxdma, (void *)&g_lpuart1port,
                             false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_LPUART2_RXDMA
  if (g_lpuart2priv.rxdma != NULL)
    {
      kinetis_dma_rxcallback(g_lpuart2priv.rxdma, (void *)&g_lpuart2port,
                             false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_LPUART3_RXDMA
  if (g_lpuart3priv.rxdma != NULL)
    {
      kinetis_dma_rxcallback(g_lpuart3priv.rxdma, (void *)&g_lpuart3port,
                             false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_LPUART4_RXDMA
  if (g_lpuart4priv.rxdma != NULL)
    {
      kinetis_dma_rxcallback(g_lpuart4priv.rxdma, (void *)&g_lpuart4port,
                             false, 0);
    }
#endif

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_PUTC
int up_putc(int ch)
{
#ifdef HAVE_LPUART_CONSOLE
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)CONSOLE_DEV.priv;
  uint32_t ie;

  kinetis_disableuartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  kinetis_restoreuartint(priv, ie);
#endif
  return ch;
}
#endif

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_PUTC
int up_putc(int ch)
{
#ifdef HAVE_LPUART_CONSOLE
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
#endif

#endif /* HAVE_LPUART_DEVICE && USE_SERIALDRIVER) */
