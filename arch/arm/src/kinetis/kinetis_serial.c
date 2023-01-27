/****************************************************************************
 * arch/arm/src/kinetis/kinetis_serial.c
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
#include "hardware/kinetis_dmamux.h"
#include "hardware/kinetis_uart.h"
#include "hardware/kinetis_pinmux.h"
#include "kinetis.h"
#include "kinetis_edma.h"
#include "kinetis_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks */

/* Is there at least one UART enabled and configured as a RS-232 device? */

#ifndef HAVE_UART_DEVICE
#  warning "No UARTs enabled"
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(HAVE_UART_DEVICE) && defined(USE_SERIALDRIVER)

/* Assume DMA is not used on the console UART */

#undef SERIAL_HAVE_CONSOLE_DMA

/* Which UART with be tty0/console and which tty1-4?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART0-5 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port /* UART0 is console */
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  if defined(CONFIG_KINETIS_UART0_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port /* UART1 is console */
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  if defined(CONFIG_KINETIS_UART1_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart2port /* UART2 is console */
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  if defined(CONFIG_KINETIS_UART2_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart3port /* UART3 is console */
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  if defined(CONFIG_KINETIS_UART3_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart4port /* UART4 is console */
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  if defined(CONFIG_KINETIS_UART4_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart5port /* UART5 is console */
#    define TTYS0_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#  if defined(CONFIG_KINETIS_UART5_RXDMA)
#    define SERIAL_HAVE_CONSOLE_DMA 1
#  endif
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_KINETIS_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_KINETIS_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_KINETIS_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(CONFIG_KINETIS_UART3)
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  elif defined(CONFIG_KINETIS_UART4)
#    define TTYS0_DEV           g_uart4port /* UART4 is ttyS0 */
#    define UART4_ASSIGNED      1
#  elif defined(CONFIG_KINETIS_UART5)
#    define TTYS0_DEV           g_uart5port /* UART5 is ttyS0 */
#    define UART5_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-5 excluding the console UART. */

#if defined(CONFIG_KINETIS_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS1_DEV           g_uart5port /* UART5 is ttyS1 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART1-5. It can't be UART0 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-5 could also be the
 * console.
 */

#if defined(CONFIG_KINETIS_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS2_DEV           g_uart5port /* UART5 is ttyS2 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART2-5. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART 2-5 could also be the console.
 */

#if defined(CONFIG_KINETIS_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS3_DEV           g_uart2port /* UART2 is ttyS3 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS3_DEV           g_uart5port /* UART5 is ttyS3 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys4. This could be one of UART3-5. It can't be UART0-2 because
 * those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * UART 3-5 could also be the console.
 */

#if defined(CONFIG_KINETIS_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS4_DEV           g_uart3port /* UART3 is ttyS4 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS4_DEV           g_uart5port /* UART5 is ttyS4 */
#  define UART5_ASSIGNED      1
#endif

/* Pick ttys5. This could be one of UART4-5. It can't be UART0-3 because
 * those have already been assigned to ttsyS0, 1, 2, 3 or 4.  One of
 * UART 4-5 could also be the console.
 */

#if defined(CONFIG_KINETIS_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS5_DEV           g_uart4port /* UART4 is ttyS5 */
#  define UART4_ASSIGNED      1
#elif defined(CONFIG_KINETIS_UART5) && !defined(UART5_ASSIGNED)
#  define TTYS5_DEV           g_uart5port /* UART5 is ttyS5 */
#  define UART5_ASSIGNED      1
#endif

#ifdef SERIAL_HAVE_DMA

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

#  if !defined(CONFIG_KINETIS_SERIAL_RXDMA_BUFFER_SIZE) || \
      (CONFIG_KINETIS_SERIAL_RXDMA_BUFFER_SIZE < ARMV7M_DCACHE_LINESIZE)
#    undef CONFIG_KINETIS_SERIAL_RXDMA_BUFFER_SIZE
#    define CONFIG_KINETIS_SERIAL_RXDMA_BUFFER_SIZE ARMV7M_DCACHE_LINESIZE
#  endif

#  define RXDMA_BUFFER_MASK   ((uint32_t)(ARMV7M_DCACHE_LINESIZE - 1))
#  define RXDMA_BUFFER_SIZE   ((CONFIG_KINETIS_SERIAL_RXDMA_BUFFER_SIZE \
                                + RXDMA_BUFFER_MASK) & ~RXDMA_BUFFER_MASK)

#endif /* SERIAL_HAVE_DMA */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase;  /* Base address of UART registers */
  uint32_t  baud;      /* Configured baud */
  uint32_t  clock;     /* Clocking frequency of the UART module */
#ifdef CONFIG_DEBUG_FEATURES
  uint8_t   irqe;      /* Error IRQ associated with this UART (for enable) */
#endif
  uint8_t   irqs;      /* Status IRQ associated with this UART (for enable) */
  uint8_t   ie;        /* Interrupts enabled */
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
#ifdef SERIAL_HAVE_DMA
  const uint8_t rxdma_reqsrc;
  DMACH_HANDLE      rxdma;     /* currently-open receive DMA stream */
  uint32_t          rxdmanext; /* Next byte in the DMA buffer to be read */
  char      *const  rxfifo;    /* Receive DMA buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
#ifdef CONFIG_DEBUG_FEATURES
static int  up_interrupt(int irq, void *context, void *arg);
#endif
static int  up_interrupts(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static void up_rxint(struct uart_dev_s *dev, bool enable);
#if !defined(SERIAL_HAVE_ALL_DMA)
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static bool up_rxavailable(struct uart_dev_s *dev);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
                             bool upper);
#endif
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
#ifdef CONFIG_KINETIS_UARTFIFOS
static bool up_txempty(struct uart_dev_s *dev);
#endif
#ifdef SERIAL_HAVE_DMA
static int  up_dma_nextrx(struct up_dev_s *priv);
static int  up_dma_setup(struct uart_dev_s *dev);
static void up_dma_shutdown(struct uart_dev_s *dev);
static int  up_dma_receive(struct uart_dev_s *dev, unsigned int *status);
static bool up_dma_rxavailable(struct uart_dev_s *dev);
static uint8_t get_and_clear_uart_status(struct up_dev_s *priv);
static void up_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if !defined(SERIAL_HAVE_ALL_DMA)
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
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
#ifdef CONFIG_KINETIS_UARTFIFOS
  .txempty        = up_txempty,
#else
  .txempty        = up_txready,
#endif
};
#endif

#ifdef SERIAL_HAVE_DMA
static const struct uart_ops_s g_uart_dma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_dma_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
#ifdef CONFIG_KINETIS_UARTFIFOS
  .txempty        = up_txempty,
#else
  .txempty        = up_txready,
#endif
};
#endif

/* I/O buffers */

#ifdef CONFIG_KINETIS_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
# ifdef CONFIG_KINETIS_UART0_RXDMA
static char g_uart0rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
# endif
#endif

#ifdef CONFIG_KINETIS_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
# ifdef CONFIG_KINETIS_UART1_RXDMA
static char g_uart1rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
# endif
#endif

#ifdef CONFIG_KINETIS_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
# ifdef CONFIG_KINETIS_UART2_RXDMA
static char g_uart2rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
# endif
#endif

#ifdef CONFIG_KINETIS_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
# ifdef CONFIG_KINETIS_UART3_RXDMA
static char g_uart3rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
# endif
#endif

#ifdef CONFIG_KINETIS_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
# ifdef CONFIG_KINETIS_UART4_RXDMA
static char g_uart4rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
# endif
#endif

#ifdef CONFIG_KINETIS_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
# ifdef CONFIG_KINETIS_UART5_RXDMA
static char g_uart5rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
# endif
#endif

/* This describes the state of the Kinetis UART0 port. */

#ifdef CONFIG_KINETIS_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = KINETIS_UART0_BASE,
  .clock          = BOARD_CORECLK_FREQ,
  .baud           = CONFIG_UART0_BAUD,
#ifdef CONFIG_DEBUG_FEATURES
  .irqe           = KINETIS_IRQ_UART0E,
#endif
  .irqs           = KINETIS_IRQ_UART0S,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .stop2          = CONFIG_UART0_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART0_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = PIN_UART0_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART0_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = PIN_UART0_RTS,
#endif
#ifdef CONFIG_KINETIS_UART0_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_UART0_RX,
  .rxfifo         = g_uart0rxfifo,
#endif
};

static uart_dev_t g_uart0port =
{
  .recv       =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },
#ifdef CONFIG_KINETIS_UART0_RXDMA
  .ops        = &g_uart_dma_ops,
#else
  .ops        = &g_uart_ops,
#endif
  .priv       = &g_uart0priv,
};
#endif

/* This describes the state of the Kinetis UART1 port. */

#ifdef CONFIG_KINETIS_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = KINETIS_UART1_BASE,
  .clock          = BOARD_CORECLK_FREQ,
  .baud           = CONFIG_UART1_BAUD,
#ifdef CONFIG_DEBUG_FEATURES
  .irqe           = KINETIS_IRQ_UART1E,
#endif
  .irqs           = KINETIS_IRQ_UART1S,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stop2          = CONFIG_UART1_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART1_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = PIN_UART1_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART1_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = PIN_UART1_RTS,
#endif
#ifdef CONFIG_KINETIS_UART1_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_UART1_RX,
  .rxfifo         = g_uart1rxfifo,
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
#ifdef CONFIG_KINETIS_UART1_RXDMA
  .ops        = &g_uart_dma_ops,
#else
  .ops        = &g_uart_ops,
#endif
  .priv       = &g_uart1priv,
};
#endif

/* This describes the state of the Kinetis UART2 port. */

#ifdef CONFIG_KINETIS_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = KINETIS_UART2_BASE,
  .clock          = BOARD_BUS_FREQ,
  .baud           = CONFIG_UART2_BAUD,
#ifdef CONFIG_DEBUG_FEATURES
  .irqe           = KINETIS_IRQ_UART2E,
#endif
  .irqs           = KINETIS_IRQ_UART2S,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
  .stop2          = CONFIG_UART2_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART2_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = PIN_UART2_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART2_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = PIN_UART2_RTS,
#endif
#ifdef CONFIG_KINETIS_UART2_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_UART2_RX,
  .rxfifo         = g_uart2rxfifo,
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
#ifdef CONFIG_KINETIS_UART2_RXDMA
  .ops        = &g_uart_dma_ops,
#else
  .ops        = &g_uart_ops,
#endif
  .priv       = &g_uart2priv,
};
#endif

/* This describes the state of the Kinetis UART3 port. */

#ifdef CONFIG_KINETIS_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = KINETIS_UART3_BASE,
  .clock          = BOARD_BUS_FREQ,
  .baud           = CONFIG_UART3_BAUD,
#ifdef CONFIG_DEBUG_FEATURES
  .irqe           = KINETIS_IRQ_UART3E,
#endif
  .irqs           = KINETIS_IRQ_UART3S,
  .parity         = CONFIG_UART3_PARITY,
  .bits           = CONFIG_UART3_BITS,
  .stop2          = CONFIG_UART3_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART3_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = PIN_UART3_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART3_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = PIN_UART3_RTS,
#endif
#ifdef CONFIG_KINETIS_UART3_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_UART3_RX,
  .rxfifo         = g_uart3rxfifo,
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
#ifdef CONFIG_KINETIS_UART3_RXDMA
  .ops        = &g_uart_dma_ops,
#else
  .ops        = &g_uart_ops,
#endif
  .priv       = &g_uart3priv,
};
#endif

/* This describes the state of the Kinetis UART4 port. */

#ifdef CONFIG_KINETIS_UART4
static struct up_dev_s g_uart4priv =
{
  .uartbase       = KINETIS_UART4_BASE,
  .clock          = BOARD_BUS_FREQ,
  .baud           = CONFIG_UART4_BAUD,
#ifdef CONFIG_DEBUG_FEATURES
  .irqe           = KINETIS_IRQ_UART4E,
#endif
  .irqs           = KINETIS_IRQ_UART4S,
  .parity         = CONFIG_UART4_PARITY,
  .bits           = CONFIG_UART4_BITS,
  .stop2          = CONFIG_UART4_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART4_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = PIN_UART4_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART4_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = PIN_UART4_RTS,
#endif
#ifdef CONFIG_KINETIS_UART4_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_UART4_RXTX,
  .rxfifo         = g_uart4rxfifo,
#endif
};

static uart_dev_t g_uart4port =
{
  .recv       =
    {
      .size   = CONFIG_UART4_RXBUFSIZE,
      .buffer = g_uart4rxbuffer,
    },
  .xmit       =
    {
      .size   = CONFIG_UART4_TXBUFSIZE,
      .buffer = g_uart4txbuffer,
    },
#ifdef CONFIG_KINETIS_UART4_RXDMA
  .ops        = &g_uart_dma_ops,
#else
  .ops        = &g_uart_ops,
#endif
  .priv       = &g_uart4priv,
};
#endif

/* This describes the state of the Kinetis UART5 port. */

#ifdef CONFIG_KINETIS_UART5
static struct up_dev_s g_uart5priv =
{
  .uartbase       = KINETIS_UART5_BASE,
  .clock          = BOARD_BUS_FREQ,
  .baud           = CONFIG_UART5_BAUD,
#ifdef CONFIG_DEBUG_FEATURES
  .irqe           = KINETIS_IRQ_UART5E,
#endif
  .irqs           = KINETIS_IRQ_UART5S,
  .parity         = CONFIG_UART5_PARITY,
  .bits           = CONFIG_UART5_BITS,
  .stop2          = CONFIG_UART5_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART5_OFLOWCONTROL)
  .oflow          = true,
  .cts_gpio       = PIN_UART5_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART5_IFLOWCONTROL)
  .iflow          = true,
  .rts_gpio       = PIN_UART5_RTS,
#endif
#ifdef CONFIG_KINETIS_UART5_RXDMA
  .rxdma_reqsrc   = KINETIS_DMA_REQUEST_SRC_UART5_RX,
  .rxfifo         = g_uart5rxfifo,
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
#ifdef CONFIG_KINETIS_UART5_RXDMA
  .ops        = &g_uart_dma_ops,
#else
  .ops        = &g_uart_ops,
#endif
  .priv       = &g_uart5priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint8_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg8(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint8_t value)
{
  putreg8(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_setuartint
 ****************************************************************************/

static void up_setuartint(struct up_dev_s *priv)
{
  irqstate_t flags;
  uint8_t regval;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in
   * ie
   */

  flags    = enter_critical_section();
  regval   = up_serialin(priv, KINETIS_UART_C2_OFFSET);
  regval  &= ~UART_C2_ALLINTS;
  regval  |= priv->ie;
  up_serialout(priv, KINETIS_UART_C2_OFFSET, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static void up_restoreuartint(struct up_dev_s *priv, uint8_t ie)
{
  irqstate_t flags;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in
   * ie
   */

  flags    = enter_critical_section();
  priv->ie = ie & UART_C2_ALLINTS;
  up_setuartint(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

#ifdef HAVE_UART_CONSOLE
static void up_disableuartint(struct up_dev_s *priv, uint8_t *ie)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (ie)
    {
      *ie = priv->ie;
    }

  up_restoreuartint(priv, 0);
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

#ifdef SERIAL_HAVE_DMA
static uint8_t get_and_clear_uart_status(struct up_dev_s *priv)
{
  uint8_t regval;

  regval = up_serialin(priv, KINETIS_UART_S1_OFFSET);
  if ((regval & (UART_S1_PF | UART_S1_FE | UART_S1_NF | UART_S1_OR)) != 0)
    {
      /* if an error occurred, clear the status flag by reading and
       * discarding the data.
       */

      up_serialin(priv, KINETIS_UART_D_OFFSET);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
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

  /* Configure the UART as an RS-232 UART */

  kinetis_uartconfigure(priv->uartbase, priv->baud, priv->clock,
                        priv->parity, priv->bits, priv->stop2,
                        iflow, oflow);
#endif

  /* Make sure that all interrupts are disabled */

  up_restoreuartint(priv, 0);
  return OK;
}

/****************************************************************************
 * Name: up_dma_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int result;
  uint8_t regval;
  DMACH_HANDLE rxdma = NULL;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = up_setup(dev);
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
  config.saddr  = priv->uartbase + KINETIS_UART_D_OFFSET;
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

  regval  = up_serialin(priv, KINETIS_UART_C5_OFFSET);
  regval |= UART_C5_RDMAS;
  up_serialout(priv, KINETIS_UART_C5_OFFSET, regval);

  /* Start the DMA channel, and arrange for callbacks at the half and
   * full points in the FIFO.  This ensures that we have half a FIFO
   * worth of time to claim bytes before they are overwritten.
   */

  kinetis_dmach_start(rxdma, up_dma_rxcallback, (void *)dev);
  priv->rxdma = rxdma;
  return OK;
}
#endif

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

  /* Disable interrupts */

  up_restoreuartint(priv, 0);

  /* Reset hardware and disable Rx and Tx */

  kinetis_uartreset(priv->uartbase);
}

/****************************************************************************
 * Name: up_dma_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void up_dma_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  DMACH_HANDLE rxdma = priv->rxdma;

  /* Perform the normal UART shutdown */

  up_shutdown(dev);

  /* Stop the DMA channel */

  kinetis_dmach_stop(rxdma);

  /* Release the DMA channel */

  kinetis_dmach_free(rxdma);

  priv->rxdma = NULL;
}
#endif

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
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

  /* Attach and enable the IRQ(s).  The interrupts are (probably) still
   * disabled in the C2 register.
   */

  ret = irq_attach(priv->irqs, up_interrupts, dev);
#ifdef CONFIG_DEBUG_FEATURES
  if (ret == OK)
    {
      ret = irq_attach(priv->irqe, up_interrupt, dev);
    }
#endif

  if (ret == OK)
    {
#ifdef CONFIG_DEBUG_FEATURES
      up_enable_irq(priv->irqe);
#endif
      up_enable_irq(priv->irqs);
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

  /* Disable interrupts */

  up_restoreuartint(priv, 0);
#ifdef CONFIG_DEBUG_FEATURES
  up_disable_irq(priv->irqe);
#endif
  up_disable_irq(priv->irqs);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irqs);
#ifdef CONFIG_DEBUG_FEATURES
  irq_detach(priv->irqe);
#endif
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART error interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint8_t            regval;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Handle error interrupts.  This interrupt may be caused by:
   *
   * FE: Framing error. To clear FE, read S1 with FE set and then read the
   *     UART data register (D).
   * NF: Noise flag. To clear NF, read S1 and then read the UART data
   *     register (D).
   * PF: Parity error flag. To clear PF, read S1 and then read the UART data
   *     register (D).
   */

  regval = up_serialin(priv, KINETIS_UART_S1_OFFSET);
  _info("S1: %02x\n", regval);
  UNUSED(regval);

  regval = up_serialin(priv, KINETIS_UART_D_OFFSET);
  UNUSED(regval);

  return OK;
}
#endif /* CONFIG_DEBUG_FEATURES */

/****************************************************************************
 * Name: up_interrupts
 *
 * Description:
 *   This is the UART status interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupts(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  int                passes;
#ifdef CONFIG_KINETIS_UARTFIFOS
  unsigned int       count;
#else
  uint8_t            s1;
#endif
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

      /* Read status register 1 */

#ifndef CONFIG_KINETIS_UARTFIFOS
      s1 = up_serialin(priv, KINETIS_UART_S1_OFFSET);
#endif

      /* Handle incoming, receive bytes */

#ifdef CONFIG_KINETIS_UARTFIFOS
      /* Check the count of bytes in the RX FIFO */

      count = up_serialin(priv, KINETIS_UART_RCFIFO_OFFSET);
      if (count > 0)
#else
      /* Check if the receive data register is full (RDRF).  NOTE:  If
       * FIFOS are enabled, this does not mean that the FIFO is full,
       * rather, it means that the number of bytes in the RX FIFO has
       * exceeded the watermark setting.  There may actually be RX data
       * available!
       *
       * The RDRF status indication is cleared when the data is read from
       * the RX data register.
       */

      if ((s1 & UART_S1_RDRF) != 0)
#endif
        {
          /* Process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes */

#ifdef CONFIG_KINETIS_UARTFIFOS
      /* Read the number of bytes currently in the FIFO and compare that to
       * the size of the FIFO.  If there are fewer bytes in the FIFO than
       * the size of the FIFO, then we are able to transmit.
       */

#  error "Missing logic"
#else
      /* Check if the transmit data register is "empty."  NOTE:  If FIFOS
       * are enabled, this does not mean that the FIFO is empty, rather,
       * it means that the number of bytes in the TX FIFO is below the
       * watermark setting.  There could actually be space for additional TX
       * data.
       *
       * The TDRE status indication is cleared when the data is written to
       * the TX data register.
       */

      if ((s1 & UART_S1_TDRE) != 0)
#endif
        {
          /* Process outgoing bytes */

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
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_KINETIS_SERIALBRK_BSDCOMPAT)
  bool               iflow = false;
  bool               oflow = false;
#endif
  struct inode      *inode;
  struct uart_dev_s *dev;
  uint8_t            regval;
  struct up_dev_s   *priv;
  int                ret   = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv  = (struct up_dev_s *)dev->priv;

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

        regval = up_serialin(priv, KINETIS_UART_C1_OFFSET);

        if ((arg & SER_SINGLEWIRE_ENABLED) != 0)
          {
            regval |= (UART_C1_LOOPS | UART_C1_RSRC);
          }
        else
          {
            regval &= ~(UART_C1_LOOPS | UART_C1_RSRC);
          }

        up_serialout(priv, KINETIS_UART_C1_OFFSET, regval);
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

        /* Note: CSIZE only supports 5-8 bits. The driver only support
         * 8/9 bit modes and therefore is no way to report 9-bit mode, we
         * always claim 8 bit mode.
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

        /* TODO: CCTS_IFLOW, CCTS_OFLOW */
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

        kinetis_uartconfigure(priv->uartbase, priv->baud, priv->clock,
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

        regval = up_serialin(priv, KINETIS_UART_S2_OFFSET);
        regval &= ~UART_S2_BRK13;
# ifdef CONFIG_KINETIS_UART_EXTEDED_BREAK
        regval |= UART_S2_BRK13;
#  endif
        up_serialout(priv, KINETIS_UART_S2_OFFSET, regval);

        /* Send a break signal */

        regval = up_serialin(priv, KINETIS_UART_C2_OFFSET);
        regval |= UART_C2_SBK;
        up_serialout(priv, KINETIS_UART_C2_OFFSET, regval);

#  ifdef CONFIG_KINETIS_SERIALBRK_BSDCOMPAT
        /* BSD compatibility: Turn break on, and leave it on */

        up_txint(dev, false);
#  else
        /* Send a single break character
         * Toggling SBK sends one break character. Per the manual
         * Toggling implies clearing the SBK field before the break
         * character has finished transmitting.
         */

        regval &= ~(UART_C2_SBK);
        up_serialout(priv, KINETIS_UART_C2_OFFSET, regval);
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

        regval = up_serialin(priv, KINETIS_UART_C2_OFFSET);
        regval &= ~UART_C2_SBK;
        up_serialout(priv, KINETIS_UART_C2_OFFSET, regval);

#  ifdef CONFIG_KINETIS_SERIALBRK_BSDCOMPAT
        /* Enable further tx activity */

        up_txint(dev, true);
#  endif
        leave_critical_section(flags);
      }
      break;
#endif /* CONFIG_KINETIS_UART_BREAKS */

#ifdef CONFIG_KINETIS_UART_INVERT
    case TIOCSINVERT:
      {
        uint8_t s2;
        uint8_t c3;
        irqstate_t flags;

        flags = enter_critical_section();

        s2 = up_serialin(priv, KINETIS_UART_S2_OFFSET);
        c3 = up_serialin(priv, KINETIS_UART_C3_OFFSET);

        /* {R|T}XINV bit fields can written any time */

        if (arg & SER_INVERT_ENABLED_RX)
          {
            s2 |= UART_S2_RXINV;
          }
        else
          {
            s2 &= ~UART_S2_RXINV;
          }

        if (arg & SER_INVERT_ENABLED_TX)
          {
            c3 |= UART_C3_TXINV;
          }
        else
          {
            c3 &= ~UART_C3_TXINV;
          }

        up_serialout(priv, KINETIS_UART_S2_OFFSET, s2);
        up_serialout(priv, KINETIS_UART_C3_OFFSET, c3);

        leave_critical_section(flags);
      }
     break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  UNUSED(regval);
  UNUSED(priv);

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

#if !defined(SERIAL_HAVE_ALL_DMA)
static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint8_t s1;

  /* Get error status information:
   *
   * FE: Framing error. To clear FE, read S1 with FE set and then read
   *     read UART data register (D).
   * NF: Noise flag. To clear NF, read S1 and then read the UART data
   *     register (D).
   * PF: Parity error flag. To clear PF, read S1 and then read the UART
   *     data register (D).
   */

  s1 = up_serialin(priv, KINETIS_UART_S1_OFFSET);

  /* Return status information */

  if (status)
    {
      *status = (uint32_t)s1;
    }

  /* Then return the actual received byte.  Reading S1 then D clears all
   * RX errors.
   */

  return (int)up_serialin(priv, KINETIS_UART_D_OFFSET);
}
#endif

/****************************************************************************
 * Name: up_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int c = 0;
  uint8_t s1;

  /* Clear uart errors and return status information */

  s1 = get_and_clear_uart_status(priv);
  if (status)
    {
      *status = (uint32_t)s1;
    }

  if (up_dma_nextrx(priv) != priv->rxdmanext)
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

          if (up_dma_nextrx(priv) != 0)
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
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an RX timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= UART_C2_RIE;
      up_setuartint(priv);
#endif
    }
  else
    {
#ifdef CONFIG_DEBUG_FEATURES
#  warning "Revisit:  How are errors enabled?"
      priv->ie &= ~UART_C2_RIE;
#else
      priv->ie &= ~UART_C2_RIE;
#endif
      up_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#if !defined(SERIAL_HAVE_ALL_DMA)
static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
#ifdef CONFIG_KINETIS_UARTFIFOS
  unsigned int count;

  /* Return true if there are any bytes in the RX FIFO */

  count = up_serialin(priv, KINETIS_UART_RCFIFO_OFFSET);
  return count > 0;
#else
  /* Return true if the receive data register is full (RDRF).  NOTE:  If
   * FIFOS are enabled, this does not mean that the FIFO is full,
   * rather, it means that the number of bytes in the RX FIFO has
   * exceeded the watermark setting.  There may actually be RX data
   * available!
   */

  return (up_serialin(priv, KINETIS_UART_S1_OFFSET) & UART_S1_RDRF) != 0;
#endif
}
#endif

/****************************************************************************
 * Name: up_dma_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static bool up_dma_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (up_dma_nextrx(priv) != priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: up_rxflowcontrol
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
static bool up_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS)
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
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
          ie &= ~UART_C2_RIE;
          up_restoreuartint(priv, ie);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input is
           * received.
           */

          up_rxint(dev, true);
        }
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: up_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_nextrx(struct up_dev_s *priv)
{
  size_t dmaresidual;

  dmaresidual = kinetis_dmach_getcount(priv->rxdma);

  return (RXDMA_BUFFER_SIZE - (int)dmaresidual) % RXDMA_BUFFER_SIZE;
}
#endif

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, KINETIS_UART_D_OFFSET, (uint8_t)ch);
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
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= UART_C2_TIE;
      up_setuartint(priv);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->ie &= ~UART_C2_TIE;
      up_setuartint(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

#ifdef CONFIG_KINETIS_UARTFIFOS
  /* Read the number of bytes currently in the FIFO and compare that to the
   * size of the FIFO.  If there are fewer bytes in the FIFO than the size
   * of the FIFO, then we are able to transmit.
   */

#  error "Missing logic"
#else
  /* Return true if the transmit data register is "empty."  NOTE:  If
   * FIFOS are enabled, this does not mean that the FIFO is empty,
   * rather, it means that the number of bytes in the TX FIFO is
   * below the watermark setting.  There may actually be space for
   * additional TX data.
   */

  return (up_serialin(priv, KINETIS_UART_S1_OFFSET) & UART_S1_TDRE) != 0;
#endif
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_UARTFIFOS
static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return true if the transmit buffer/fifo is "empty." */

  return (up_serialin(priv, KINETIS_UART_SFIFO_OFFSET) & \
          UART_SFIFO_TXEMPT) != 0;
}
#endif

/****************************************************************************
 * Name: up_dma_rxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void up_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                              int result)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;

  if (up_dma_rxavailable(dev))
    {
      uart_recvchars(dev);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_uart_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

#if defined(USE_EARLYSERIALINIT)
void kinetis_uart_earlyserialinit(void)
{
  /* Disable interrupts from all UARTS. */

  up_restoreuartint(TTYS0_DEV.priv, 0);
#ifdef TTYS1_DEV
  up_restoreuartint(TTYS1_DEV.priv, 0);
#endif
#ifdef TTYS2_DEV
  up_restoreuartint(TTYS2_DEV.priv, 0);
#endif
#ifdef TTYS3_DEV
  up_restoreuartint(TTYS3_DEV.priv, 0);
#endif
#ifdef TTYS4_DEV
  up_restoreuartint(TTYS4_DEV.priv, 0);
#endif
#ifdef TTYS5_DEV
  up_restoreuartint(TTYS5_DEV.priv, 0);
#endif

  /* Configuration whichever one is the console. */

#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: kinetis_uart_serialinit
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

unsigned int kinetis_uart_serialinit(unsigned int first)
{
  char devname[] = "/dev/ttySx";

  /* Register the console */

#ifdef HAVE_UART_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);

#  ifdef SERIAL_HAVE_CONSOLE_DMA
  /* If we need to re-initialise the console to enable DMA do that here. */

  up_dma_setup(&CONSOLE_DEV);
#  endif
#endif

  /* Register all UARTs */

  devname[(sizeof(devname) / sizeof(devname[0]))-2] = '0' + first++;
  uart_register(devname, &TTYS0_DEV);
#ifdef TTYS1_DEV
  devname[(sizeof(devname) / sizeof(devname[0]))-2] = '0' + first++;
  uart_register(devname, &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  devname[(sizeof(devname) / sizeof(devname[0]))-2] = '0' + first++;
  uart_register(devname, &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  devname[(sizeof(devname) / sizeof(devname[0]))-2] = '0' + first++;
  uart_register(devname, &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  devname[(sizeof(devname) / sizeof(devname[0]))-2] = '0' + first++;
  uart_register(devname, &TTYS4_DEV);
#endif
#ifdef TTYS5_DEV
  devname[(sizeof(devname) / sizeof(devname[0]))-2] = '0' + first++;
  uart_register(devname, &TTYS5_DEV);
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

#ifdef SERIAL_HAVE_DMA
void kinetis_serial_dma_poll(void)
{
    irqstate_t flags;

    flags = enter_critical_section();

#ifdef CONFIG_KINETIS_UART0_RXDMA
  if (g_uart0priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart0priv.rxdma, (void *)&g_uart0port, false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_UART1_RXDMA
  if (g_uart1priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart1priv.rxdma, (void *)&g_uart1port, false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_UART2_RXDMA
  if (g_uart2priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart2priv.rxdma, (void *)&g_uart2port, false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_UART3_RXDMA
  if (g_uart3priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart3priv.rxdma, (void *)&g_uart3port, false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_UART4_RXDMA
  if (g_uart4priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart4priv.rxdma, (void *)&g_uart4port, false, 0);
    }
#endif

#ifdef CONFIG_KINETIS_UART5_RXDMA
  if (g_uart5priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart5priv.rxdma, (void *)&g_uart5port, false, 0);
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

#ifdef HAVE_UART_PUTC
int up_putc(int ch)
{
#ifdef HAVE_UART_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint8_t ie;

  up_disableuartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  up_restoreuartint(priv, ie);
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

#ifdef HAVE_UART_PUTC
int up_putc(int ch)
{
#ifdef HAVE_UART_CONSOLE
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

#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER) */
