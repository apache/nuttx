/****************************************************************************
 * arch/arm/src/samv7/sam_serial.c
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
#include "sam_config.h"

#include "hardware/sam_pinmap.h"
#include "hardware/sam_uart.h"
#include "hardware/sam_xdmac.h"
#include "sam_xdmac.h"
#include "sam_gpio.h"
#include "sam_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/* DMA Configuration */

#if defined(CONFIG_USART0_RXDMA) || defined(CONFIG_USART1_RXDMA) || \
    defined(CONFIG_USART2_RXDMA)
# define SERIAL_HAVE_RXDMA
#else
# undef SERIAL_HAVE_RXDMA
#endif

#if defined(CONFIG_UART0_RXDMA) || defined(CONFIG_UART1_RXDMA) || \
    defined(CONFIG_UART2_RXDMA) || defined(CONFIG_UART3_RXDMA) || \
    defined(CONFIG_UART4_RXDMA)
# warning RX DMA is currently supported only for USART driver.
#endif

#if defined(SERIAL_HAVE_RXDMA) && !defined(CONFIG_SAMV7_XDMAC)
# error SERIAL DMA requires CONFIG_SAMV7_XDMAC to be selected
#endif

#ifdef SERIAL_HAVE_CONSOLE_RXDMA
# error RX DMA for serial console is currently not supported.
#endif

#if defined(CONFIG_UART0_TXDMA) || defined(CONFIG_UART1_TXDMA) || \
    defined(CONFIG_UART2_TXDMA) || defined(CONFIG_UART3_TXDMA) || \
    defined(CONFIG_UART4_TXDMA) || defined(CONFIG_USART0_TXDMA) || \
    defined(CONFIG_USART1_TXDMA) || defined(CONFIG_USART2_TXDMA)
# warning TX DMA is currently not supported.
#endif

#ifndef CONFIG_SAMV7_SERIAL_DMA_TIMEOUT
# define CONFIG_SAMV7_SERIAL_DMA_TIMEOUT 0
#endif

#ifdef SERIAL_HAVE_RXDMA

# define DMA_RXFLAGS  (DMACH_FLAG_FIFOCFG_LARGEST | \
     DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |  \
     DMACH_FLAG_PERIPHWIDTH_32BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
     DMACH_FLAG_MEMPID_MAX | DMACH_FLAG_MEMAHB_AHB_IF0 | \
     DMACH_FLAG_PERIPHAHB_AHB_IF1 | DMACH_FLAG_MEMWIDTH_32BITS | \
     DMACH_FLAG_MEMINCREMENT | DMACH_FLAG_MEMCHUNKSIZE_1 | \
     DMACH_FLAG_MEMBURST_1)

/* The DMA buffer size when using RX DMA to emulate a FIFO.
 *
 * When streaming data, the generic serial layer will be called
 * every time the FIFO receives half this number of bytes.
 */

#  ifndef CONFIG_SAMV7_SERIAL_RXDMA_BUFFER
#    define CONFIG_SAMV7_SERIAL_RXDMA_BUFFER 128
#  endif
#define RXDMA_CACHE_SIZE    (ARMV7M_DCACHE_LINESIZE/sizeof(uint32_t))
#define RXDMA_MUTIPLE       (sizeof(uint32_t) > RXDMA_CACHE_SIZE ? \
                             sizeof(uint32_t) : RXDMA_CACHE_SIZE)
#define RXDMA_MUTIPLE_MASK  (RXDMA_MUTIPLE - 1)
#define RXDMA_BUFFER_SIZE   ((CONFIG_SAMV7_SERIAL_RXDMA_BUFFER \
                              + RXDMA_MUTIPLE_MASK) \
                              & ~RXDMA_MUTIPLE_MASK)

#endif  /* SERIAL_HAVE_RXDMA */

/* Which UART/USART with be tty0/console and which tty1-7? */

/* First pick the console and ttys0.  This could be any of UART0-4,
 * USART0-2
 */

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
#  elif defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER)
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#  elif defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER)
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED     1
#  elif defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER)
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
#elif defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER) && \
     !defined(USART0_ASSIGNED)
#  define TTYS1_DEV           g_usart0port /* USART0 is ttyS1 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER) && \
     !defined(USART1_ASSIGNED)
#  define TTYS1_DEV           g_usart1port /* USART1 is ttyS1 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER) && \
     !defined(USART2_ASSIGNED)
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
#elif defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER) && \
     !defined(USART0_ASSIGNED)
#  define TTYS2_DEV           g_usart0port /* USART0 is ttyS2 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER) && \
     !defined(USART1_ASSIGNED)
#  define TTYS2_DEV           g_usart1port /* USART1 is ttyS2 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER) && \
     !defined(USART2_ASSIGNED)
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
#elif defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER) && \
     !defined(USART0_ASSIGNED)
#  define TTYS3_DEV           g_usart0port /* USART0 is ttyS3 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER) && \
     !defined(USART1_ASSIGNED)
#  define TTYS3_DEV           g_usart1port /* USART1 is ttyS3 */
#  define USART1_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER) && \
     !defined(USART2_ASSIGNED)
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
#elif defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER) && \
     !defined(USART0_ASSIGNED)
#  define TTYS4_DEV           g_usart0port /* USART0 is ttyS4 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER) && \
     !defined(USART1_ASSIGNED)
#  define TTYS4_DEV           g_usart1port /* USART1 is ttyS4 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER) && \
     !defined(USART2_ASSIGNED)
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
#elif defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER) && \
     !defined(USART0_ASSIGNED)
#  define TTYS5_DEV           g_usart0port /* USART0 is ttyS5 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER) && \
     !defined(USART1_ASSIGNED)
#  define TTYS5_DEV           g_usart1port /* USART1 is ttyS5 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER) && \
     !defined(USART2_ASSIGNED)
#  define TTYS5_DEV           g_usart2port /* USART2 is ttyS5 */
#  define USART2_ASSIGNED      1
#endif

/* Pick ttys6. This could be one of USART0-2. It can't be UART0-4
 * because those have already been assigned to ttsyS0-5.  One of
 * One of USART0-2 could also be the console.
 */

#if defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER) && \
   !defined(USART0_ASSIGNED)
#  define TTYS6_DEV           g_usart0port /* USART0 is ttyS6 */
#  define USART0_ASSIGNED     1
#elif defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER) && \
     !defined(USART1_ASSIGNED)
#  define TTYS6_DEV           g_usart1port /* USART1 is ttyS6 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER) && \
     !defined(USART2_ASSIGNED)
#  define TTYS6_DEV           g_usart2port /* USART2 is ttyS6 */
#  define USART2_ASSIGNED      1
#endif

/* Pick ttys7. This could be one of USART1-2. It can't be UART0-4
 * or USART 1 because those have already been assigned to ttsyS0-6.
 * One of USART1-2 could also be the console.
 */

#if defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER) && \
   !defined(USART1_ASSIGNED)
#  define TTYS7_DEV           g_usart1port /* USART1 is ttyS7 */
#  define USART1_ASSIGNED      1
#elif defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER) && \
     !defined(USART2_ASSIGNED)
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
  struct uart_dev_s dev;        /* Generic UART device */
  const uint32_t usartbase;     /* Base address of USART registers */
  uint32_t baud;                /* Configured baud */
  uint32_t sr;                  /* Saved status bits */
  uint8_t  irq;                 /* IRQ associated with this USART */
  uint8_t  pid;                 /* Peripheral PID */
  uint8_t  parity;              /* 0=none, 1=odd, 2=even */
  uint8_t  bits;                /* Number of bits (5-9) */
  bool     stopbits2;           /* true: Configure with 2 stop bits instead of 1 */
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  bool     flowc;               /* input flow control (RTS) enabled */
#endif

  bool     has_rxdma;           /* True if RX DMA is enabled */
  bool     has_rs485;           /* True if RS-485 mode is enabled */

#ifdef SERIAL_HAVE_RS485
  uint32_t rs485_dir_gpio;      /* RS-485 RTS pin */
#endif
  /* RX DMA state */

#ifdef SERIAL_HAVE_RXDMA
  const unsigned int rxdma_channel; /* DMA channel assigned */
  DMA_HANDLE         rxdma;         /* currently-open receive DMA stream */
  bool               rxenable;      /* DMA-based reception en/disable */
  bool               odd;           /* True if odd buffer is used */
  uint8_t            buf_idx;       /* 0 or 1, points to the correct buffer */
  uint32_t           nextcache;     /* Next byte in data cache to be invalidated */
  uint32_t           rxdmanext;     /* Next byte in the DMA buffer to be read */
  uint32_t * const   rxbuf[2];      /* Receive DMA buffer */
  struct chnext_view1_s *desc[2];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sam_setup(struct uart_dev_s *dev);
static void sam_shutdown(struct uart_dev_s *dev);
static int  sam_attach(struct uart_dev_s *dev);
static void sam_detach(struct uart_dev_s *dev);
static int  sam_interrupt(int irq, void *context, void *arg);
static int  sam_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifdef SERIAL_HAVE_NODMA_OPS
static int  sam_receive(struct uart_dev_s *dev, unsigned int *status);
static void sam_rxint(struct uart_dev_s *dev, bool enable);
static bool sam_rxavailable(struct uart_dev_s *dev);
#endif
static void sam_send(struct uart_dev_s *dev, int ch);
static void sam_txint(struct uart_dev_s *dev, bool enable);
static bool sam_txready(struct uart_dev_s *dev);
static bool sam_txempty(struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_RXDMA
static int  sam_dma_setup(struct uart_dev_s *dev);
static void sam_dma_shutdown(struct uart_dev_s *dev);
static int  sam_dma_receive(struct uart_dev_s *dev, unsigned int *status);
static void sam_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool sam_dma_rxavailable(struct uart_dev_s *dev);
static void sam_dma_rxcallback(DMA_HANDLE handle, void *arg, int status);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* No DMA */

#ifdef SERIAL_HAVE_NODMA_OPS
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
#endif

/* RX DMA is defined */

#ifdef SERIAL_HAVE_RXDMA_OPS
static const struct uart_ops_s g_uart_rxdma_ops =
{
  .setup          = sam_dma_setup,
  .shutdown       = sam_dma_shutdown,
  .attach         = sam_attach,
  .detach         = sam_detach,
  .ioctl          = sam_ioctl,
  .receive        = sam_dma_receive,
  .rxint          = sam_dma_rxint,
  .rxavailable    = sam_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = sam_send,
  .txint          = sam_txint,
  .txready        = sam_txready,
  .txempty        = sam_txready,
};
#endif

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
#if defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER)
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
# ifdef CONFIG_USART0_RXDMA
static uint32_t g_usart0rxbuf[2][RXDMA_BUFFER_SIZE]
aligned_data(ARMV7M_DCACHE_LINESIZE);
static struct chnext_view1_s g_usart0rxdesc[2];
# endif
#endif
#if defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER)
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
# ifdef CONFIG_USART1_RXDMA
static uint32_t g_usart1rxbuf[2][RXDMA_BUFFER_SIZE]
aligned_data(ARMV7M_DCACHE_LINESIZE);
static struct chnext_view1_s g_usart1rxdesc[2];
# endif
#endif
#if defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER)
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
# ifdef CONFIG_USART2_RXDMA
static uint32_t g_usart2rxbuf[2][RXDMA_BUFFER_SIZE]
aligned_data(ARMV7M_DCACHE_LINESIZE);
static struct chnext_view1_s g_usart2rxdesc[2];
# endif
#endif

/* This describes the state of the UART0 port. */

#ifdef CONFIG_SAMV7_UART0
static struct sam_dev_s g_uart0priv =
{
  .usartbase      = SAM_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = SAM_IRQ_UART0,
  .pid            = SAM_PID_UART0,
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
  .baud           = CONFIG_UART1_BAUD,
  .irq            = SAM_IRQ_UART1,
  .pid            = SAM_PID_UART1,
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
  .baud           = CONFIG_UART2_BAUD,
  .irq            = SAM_IRQ_UART2,
  .pid            = SAM_PID_UART2,
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
  .baud           = CONFIG_UART3_BAUD,
  .irq            = SAM_IRQ_UART3,
  .pid            = SAM_PID_UART3,
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
  .baud           = CONFIG_UART4_BAUD,
  .irq            = SAM_IRQ_UART4,
  .pid            = SAM_PID_UART4,
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

#if defined(CONFIG_SAMV7_USART0) && defined(CONFIG_USART0_SERIALDRIVER)
static struct sam_dev_s g_usart0priv =
{
  .usartbase      = SAM_USART0_BASE,
  .baud           = CONFIG_USART0_BAUD,
  .irq            = SAM_IRQ_USART0,
  .pid            = SAM_PID_USART0,
  .parity         = CONFIG_USART0_PARITY,
  .bits           = CONFIG_USART0_BITS,
  .stopbits2      = CONFIG_USART0_2STOP,
#if defined(CONFIG_USART0_OFLOWCONTROL) || defined(CONFIG_USART0_IFLOWCONTROL)
  .flowc          = true,
#endif
#ifdef CONFIG_USART0_RXDMA
  .buf_idx        = 0,
  .nextcache      = 0,
  .rxbuf          =
  {
    g_usart0rxbuf[0], g_usart0rxbuf[1]
  },
  .desc           =
  {
    &g_usart0rxdesc[0], &g_usart0rxdesc[1]
  },
  .has_rxdma      = true,
#endif
#ifdef CONFIG_SAMV7_USART0_RS485MODE
  .has_rs485      = true,
  .rs485_dir_gpio = GPIO_USART0_RTS,
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
#ifdef CONFIG_USART0_RXDMA
  .ops      = &g_uart_rxdma_ops,
#else
  .ops      = &g_uart_ops,
#endif
  .priv     = &g_usart0priv,
};
#endif

/* This describes the state of the USART1 port. */

#if defined(CONFIG_SAMV7_USART1) && defined(CONFIG_USART1_SERIALDRIVER)
static struct sam_dev_s g_usart1priv =
{
  .usartbase      = SAM_USART1_BASE,
  .baud           = CONFIG_USART1_BAUD,
  .irq            = SAM_IRQ_USART1,
  .pid            = SAM_PID_USART1,
  .parity         = CONFIG_USART1_PARITY,
  .bits           = CONFIG_USART1_BITS,
  .stopbits2      = CONFIG_USART1_2STOP,
#if defined(CONFIG_USART1_OFLOWCONTROL) || defined(CONFIG_USART1_IFLOWCONTROL)
  .flowc          = true,
#endif
#ifdef CONFIG_USART1_RXDMA
  .buf_idx        = 0,
  .nextcache      = 0,
  .rxbuf          =
  {
    g_usart1rxbuf[0], g_usart1rxbuf[1]
  },
  .desc           =
  {
    &g_usart1rxdesc[0], &g_usart1rxdesc[1]
  },
  .has_rxdma      = true,
#endif
#ifdef CONFIG_SAMV7_USART1_RS485MODE
  .has_rs485      = true,
  .rs485_dir_gpio = GPIO_USART1_RTS,
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
#ifdef CONFIG_USART1_RXDMA
  .ops      = &g_uart_rxdma_ops,
#else
  .ops      = &g_uart_ops,
#endif
  .priv     = &g_usart1priv,
};
#endif

/* This describes the state of the USART2 port. */

#if defined(CONFIG_SAMV7_USART2) && defined(CONFIG_USART2_SERIALDRIVER)
static struct sam_dev_s g_usart2priv =
{
  .usartbase      = SAM_USART2_BASE,
  .baud           = CONFIG_USART2_BAUD,
  .irq            = SAM_IRQ_USART2,
  .pid            = SAM_PID_USART2,
  .parity         = CONFIG_USART2_PARITY,
  .bits           = CONFIG_USART2_BITS,
  .stopbits2      = CONFIG_USART2_2STOP,
#if defined(CONFIG_USART2_OFLOWCONTROL) || defined(CONFIG_USART2_IFLOWCONTROL)
  .flowc          = true,
#endif
#ifdef CONFIG_USART2_RXDMA
  .buf_idx        = 0,
  .nextcache      = 0,
  .rxbuf          =
  {
    g_usart2rxbuf[0], g_usart2rxbuf[1]
  },
  .desc           =
  {
    &g_usart2rxdesc[0], &g_usart2rxdesc[1]
  },
  .has_rxdma      = true,
#endif
#ifdef CONFIG_SAMV7_USART2_RS485MODE
  .has_rs485      = true,
  .rs485_dir_gpio = GPIO_USART2_RTS,
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
#ifdef CONFIG_USART2_RXDMA
  .ops        = &g_uart_rxdma_ops,
#else
  .ops        = &g_uart_ops,
#endif
  .priv       = &g_usart2priv,
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

static inline void sam_serialout(struct sam_dev_s *priv, int offset,
                                 uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: sam_restoreusartint
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TERMIOS
static inline void sam_restoreusartint(struct sam_dev_s *priv, uint32_t imr)
{
  /* Restore the previous interrupt state (assuming all interrupts
   * disabled)
   */

  sam_serialout(priv, SAM_UART_IER_OFFSET, imr);
}
#endif

/****************************************************************************
 * Name: sam_disableallints
 ****************************************************************************/

static void sam_disableallints(struct sam_dev_s *priv, uint32_t *imr)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = enter_critical_section();
  if (imr)
    {
      /* Return the current interrupt mask */

      *imr = sam_serialin(priv, SAM_UART_IMR_OFFSET);
    }

  /* Disable all interrupts */

  sam_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_ALLINTS);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int sam_dma_nextrx(struct sam_dev_s *priv)
{
  uint32_t cda;
  uint32_t ret;

  /* Get the DMA destination address */

  cda = sam_destaddr(priv->rxdma);

  if (!priv->odd)
    {
      ret = (cda - (uint32_t)priv->rxbuf[0]) / sizeof(uint32_t);
      if (cda >= ((uint32_t)priv->rxbuf[0] + (RXDMA_BUFFER_SIZE << 2)))
        {
          /* We are already in another buffer so we need to read the
           * whole RXDMA_BUFFER_SIZE.
           */

          ret = RXDMA_BUFFER_SIZE;
        }

      return ret;
    }
  else
    {
      ret = (cda - (uint32_t)priv->rxbuf[1]) / sizeof(uint32_t);
      if (cda < (uint32_t)priv->rxbuf[1])
        {
          /* We are already in another buffer so we need to read the
           * whole RXDMA_BUFFER_SIZE.
           */

          ret = RXDMA_BUFFER_SIZE;
        }

      return ret;
    }
}
#endif

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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
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
   *  reception. The transmitter can handle hardware handshaking in any
   *  case."
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

  sam_serialout(priv, SAM_UART_CR_OFFSET, (UART_CR_RXEN | UART_CR_TXEN));

#ifdef SERIAL_HAVE_RS485
  if (priv->has_rs485)
    {
      sam_configgpio(priv->rs485_dir_gpio);

      regval = sam_serialin(priv, SAM_UART_MR_OFFSET);
      regval |= UART_MR_MODE_RS485;
      sam_serialout(priv, SAM_UART_MR_OFFSET, regval);
    }
#endif
#endif

  return OK;
}

/****************************************************************************
 * Name: sam_dma_setup
 *
 * Description:
 *   Configure the UART (USART) baud, bits, parity, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int sam_dma_setup(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  int result;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = sam_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

#ifdef SERIAL_HAVE_RXDMA
  /* Acquire the DMA channel. This should always succeed. */

  if (priv->has_rxdma)
    {
      /* Do this only if this peripheral has RX DMA. This might not be
       * neccessary at current state of the driver as sam_dma_setup()
       * cannot be called if USARTn does not have RX DMA but it will be
       * needed when TX DMA is implemented. The sam_dma_setup() can be
       * called even when TX DMA is defined and RX DMA is not.
       */

      priv->rxdma = sam_dmachannel(0, DMA_RXFLAGS |
                                      DMACH_FLAG_PERIPHPID(priv->pid));

      /* Configure for circular DMA reception into the RX fifo */

      uint32_t paddr = priv->usartbase + SAM_UART_RHR_OFFSET;
      uint32_t maddr[] =
        {
          (uintptr_t)priv->rxbuf[0],
          (uintptr_t)priv->rxbuf[1]
        };

      /* sam_dmarxsetup() needs number of bytes to transfer. Since 1
       * transfer is 32 bits = 4 bytes we need to multiply
       * RXDMA_BUFFER_SIZE by 4 to get the number of characters as
       * defined in RXDMA_BUFFER_SIZE.
       */

      size_t buflen = RXDMA_BUFFER_SIZE << 2;

      sam_dmarxsetup_circular(priv->rxdma, priv->desc, maddr,
                              paddr, buflen, 2);

      /* Reset our DMA shadow pointer to match the address just
       * programmed above.
       */

      priv->rxdmanext = 0;
      priv->rxenable = true;
      priv->odd = false;

      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      sam_dmastart_circular(priv->rxdma, sam_dma_rxcallback, (void *)dev);

      /* Use defined timout to check if RX bus is in idle state */

      sam_serialout(priv, SAM_UART_RTOR_OFFSET,
                    CONFIG_SAMV7_SERIAL_DMA_TIMEOUT);
      sam_serialout(priv, SAM_UART_IER_OFFSET, UART_INT_TIMEOUT);
    }

#endif

  return OK;
}
#endif

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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  uint32_t regval;

  /* Reset and disable receiver and transmitter */

  sam_serialout(priv, SAM_UART_CR_OFFSET,
                (UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS |
                 UART_CR_TXDIS));

  /* Set mode back to normal */

  regval = sam_serialin(priv, SAM_UART_MR_OFFSET);
  regval &= ~UART_MR_MODE_MASK;
  sam_serialout(priv, SAM_UART_MR_OFFSET, regval);

#ifdef SERIAL_HAVE_RS485
  if (priv->has_rs485)
    {
      /* Force RTS pin to get low if RS-485 mode is enabled */

      sam_serialout(priv, SAM_UART_CR_OFFSET, UART_CR_RTSEN);
    }
#endif

  /* Disable all interrupts */

  sam_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: sam_dma_shutdown
 *
 * Description:
 *   Disable the UART/USART. This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void sam_dma_shutdown(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* Perform the normal UART shutdown */

  sam_shutdown(dev);

#ifdef SERIAL_HAVE_RXDMA
  /* Stop the RX DMA channel */

  sam_dmastop(priv->rxdma);

  /* Release the RX DMA channel */

  sam_dmafree(priv->rxdma);
  priv->rxdma = NULL;
#endif
}
#endif

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
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, sam_interrupt, dev);
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
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void sam_detach(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
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

static int sam_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct sam_dev_s *priv;
  uint32_t pending;
  uint32_t imr;
  int passes;
  bool handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct sam_dev_s *)dev->priv;

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

      priv->sr = sam_serialin(priv, SAM_UART_SR_OFFSET);
      imr      = sam_serialin(priv, SAM_UART_IMR_OFFSET);
      pending  = priv->sr & imr;

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

      /* Timeout interrupt (idle detection for USART) */

#ifdef SERIAL_HAVE_RXDMA
      if ((pending & UART_INT_TIMEOUT) != 0)
        {
          if (priv->has_rxdma)
            {
              /* We received Timeout interrupt */

              uint32_t regval = sam_serialin(priv, SAM_UART_CR_OFFSET);
              regval |= UART_CR_STTTO;
              sam_serialout(priv, SAM_UART_CR_OFFSET, regval);

              sam_dma_rxcallback(priv->rxdma, dev, 0);
            }
        }
#endif /* SERIAL_HAVE_RXDMA */
    }

  return OK;
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
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  int                ret   = OK;

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

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct sam_dev_s *priv    = (struct sam_dev_s *)dev->priv;

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
        struct sam_dev_s *priv    = (struct sam_dev_s *)dev->priv;
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

    case TIOCSLINID:
      {
        /* Switch USART to LIN mode -> Set identifier register -> This will
         * issue a LIN header -> Restore USART mode
         */

        struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
        uint32_t regvalsave = 0;
        uint32_t regval = 0;

        /* Save actual configuration */

        regvalsave = sam_serialin(priv, SAM_UART_MR_OFFSET);

        /* Issue a LIN header (Master mode only) */

        regval  = regvalsave & ~UART_MR_MODE_MASK;
        regval |= UART_MR_MODE_LINMSTR;
        sam_serialout(priv, SAM_UART_MR_OFFSET, regval);

        /* Reset transaction status bits */

        sam_serialout(priv, SAM_UART_CR_OFFSET, UART_CR_RSTSTA);

        /* Write LIN ID to trigger header transmission */

        regval = UART_LINIR_MASK & (uint8_t)arg;
        sam_serialout(priv, SAM_UART_LINIR_OFFSET, regval);

        /* Wait until the header is on the wire */

        do
          {
            /* Header takes 34 bits to transmit so poll with 34 / 4 = 8 bits
             * cycle time
             *
             * TODO: Interrupt + semaphore can be used instead of busy
             * waiting to improve CPU utilization
             */

            usleep((8 * 1000 * 1000) / priv->baud);

            regval = sam_serialin(priv, SAM_UART_SR_OFFSET);
          }
        while ((regval & UART_INT_LINID) == 0);

        /* Reset transaction status bits */

        sam_serialout(priv, SAM_UART_CR_OFFSET, UART_CR_RSTSTA);

        /* Restore USART mode */

        sam_serialout(priv, SAM_UART_MR_OFFSET, regvalsave);
      }
      break;

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

#ifdef SERIAL_HAVE_NODMA_OPS
static int sam_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* Return the error information in the saved status */

  *status  = priv->sr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return (int)(sam_serialin(priv, SAM_UART_RHR_OFFSET) & 0xff);
}
#endif

/****************************************************************************
 * Name: sam_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_NODMA_OPS
static void sam_rxint(struct uart_dev_s *dev, bool enable)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an RX timeout occurs).
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
#endif

/****************************************************************************
 * Name: sam_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_NODMA_OPS
static bool sam_rxavailable(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  return ((sam_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_RXRDY) != 0);
}
#endif

/****************************************************************************
 * Name: sam_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int sam_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  int c = 0;

  *status  = priv->sr;
  priv->sr = 0;

  /* Read character from the RX FIFO */

  c = priv->rxbuf[priv->buf_idx][priv->rxdmanext] & UART_RHR_RXCHR_MASK;
  priv->rxdmanext++;

  if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
    {
      priv->rxdmanext = 0;
      priv->nextcache = 0;
      priv->odd = !priv->odd;
      priv->buf_idx = 1 & ~priv->buf_idx;
    }

  return c;
}
#endif

/****************************************************************************
 * Name: sam_dma_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void sam_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* En/disable DMA reception.
   *
   * Note that it is not safe to check for available bytes and immediately
   * pass them to uart_recvchars as that could potentially recurse back
   * to us again.  Instead, bytes must wait until the next sam_dma_poll or
   * DMA event.
   */

  if (priv->rxenable != enable)
    {
      priv->rxenable = enable;

      if (enable)
        {
          sam_serialout(priv, SAM_UART_IER_OFFSET, UART_INT_TIMEOUT);
        }
      else
        {
          sam_serialout(priv, SAM_UART_IDR_OFFSET, UART_INT_TIMEOUT);
        }
    }
}
#endif

/****************************************************************************
 * Name: sam_dma_rxavailable
 *
 * Description:
 *   Return true if there are some data in the buffer we can read. Also takes
 *   care of invalidating data cache.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static bool sam_dma_rxavailable(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  uint32_t nextrx;
  bool ret;

  ret = false;

  /* Get the current DMA pointer */

  nextrx = sam_dma_nextrx(priv);

  /* Compare our receive pointer to the current DMA pointer, if they
   * do match, then there are bytes to be received.
   */

  if ((nextrx != priv->rxdmanext) && priv->rxenable)
    {
      /* Invalidate data cache if necessary. This basically ensures
       * we invalidate only that part of cache we need to.
       */

      if (priv->nextcache < nextrx)
        {
          up_invalidate_dcache((uintptr_t)priv->rxbuf[priv->buf_idx]
                                + (priv->nextcache << 2),
                               (uintptr_t)priv->rxbuf[priv->buf_idx]
                                + (nextrx << 2));

          /* Move the pointer to the memory for which the cache was
           * invalidated.
           */

          priv->nextcache = nextrx;
        }

      ret = true;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: sam_send
 *
 * Description:
 *   This method will send one byte on the UART/USART
 *
 ****************************************************************************/

static void sam_send(struct uart_dev_s *dev, int ch)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  sam_serialout(priv, SAM_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: sam_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA_OPS) || defined(SERIAL_HAVE_NODMA_OPS)
static void sam_txint(struct uart_dev_s *dev, bool enable)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
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

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: sam_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool sam_txready(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  return ((sam_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_TXRDY) != 0);
}

/****************************************************************************
 * Name: sam_dma_rxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void sam_dma_rxcallback(DMA_HANDLE handle, void *arg, int status)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  if (priv->rxenable && sam_dma_rxavailable(dev))
    {
      uart_recvchars(dev);
    }
}
#endif

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
  return ((sam_serialin(priv, SAM_UART_SR_OFFSET) & UART_INT_TXEMPTY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 *   This polling also in not neccessary if CONFIG_SAMV7_SERIAL_DMA_TIMEOUT
 *   is defined as sam_dma_rxcallback() is called each time idle bus is
 *   detected.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void sam_serial_dma_poll(void)
{
    irqstate_t flags;

    flags = enter_critical_section();

#ifdef CONFIG_USART0_RXDMA
  if (g_usart0priv.rxdma != NULL)
    {
      sam_dma_rxcallback(g_usart0priv.rxdma, &g_usart0port, 0);
    }
#endif

#ifdef CONFIG_USART1_RXDMA
  if (g_usart1priv.rxdma != NULL)
    {
      sam_dma_rxcallback(g_usart1priv.rxdma, &g_usart1port, 0);
    }
#endif

#ifdef CONFIG_USART2_RXDMA
  if (g_usart2priv.rxdma != NULL)
    {
      sam_dma_rxcallback(g_usart2priv.rxdma, &g_usart2port, 0);
    }
#endif

  leave_critical_section(flags);
}
#endif

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

#ifdef TTYS0_DEV
  sam_disableallints(TTYS0_DEV.priv, NULL);
#endif
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

#ifdef SERIAL_HAVE_CONSOLE_RXDMA
  /* If we need to re-initialise the console to enable DMA do that here. */

  sam_dma_setup(&CONSOLE_DEV);
#endif

  /* Register all USARTs */

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

#endif /* USE_SERIALDRIVER */
