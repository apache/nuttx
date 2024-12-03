/****************************************************************************
 * drivers/serial/uart_16550.c
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

/* Serial driver for 16550 UART */

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

#include <nuttx/spinlock.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/dma/dma.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/uart_16550.h>

#include <arch/board/board.h>

#ifdef CONFIG_16550_UART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Are any UARTs enabled? */

#undef HAVE_16550_UART
#if defined(CONFIG_16550_UART0) || defined(CONFIG_16550_UART1)  \
  || defined(CONFIG_16550_UART2) || defined(CONFIG_16550_UART3)
#  define HAVE_16550_UART 1
#endif

/* Timeout for UART Busy Wait, in milliseconds */

#define UART_TIMEOUT_MS 100

/* Default getreg/putreg operations */

#ifdef CONFIG_SERIAL_UART_ARCH_MMIO
#  define u16550_getreg u16550_mmio_getreg
#  define u16550_putreg u16550_mmio_putreg
#else
#  define u16550_getreg uart_getreg
#  define u16550_putreg uart_putreg
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SERIAL_UART_ARCH_MMIO
static uart_datawidth_t u16550_mmio_getreg(FAR struct u16550_s *priv,
                                           unsigned int offset);
static void u16550_mmio_putreg(FAR struct u16550_s *priv,
                               unsigned int offset,
                               uart_datawidth_t value);
#endif

static int  u16550_setup(FAR struct uart_dev_s *dev);
static void u16550_shutdown(FAR struct uart_dev_s *dev);
static int  u16550_attach(FAR struct uart_dev_s *dev);
static void u16550_detach(FAR struct uart_dev_s *dev);
static int  u16550_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  u16550_receive(FAR struct uart_dev_s *dev, unsigned int *status);
static void u16550_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool u16550_rxavailable(FAR struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool u16550_rxflowcontrol(struct uart_dev_s *dev,
                                 unsigned int nbuffered, bool upper);
#endif
#ifdef HAVE_16550_UART_DMA
static void u16550_dmasend(FAR struct uart_dev_s *dev);
static void u16550_dmatxavail(FAR struct uart_dev_s *dev);
static void u16550_dmatxconfig(FAR struct uart_dev_s *dev);
static void u16550_dmareceive(FAR struct uart_dev_s *dev);
static void u16550_dmarxfree(FAR struct uart_dev_s *dev);
static void u16550_dmarxconfig(FAR struct uart_dev_s *dev);
#endif
static void u16550_send(FAR struct uart_dev_s *dev, int ch);
static void u16550_txint(FAR struct uart_dev_s *dev, bool enable);
static bool u16550_txready(FAR struct uart_dev_s *dev);
static bool u16550_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_16550_UART
static const struct u16550_ops_s g_u16550_ops =
{
  .isr        = u16550_interrupt,
  .getreg     = u16550_getreg,
  .putreg     = u16550_putreg,
#  ifdef CONFIG_SERIAL_UART_ARCH_IOCTL
  .ioctl      = uart_ioctl,
#  endif
#  ifdef HAVE_16550_UART_DMA
  .dmachan    = uart_dmachan,
#  endif
};
#endif

static const struct uart_ops_s g_uart_ops =
{
  .setup          = u16550_setup,
  .shutdown       = u16550_shutdown,
  .attach         = u16550_attach,
  .detach         = u16550_detach,
  .ioctl          = u16550_ioctl,
  .receive        = u16550_receive,
  .rxint          = u16550_rxint,
  .rxavailable    = u16550_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = u16550_rxflowcontrol,
#endif
#ifdef HAVE_16550_UART_DMA
  .dmasend        = u16550_dmasend,
  .dmareceive     = u16550_dmareceive,
  .dmarxfree      = u16550_dmarxfree,
  .dmatxavail     = u16550_dmatxavail,
#endif
  .send           = u16550_send,
  .txint          = u16550_txint,
  .txready        = u16550_txready,
  .txempty        = u16550_txempty,
};

/* I/O buffers */

#ifdef CONFIG_16550_UART0
static char g_uart0rxbuffer[CONFIG_16550_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_16550_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART1
static char g_uart1rxbuffer[CONFIG_16550_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_16550_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART2
static char g_uart2rxbuffer[CONFIG_16550_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_16550_UART2_TXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART3
static char g_uart3rxbuffer[CONFIG_16550_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_16550_UART3_TXBUFSIZE];
#endif

/* DMA receive buffers */

#ifdef CONFIG_16550_UART0_DMA_RXBUFSIZE
static char g_uart0dmarxbuf[CONFIG_16550_UART0_DMA_RXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART1_DMA_RXBUFSIZE
static char g_uart1dmarxbuf[CONFIG_16550_UART1_DMA_RXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART2_DMA_RXBUFSIZE
static char g_uart2dmarxbuf[CONFIG_16550_UART2_DMA_RXBUFSIZE];
#endif
#ifdef CONFIG_16550_UART3_DMA_RXBUFSIZE
static char g_uart3dmarxbuf[CONFIG_16550_UART3_DMA_RXBUFSIZE];
#endif

/* This describes the state of the 16550 uart0 port. */

#ifdef CONFIG_16550_UART0
static struct u16550_s g_uart0priv =
{
  .ops            = &g_u16550_ops,
  .uartbase       = CONFIG_16550_UART0_BASE,
  .regincr        = CONFIG_16550_REGINCR,
#ifdef CONFIG_16550_UART0_DMA
  .dmatx          = CONFIG_16550_UART0_DMA_TX,
  .dmarx          = CONFIG_16550_UART0_DMA_RX,
#  if CONFIG_16550_UART0_DMA_RX != -1
  .dmarxbuf       = g_uart0dmarxbuf,
  .dmarxsize      = CONFIG_16550_UART0_DMA_RXBUFSIZE,
  .dmarxtimeout   = CONFIG_16550_UART0_DMA_RXTIMEOUT,
#  endif
#elif defined(HAVE_16550_UART_DMA)
  .dmatx          = -1,
  .dmarx          = -1,
#endif
#if !defined(CONFIG_16550_SUPRESS_CONFIG) || defined(CONFIG_16550_UART0_DMA)
  .baud           = CONFIG_16550_UART0_BAUD,
  .uartclk        = CONFIG_16550_UART0_CLOCK,
#endif
#ifdef CONFIG_CLK
  .clk_name       = CONFIG_16550_UART0_CLOCK_NAME,
#endif
  .irq            = CONFIG_16550_UART0_IRQ,
#ifndef CONFIG_16550_SUPRESS_CONFIG
  .parity         = CONFIG_16550_UART0_PARITY,
  .bits           = CONFIG_16550_UART0_BITS,
  .stopbits2      = CONFIG_16550_UART0_2STOP,
#if defined(CONFIG_16550_UART0_IFLOWCONTROL) || defined(CONFIG_16550_UART0_OFLOWCONTROL)
  .flow           = true,
#endif
#endif
  .rxtrigger      = CONFIG_16550_UART0_RX_TRIGGER,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_16550_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

/* This describes the state of the 16550 uart1 port. */

#ifdef CONFIG_16550_UART1
static struct u16550_s g_uart1priv =
{
  .ops            = &g_u16550_ops,
  .uartbase       = CONFIG_16550_UART1_BASE,
  .regincr        = CONFIG_16550_REGINCR,
#ifdef CONFIG_16550_UART1_DMA
  .dmatx          = CONFIG_16550_UART1_DMA_TX,
  .dmarx          = CONFIG_16550_UART1_DMA_RX,
#  if CONFIG_16550_UART1_DMA_RX != -1
  .dmarxbuf       = g_uart1dmarxbuf,
  .dmarxsize      = CONFIG_16550_UART1_DMA_RXBUFSIZE,
  .dmarxtimeout   = CONFIG_16550_UART1_DMA_RXTIMEOUT,
#  endif
#elif defined(HAVE_16550_UART_DMA)
  .dmatx          = -1,
  .dmarx          = -1,
#endif
#if !defined(CONFIG_16550_SUPRESS_CONFIG) || defined(CONFIG_16550_UART1_DMA)
  .baud           = CONFIG_16550_UART1_BAUD,
  .uartclk        = CONFIG_16550_UART1_CLOCK,
#endif
#ifdef CONFIG_CLK
  .clk_name       = CONFIG_16550_UART1_CLOCK_NAME,
#endif
  .irq            = CONFIG_16550_UART1_IRQ,
#ifndef CONFIG_16550_SUPRESS_CONFIG
  .parity         = CONFIG_16550_UART1_PARITY,
  .bits           = CONFIG_16550_UART1_BITS,
  .stopbits2      = CONFIG_16550_UART1_2STOP,
#if defined(CONFIG_16550_UART1_IFLOWCONTROL) || defined(CONFIG_16550_UART1_OFLOWCONTROL)
  .flow           = true,
#endif
#endif
  .rxtrigger      = CONFIG_16550_UART1_RX_TRIGGER,
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_16550_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif

/* This describes the state of the 16550 uart1 port. */

#ifdef CONFIG_16550_UART2
static struct u16550_s g_uart2priv =
{
  .ops            = &g_u16550_ops,
  .uartbase       = CONFIG_16550_UART2_BASE,
  .regincr        = CONFIG_16550_REGINCR,
#ifdef CONFIG_16550_UART2_DMA
  .dmatx          = CONFIG_16550_UART2_DMA_TX,
  .dmarx          = CONFIG_16550_UART2_DMA_RX,
#  if CONFIG_16550_UART2_DMA_RX != -1
  .dmarxbuf       = g_uart2dmarxbuf,
  .dmarxsize      = CONFIG_16550_UART2_DMA_RXBUFSIZE,
  .dmarxtimeout   = CONFIG_16550_UART2_DMA_RXTIMEOUT,
#  endif
#elif defined(HAVE_16550_UART_DMA)
  .dmatx          = -1,
  .dmarx          = -1,
#endif
#if !defined(CONFIG_16550_SUPRESS_CONFIG) || defined(CONFIG_16550_UART2_DMA)
  .baud           = CONFIG_16550_UART2_BAUD,
  .uartclk        = CONFIG_16550_UART2_CLOCK,
#endif
#ifdef CONFIG_CLK
  .clk_name       = CONFIG_16550_UART2_CLOCK_NAME,
#endif
  .irq            = CONFIG_16550_UART2_IRQ,
#ifndef CONFIG_16550_SUPRESS_CONFIG
  .parity         = CONFIG_16550_UART2_PARITY,
  .bits           = CONFIG_16550_UART2_BITS,
  .stopbits2      = CONFIG_16550_UART2_2STOP,
#if defined(CONFIG_16550_UART2_IFLOWCONTROL) || defined(CONFIG_16550_UART2_OFLOWCONTROL)
  .flow           = true,
#endif
#endif
  .rxtrigger      = CONFIG_16550_UART2_RX_TRIGGER,
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_16550_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

/* This describes the state of the 16550 uart1 port. */

#ifdef CONFIG_16550_UART3
static struct u16550_s g_uart3priv =
{
  .ops            = &g_u16550_ops,
  .uartbase       = CONFIG_16550_UART3_BASE,
  .regincr        = CONFIG_16550_REGINCR,
#ifdef CONFIG_16550_UART3_DMA
  .dmatx          = CONFIG_16550_UART3_DMA_TX,
  .dmarx          = CONFIG_16550_UART3_DMA_RX,
#  if CONFIG_16550_UART3_DMA_RX != -1
  .dmarxbuf       = g_uart3dmarxbuf,
  .dmarxsize      = CONFIG_16550_UART3_DMA_RXBUFSIZE,
  .dmarxtimeout   = CONFIG_16550_UART3_DMA_RXTIMEOUT,
#  endif
#elif defined(HAVE_16550_UART_DMA)
  .dmatx          = -1,
  .dmarx          = -1,
#endif
#if !defined(CONFIG_16550_SUPRESS_CONFIG) || defined(CONFIG_16550_UART3_DMA)
  .baud           = CONFIG_16550_UART3_BAUD,
  .uartclk        = CONFIG_16550_UART3_CLOCK,
#endif
#ifdef CONFIG_CLK
  .clk_name       = CONFIG_16550_UART3_CLOCK_NAME,
#endif
  .irq            = CONFIG_16550_UART3_IRQ,
#ifndef CONFIG_16550_SUPRESS_CONFIG
  .parity         = CONFIG_16550_UART3_PARITY,
  .bits           = CONFIG_16550_UART3_BITS,
  .stopbits2      = CONFIG_16550_UART3_2STOP,
#if defined(CONFIG_16550_UART3_IFLOWCONTROL) || defined(CONFIG_16550_UART3_OFLOWCONTROL)
  .flow           = true,
#endif
#endif
  .rxtrigger      = CONFIG_16550_UART3_RX_TRIGGER,
};

static uart_dev_t g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_16550_UART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_16550_UART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};
#endif

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#ifdef CONFIG_16550_SERIAL_DISABLE_REORDERING

#  if defined(CONFIG_16550_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port    /* UART0=console */
#  elif defined(CONFIG_16550_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port    /* UART1=console */
#  elif defined(CONFIG_16550_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2port    /* UART2=console */
#  elif defined(CONFIG_16550_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3port    /* UART3=console */
#  endif

#  ifdef CONFIG_16550_UART0
#    define TTYS0_DEV       g_uart0port
#  endif

#  ifdef CONFIG_16550_UART1
#    define TTYS1_DEV       g_uart1port
#  endif

#  ifdef CONFIG_16550_UART2
#    define TTYS2_DEV       g_uart2port
#  endif

#  ifdef CONFIG_16550_UART3
#    define TTYS3_DEV       g_uart3port
#  endif

#else  /* CONFIG_16550_SERIAL_DISABLE_REORDERING */

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#  if defined(CONFIG_16550_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port    /* UART0=console */
#    define TTYS0_DEV       g_uart0port    /* UART0=ttyS0 */
#    ifdef CONFIG_16550_UART1
#      define TTYS1_DEV     g_uart1port    /* UART0=ttyS0;UART1=ttyS1 */
#      ifdef CONFIG_16550_UART2
#        define TTYS2_DEV   g_uart2port    /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2 */
#        ifdef CONFIG_16550_UART3
#          define TTYS3_DEV g_uart3port    /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_16550_UART3
#          define TTYS2_DEV g_uart3port    /* UART0=ttyS0;UART1=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART0=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_16550_UART2
#        define TTYS1_DEV   g_uart2port    /* UART0=ttyS0;UART2=ttyS1;No ttyS3 */
#        ifdef CONFIG_16550_UART3
#          define TTYS2_DEV g_uart3port    /* UART0=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART0=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_16550_UART3
#          define TTYS1_DEV g_uart3port    /* UART0=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* UART0=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#          undef TTYS2_DEV                 /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_16550_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port    /* UART1=console */
#    define TTYS0_DEV       g_uart1port    /* UART1=ttyS0 */
#    ifdef CONFIG_16550_UART0
#      define TTYS1_DEV     g_uart0port    /* UART1=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_16550_UART2
#        define TTYS2_DEV   g_uart2port    /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2 */
#        ifdef CONFIG_16550_UART3
#          define TTYS3_DEV g_uart3port    /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_16550_UART3
#          define TTYS2_DEV g_uart3port    /* UART1=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART1=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_16550_UART2
#        define TTYS1_DEV   g_uart2port    /* UART1=ttyS0;UART2=ttyS1 */
#        ifdef CONFIG_16550_UART3
#          define TTYS2_DEV g_uart3port    /* UART1=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART1=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_16550_UART3
#          define TTYS1_DEV   g_uart3port  /* UART1=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* UART1=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                   /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_16550_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2port    /* UART2=console */
#    define TTYS0_DEV       g_uart2port    /* UART2=ttyS0 */
#    ifdef CONFIG_16550_UART0
#      define TTYS1_DEV     g_uart0port    /* UART2=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_16550_UART1
#        define TTYS2_DEV   g_uart1port    /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#        ifdef CONFIG_16550_UART3
#          define TTYS3_DEV g_uart3port    /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_16550_UART3
#          define TTYS2_DEV g_uart3port    /* UART2=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART2=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_16550_UART1
#        define TTYS1_DEV   g_uart1port    /* UART2=ttyS0;UART1=ttyS1 */
#        ifdef CONFIG_16550_UART3
#          define TTYS2_DEV g_uart3port    /* UART2=ttyS0;UART1=ttyS1;UART3=ttyS2 */
#        else
#          undef TTYS2_DEV                 /* UART2=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_16550_UART3
#          define TTYS1_DEV g_uart3port    /* UART2=ttyS0;UART3=ttyS1;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* UART2=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                   /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_16550_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3port    /* UART3=console */
#    define TTYS0_DEV       g_uart3port    /* UART3=ttyS0 */
#    ifdef CONFIG_16550_UART0
#      define TTYS1_DEV     g_uart0port    /* UART3=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_16550_UART1
#        define TTYS2_DEV   g_uart1port    /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#        ifdef CONFIG_16550_UART2
#          define TTYS3_DEV g_uart2port    /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2;UART2=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_16550_UART2
#          define TTYS2_DEV g_uart2port    /* UART3=ttyS0;UART0=ttyS1;UART2=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART3=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#          undef TTYS3_DEV                 /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_16550_UART1
#        define TTYS1_DEV   g_uart1port    /* UART3=ttyS0;UART1=ttyS1 */
#        ifdef CONFIG_16550_UART2
#          define TTYS2_DEV g_uart2port    /* UART3=ttyS0;UART1=ttyS1;UART2=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART3=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_16550_UART2
#          define TTYS1_DEV   g_uart2port  /* UART3=ttyS0;UART2=ttyS1;No ttyS3;No ttyS3 */
#          undef TTYS3_DEV                 /* UART3=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* UART3=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                   /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  endif

#endif /* CONFIG_16550_SERIAL_DISABLE_REORDERING */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SERIAL_UART_ARCH_MMIO
/****************************************************************************
 * Name: u16550_mmio_getreg
 ****************************************************************************/

static uart_datawidth_t u16550_mmio_getreg(FAR struct u16550_s *priv,
                                           unsigned int offset)
{
  uintptr_t addr = priv->uartbase + offset;
  return *((FAR volatile uart_datawidth_t *)addr);
}

/****************************************************************************
 * Name: u16550_mmio_putreg
 ****************************************************************************/

static void u16550_mmio_putreg(FAR struct u16550_s *priv,
                               unsigned int offset,
                               uart_datawidth_t value)
{
  uintptr_t addr = priv->uartbase + offset;
  *((FAR volatile uart_datawidth_t *)addr) = value;
}
#endif

/****************************************************************************
 * Name: u16550_serialin
 ****************************************************************************/

static inline uart_datawidth_t u16550_serialin(FAR struct u16550_s *priv,
                                               int offset)
{
  /* Get correct offset */

  offset *= (priv->regincr * sizeof(uart_datawidth_t));
  return priv->ops->getreg(priv, offset);
}

/****************************************************************************
 * Name: u16550_serialout
 ****************************************************************************/

static inline void u16550_serialout(FAR struct u16550_s *priv, int offset,
                                    uart_datawidth_t value)
{
  /* Get correct offset */

  offset *= (priv->regincr * sizeof(uart_datawidth_t));
  priv->ops->putreg(priv, offset, value);
}

#ifdef CONFIG_16550_WAIT_LCR
/****************************************************************************
 * Name: u16550_wait
 *
 * Description:
 *   Wait until UART is not busy. This is needed before writing to LCR.
 *   Otherwise we will get spurious interrupts on Synopsys DesignWare 8250.
 *
 * Input Parameters:
 *   priv: UART Struct
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ****************************************************************************/

static int u16550_wait(FAR struct u16550_s *priv)
{
  int i;

  for (i = 0; i < UART_TIMEOUT_MS; i++)
    {
      uint32_t status = u16550_serialin(priv, UART_USR_OFFSET);

      if ((status & UART_USR_BUSY) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  _err("UART timeout\n");
  return ERROR;
}
#endif /* CONFIG_16550_WAIT_LCR */

/****************************************************************************
 * Name: u16550_disableuartint
 ****************************************************************************/

static inline void u16550_disableuartint(FAR struct u16550_s *priv,
                                         FAR uart_datawidth_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLIE;
    }

  priv->ier &= ~UART_IER_ALLIE;
  u16550_serialout(priv, UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: u16550_enablebreaks
 ****************************************************************************/

static inline void u16550_enablebreaks(FAR struct u16550_s *priv,
                                       bool enable)
{
  uint32_t lcr = u16550_serialin(priv, UART_LCR_OFFSET);

  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }

#ifdef CONFIG_16550_WAIT_LCR
  /* Wait till UART is not busy before setting LCR */

  if (u16550_wait(priv) < 0)
    {
      _err("UART wait failed\n");
    }
#endif /* CONFIG_16550_WAIT_LCR */

  u16550_serialout(priv, UART_LCR_OFFSET, lcr);
}

/****************************************************************************
 * Name: u16550_divisor
 *
 * Description:
 *   Select a divider to produce the BAUD from the UART_CLK.
 *
 *     BAUD = UART_CLK / (16 * DL), or
 *     DIV  = UART_CLK / BAUD / 16
 *
 *   Ignoring the fractional divider for now.
 *
 ****************************************************************************/

#ifndef CONFIG_16550_SUPRESS_CONFIG
static inline uint32_t u16550_divisor(FAR struct u16550_s *priv)
{
  uint32_t base = 16 * priv->baud;
  uint32_t quot = priv->uartclk / base;
  uint32_t rem  = priv->uartclk % base;
  uint32_t frac = ((rem << CONFIG_16550_DLF_SIZE) + base / 2) / base;

#if CONFIG_16550_DLF_SIZE != 0
  return quot | (frac << 16);
#else
  return quot + frac;
#endif
}
#endif

/****************************************************************************
 * Name: u16550_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int u16550_setup(FAR struct uart_dev_s *dev)
{
#ifndef CONFIG_16550_SUPRESS_CONFIG
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  uint32_t div;
  uint32_t lcr;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL) || \
    defined(CONFIG_16550_SET_MCR_OUT2)
  uint32_t mcr;
#endif

  if (priv->uartbase == 0)
    {
      /* Device must be initialized */

      return -EPERM;
    }

  /* Clear fifos */

  u16550_serialout(priv, UART_FCR_OFFSET,
                   (UART_FCR_RXRST | UART_FCR_TXRST));

  /* Set up the IER */

  priv->ier = u16550_serialin(priv, UART_IER_OFFSET);

  /* Set up the LCR */

  lcr = 0;
  switch (priv->bits)
    {
      case 5 :
        lcr |= UART_LCR_WLS_5BIT;
        break;

      case 6 :
        lcr |= UART_LCR_WLS_6BIT;
        break;

      case 7 :
        lcr |= UART_LCR_WLS_7BIT;
        break;

      default:
      case 8 :
        lcr |= UART_LCR_WLS_8BIT;
        break;
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STB;
    }

  if (priv->parity == 1)
    {
      lcr |= UART_LCR_PEN;
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

#ifdef CONFIG_16550_WAIT_LCR
  /* Wait till UART is not busy before setting LCR */

  if (u16550_wait(priv) < 0)
    {
      _err("UART wait failed\n");
      return ERROR;
    }
#endif /* CONFIG_16550_WAIT_LCR */

  /* Enter DLAB=1 */

  u16550_serialout(priv, UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  /* Set the BAUD divisor */

  div = u16550_divisor(priv);
#if CONFIG_16550_DLF_SIZE != 0
  u16550_serialout(priv, UART_DLF_OFFSET, (div >> 16) & 0xff);
#endif
  u16550_serialout(priv, UART_DLM_OFFSET, (div >>  8) & 0xff);
  u16550_serialout(priv, UART_DLL_OFFSET, div & 0xff);

#ifdef CONFIG_16550_WAIT_LCR
  /* Wait till UART is not busy before setting LCR */

  if (u16550_wait(priv) < 0)
    {
      _err("UART wait failed\n");
      return ERROR;
    }
#endif /* CONFIG_16550_WAIT_LCR */

  /* Clear DLAB */

  u16550_serialout(priv, UART_LCR_OFFSET, lcr);

  /* Configure the FIFOs */

  u16550_serialout(priv, UART_FCR_OFFSET,
                   (priv->rxtrigger << UART_FCR_RXTRIGGER_SHIFT |
                    UART_FCR_TXRST | UART_FCR_RXRST |
                    UART_FCR_FIFOEN));

#ifdef CONFIG_16550_SET_MCR_OUT2
  /* Set OUT2 bit in MCR register */

  mcr = u16550_serialin(priv, UART_MCR_OFFSET);
  u16550_serialout(priv, UART_MCR_OFFSET, mcr | UART_MCR_OUT2);
#endif

  /* Set up the auto flow control */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  mcr = u16550_serialin(priv, UART_MCR_OFFSET);
  if (priv->flow)
    {
      mcr |= UART_MCR_AFCE;
    }
  else
    {
      mcr &= ~UART_MCR_AFCE;
    }

  mcr |= UART_MCR_RTS;

  u16550_serialout(priv, UART_MCR_OFFSET, mcr);
#endif /* defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL) */

  /* Reconfigure DMA Rx timeout value */

#ifdef HAVE_16550_UART_DMA
  u16550_dmarxconfig(dev);
#endif

#endif
  return OK;
}

/****************************************************************************
 * Name: u16550_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void u16550_shutdown(struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  u16550_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: u16550_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
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

static int u16550_attach(struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  int ret;

#ifdef CONFIG_CLK
  /* Clk enable */

  priv->mclk = clk_get(priv->clk_name);
  if (priv->mclk)
    {
      clk_set_rate(priv->mclk, priv->uartclk);
      clk_enable(priv->mclk);
    }
#endif

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, priv->ops->isr, dev);
#ifndef CONFIG_ARCH_NOINTC
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);

#ifdef HAVE_16550_UART_DMA
      if (priv->chanrx)
        {
          DMA_RESUME(priv->chanrx);
        }
#endif
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: u16550_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void u16550_detach(FAR struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;

#ifdef HAVE_16550_UART_DMA
  if (priv->chanrx)
    {
      DMA_PAUSE(priv->chanrx);
    }
#endif

  up_disable_irq(priv->irq);
  irqchain_detach(priv->irq, priv->ops->isr, dev);

#ifdef CONFIG_CLK
  /* Clk disaable */

  if (priv->mclk)
    {
      clk_disable(priv->mclk);
    }
#endif
}

/****************************************************************************
 * Name: u16550_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int u16550_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode    = filep->f_inode;
  FAR struct uart_dev_s *dev = inode->i_private;
  FAR struct u16550_s *priv  = (FAR struct u16550_s *)dev->priv;
  int ret;

#ifdef CONFIG_SERIAL_UART_ARCH_IOCTL
  ret = priv->ops->ioctl(priv, cmd, arg);

  if (ret != -ENOTTY)
    {
      return ret;
    }

#else
  ret = OK;
#endif

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        FAR struct u16550_s *user = (FAR struct u16550_s *)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct u16550_s));
          }
      }
      break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = enter_critical_section();
        u16550_enablebreaks(priv, true);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = enter_critical_section();
        u16550_enablebreaks(priv, false);
        leave_critical_section(flags);
      }
      break;

#if defined(CONFIG_SERIAL_TERMIOS) && !defined(CONFIG_16550_SUPRESS_CONFIG)
    case TCGETS:
      {
        FAR struct termios *termiosp = (FAR struct termios *)arg;
        irqstate_t flags;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        flags = enter_critical_section();

        cfsetispeed(termiosp, priv->baud);
        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);
        termiosp->c_cflag |= (priv->stopbits2) ? CSTOPB : 0;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        termiosp->c_cflag |= priv->flow ? CRTSCTS : 0;
#endif

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

          case 8:
          default:
            termiosp->c_cflag |= CS8;
            break;
          }

        leave_critical_section(flags);
      }
      break;

    case TCSETS:
      {
        FAR struct termios *termiosp = (FAR struct termios *)arg;
        irqstate_t flags;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        flags = enter_critical_section();

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            priv->bits = 5;
            break;

          case CS6:
            priv->bits = 6;
            break;

          case CS7:
            priv->bits = 7;
            break;

          case CS8:
          default:
            priv->bits = 8;
            break;
          }

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        priv->baud      = cfgetispeed(termiosp);
        priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        priv->flow      = (termiosp->c_cflag & CRTSCTS) != 0;
#endif

        u16550_setup(dev);
        leave_critical_section(flags);

#ifdef CONFIG_CLK
        /* Clk enable */

        priv->mclk = clk_get(priv->clk_name);
        if (priv->mclk)
          {
            clk_set_rate(priv->mclk, priv->uartclk);
          }
#endif
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
 * Name: u16550_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int u16550_receive(struct uart_dev_s *dev, unsigned int *status)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  uint32_t rbr;

  *status = u16550_serialin(priv, UART_LSR_OFFSET);
  rbr     = u16550_serialin(priv, UART_RBR_OFFSET);
  return rbr;
}

/****************************************************************************
 * Name: u16550_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void u16550_rxint(struct uart_dev_s *dev, bool enable)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;

#ifdef HAVE_16550_UART_DMA
  if (priv->chanrx)
    {
      return; /* Monitor DMA interrupt instead */
    }
#endif

  if (enable)
    {
      priv->ier |= UART_IER_ERBFI;
    }
  else
    {
      priv->ier &= ~UART_IER_ERBFI;
    }

  u16550_serialout(priv, UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: u16550_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool u16550_rxavailable(struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  return ((u16550_serialin(priv, UART_LSR_OFFSET) & UART_LSR_DR) != 0);
}

/****************************************************************************
 * Name: u16550_dma*
 *
 * Description:
 *   Stubbed out DMA-related methods
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool u16550_rxflowcontrol(struct uart_dev_s *dev,
                                 unsigned int nbuffered, bool upper)
{
#ifndef CONFIG_16550_SUPRESS_CONFIG
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;

  if (priv->flow)
    {
#ifdef HAVE_16550_UART_DMA
      if (priv->chanrx)
        {
          /* Pause Rx DMA receive to prevent more data being from
           * peripheral if the RX buffer is near full. When hardware
           * RTS is enabled, this will prevent more data from coming
           * in. Otherwise, Resume Rx DMA to make sure that more
           * input is received.
           */

          if (upper)
            {
              DMA_PAUSE(priv->chanrx);
            }
          else
            {
              DMA_RESUME(priv->chanrx);
            }
        }
      else
#endif
        {
          /* Disable Rx interrupt to prevent more data being from
           * peripheral if the RX buffer is near full. When hardware
           * RTS is enabled, this will prevent more data from coming
           * in. Otherwise, enable Rx interrupt to make sure that more
           * input is received.
           */

          u16550_rxint(dev, !upper);
        }

      return true;
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: u16550_dma*
 *
 * Description:
 *   Stub functions used when serial DMA is enabled.
 *
 ****************************************************************************/

#ifdef HAVE_16550_UART_DMA
static void u16550_dmasend_done(FAR struct dma_chan_s *chan,
                                FAR void *arg, ssize_t len)
{
  FAR struct uart_dev_s *dev = arg;

  if (len > 0)
    {
      dev->dmatx.nbytes = len;
      uart_xmitchars_done(dev);
      uart_xmitchars_dma(dev);
    }
  else /* Fail, resend */
    {
      u16550_dmasend(dev);
    }
}

static void u16550_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = dev->priv;
  FAR void *buffer = dev->dmatx.buffer;
  size_t length = dev->dmatx.length;

  up_clean_dcache((uintptr_t)buffer, (uintptr_t)buffer + length);
  DMA_START(priv->chantx, u16550_dmasend_done, dev,
            up_addrenv_va_to_pa((FAR void *)priv->uartbase),
            up_addrenv_va_to_pa(buffer), length);
}

static void u16550_dmareceive_done(FAR struct dma_chan_s *chan,
                                   FAR void *arg, ssize_t len)
{
  FAR struct uart_dev_s *dev = arg;
  FAR struct u16550_s *priv = dev->priv;

  if (len >= 0)
    {
      size_t slot   = priv->dmarxhead / priv->dmarxsize;
      size_t offset = priv->dmarxhead - slot * priv->dmarxsize;

      if (len >= priv->dmarxsize)
        {
          len = 0;
        }

      if (len < offset)
        {
          slot++; /* Wrap, move to the next slot */
        }

      priv->dmarxhead = slot * priv->dmarxsize + len;
      if (priv->dmarxhead - priv->dmarxtail >= priv->dmarxsize)
        {
          serr("The receive dma buffer is overrun\n");
          priv->dmarxtail = priv->dmarxhead - priv->dmarxsize / 2;
        }

      /* The receive isn't in the process? */

      if (dev->dmarx.length == 0)
        {
          /* Trigger the receive process */

          uart_recvchars_dma(dev);
        }
      else
        {
          /* Copy the received data */

          u16550_dmareceive(dev);
        }
    }
}

static void u16550_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = dev->priv;

  if (priv->dmarxhead != priv->dmarxtail)
    {
      size_t length = priv->dmarxhead - priv->dmarxtail;
      size_t offset = priv->dmarxtail % priv->dmarxsize;
      FAR char *buffer = priv->dmarxbuf + offset;

      if (offset + length > priv->dmarxsize)
        {
          length = priv->dmarxsize - offset;
        }

      if (length > dev->dmarx.length)
        {
          length = dev->dmarx.length;
        }

      up_invalidate_dcache((uintptr_t)buffer, (uintptr_t)buffer + length);
      memcpy(dev->dmarx.buffer, buffer, length);
      dev->dmarx.nbytes = length;
      priv->dmarxtail += length;

      uart_recvchars_done(dev);
      if (priv->dmarxhead != priv->dmarxtail)
        {
          /* Trigger the receive process again */

          uart_recvchars_dma(dev);
        }
    }
}

static void u16550_dmarxconfig(FAR struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = dev->priv;
  struct dma_config_s config;

  if (priv->chanrx != NULL)
    {
      memset(&config, 0, sizeof(config));
      config.direction = DMA_DEV_TO_MEM;

      /* 12bit = 1bit start + 8bit data + 1bit parity + 2bit stop */

      config.timeout   = 12 * 1000000ull * priv->dmarxtimeout / priv->baud;
      config.src_width = 1;
      DMA_CONFIG(priv->chanrx, &config);
    }
}

static void u16550_dmarxfree(FAR struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = dev->priv;

  if (priv->dmarx == -1)
    {
      return; /* Can't receive by DMA */
    }

  if (priv->chanrx == NULL)
    {
      priv->chanrx = priv->ops->dmachan(priv, priv->dmarx)
      if (priv->chanrx == NULL)
        {
          return; /* Fail to get DMA channel */
        }

      u16550_dmarxconfig(dev);

      /* Start a never stop DMA cyclic transfer in the background */

      DMA_START_CYCLIC(priv->chanrx, u16550_dmareceive_done, dev,
                       up_addrenv_va_to_pa(priv->dmarxbuf),
                       up_addrenv_va_to_pa((FAR void *)priv->uartbase),
                       priv->dmarxsize, priv->dmarxsize / 4);
    }

  /* The receive isn't in the process? */

  if (dev->dmarx.length == 0)
    {
      /* Trigger the receive process */

      uart_recvchars_dma(dev);
    }
}

static void u16550_dmatxconfig(FAR struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = dev->priv;
  struct dma_config_s config;

  if (priv->chantx != NULL)
    {
      memset(&config, 0, sizeof(config));
      config.direction = DMA_MEM_TO_DEV;
      config.dst_width = 1;
      DMA_CONFIG(priv->chantx, &config);
    }
}

static void u16550_dmatxavail(FAR struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = dev->priv;

  if (priv->dmatx == -1)
    {
      return; /* Can't send by DMA */
    }

  if (priv->chantx == NULL)
    {
      priv->chantx = priv->ops->dmachan(priv, priv->dmatx);
      if (priv->chantx == NULL)
        {
          return; /* Fail to get DMA channel */
        }

      u16550_dmatxconfig(dev);
    }

  /* DMA isn't busy for sending? */

  if (dev->dmatx.length == 0)
    {
      /* Start DMA for sending */

      uart_xmitchars_dma(dev);
    }
}
#endif

/****************************************************************************
 * Name: u16550_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void u16550_send(struct uart_dev_s *dev, int ch)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  u16550_serialout(priv, UART_THR_OFFSET, (uart_datawidth_t)ch);
}

/****************************************************************************
 * Name: u16550_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void u16550_txint(struct uart_dev_s *dev, bool enable)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  irqstate_t flags;

#ifdef HAVE_16550_UART_DMA
  if (priv->chantx)
    {
      return; /* Monitor DMA interrupt instead */
    }
#endif

  flags = enter_critical_section();
  if (enable)
    {
      priv->ier |= UART_IER_ETBEI;
      u16550_serialout(priv, UART_IER_OFFSET, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
    }
  else
    {
      priv->ier &= ~UART_IER_ETBEI;
      u16550_serialout(priv, UART_IER_OFFSET, priv->ier);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: u16550_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool u16550_txready(struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  return ((u16550_serialin(priv, UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Name: u16550_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool u16550_txempty(struct uart_dev_s *dev)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)dev->priv;
  return ((u16550_serialin(priv, UART_LSR_OFFSET) & UART_LSR_TEMT) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: u16550_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before uart_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by uart_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

void u16550_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
#ifndef CONFIG_16550_SUPRESS_INITIAL_CONFIG
  u16550_setup(&CONSOLE_DEV);
#endif
#endif
}

/****************************************************************************
 * Name: u16550_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   u16550_earlyserialinit was called previously.
 *
 ****************************************************************************/

void u16550_serialinit(void)
{
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
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
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

#ifdef HAVE_16550_CONSOLE
void up_putc(int ch)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)CONSOLE_DEV.priv;

  u16550_putc(priv, ch);
}
#endif

/****************************************************************************
 * Name: u16550_bind
 *
 * Description:
 *   Bind 16550 compatible device with this driver.
 *
 ****************************************************************************/

int u16550_bind(FAR uart_dev_t *dev)
{
  dev->ops = &g_uart_ops;

#ifdef CONFIG_16550_PCI_CONSOLE
  if (dev->isconsole)
    {
      /* Setup console device */

      u16550_setup(dev);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: u16550_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

int u16550_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  FAR struct u16550_s *priv;
  uint32_t status;
  int passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (FAR struct u16550_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

      status = u16550_serialin(priv, UART_IIR_OFFSET);

      /* The UART_IIR_INTSTATUS bit should be zero if there are pending
       * interrupts
       */

      if ((status & UART_IIR_INTSTATUS) != 0)
        {
          /* Break out of the loop when there is no longer a
           * pending interrupt
           */

          break;
        }

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_INTID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_INTID_RDA:
          case UART_IIR_INTID_CTI:
            {
              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_INTID_THRE:
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts (UART1 only) */

          case UART_IIR_INTID_MSI:
            {
              /* Read the modem status register (MSR) to clear */

              status = u16550_serialin(priv, UART_MSR_OFFSET);
              sinfo("MSR: %02"PRIx32"\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_INTID_RLS:
            {
              /* Read the line status register (LSR) to clear */

              status = u16550_serialin(priv, UART_LSR_OFFSET);
              sinfo("LSR: %02"PRIx32"\n", status);
              break;
            }

          /* There should be no other values */

          default:
            {
              serr("ERROR: Unexpected IIR: %02"PRIx32"\n", status);
              break;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: u16550_putc
 *
 * Description:
 *   Write one character to the UART (polled)
 *
 ****************************************************************************/

void u16550_putc(FAR struct u16550_s *priv, int ch)
{
  while ((u16550_serialin(priv, UART_LSR_OFFSET) & UART_LSR_THRE) == 0);
  u16550_serialout(priv, UART_THR_OFFSET, (uart_datawidth_t)ch);
}

#endif /* CONFIG_16550_UART */
