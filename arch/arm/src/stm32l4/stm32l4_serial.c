/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_serial.c
 *
 *   Copyright (C) 2009-2014, 2016-2017,2019 Gregory Nutt.
 *            All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david.sidrane@nscdg.com>
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/power/pm.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "stm32l4_gpio.h"
#include "stm32l4_uart.h"
#include "stm32l4_dma.h"
#include "stm32l4_rcc.h"
#include "arm_arch.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* DMA configuration */

/* If DMA is enabled on any USART, then very that other pre-requisites
 * have also been selected.
 * UART DMA1 DMA2
 *    1  X    X
 *    2  X
 *    3  X
 *    4       X
 *    5       X
 */

#ifdef SERIAL_HAVE_RXDMA

/* Verify that DMA has been enabled and the DMA channel has been defined.
 */

#  if defined(CONFIG_USART2_RXDMA) || defined(CONFIG_USART3_RXDMA)
#    if !defined(CONFIG_STM32L4_DMA1) && !defined(CONFIG_STM32L4_DMAMUX)
#      error STM32L4 USART2/3 receive DMA requires CONFIG_STM32L4_DMA1
#    endif
#  endif

#  if defined(CONFIG_UART4_RXDMA) || defined(CONFIG_UART5_RXDMA)
#    if !defined(CONFIG_STM32L4_DMA2) && !defined(CONFIG_STM32L4_DMAMUX)
#      error STM32L4 UART4/5 receive DMA requires CONFIG_STM32L4_DMA2
#    endif
#  endif

/* Currently RS-485 support cannot be enabled when RXDMA is in use due to
 * lack of testing - RS-485 support was developed on STM32F1x
 */

#  if (defined(CONFIG_USART1_RXDMA) && defined(CONFIG_USART1_RS485)) || \
      (defined(CONFIG_USART2_RXDMA) && defined(CONFIG_USART2_RS485)) || \
      (defined(CONFIG_USART3_RXDMA) && defined(CONFIG_USART3_RS485)) || \
      (defined(CONFIG_UART4_RXDMA) && defined(CONFIG_UART4_RS485))   || \
      (defined(CONFIG_UART5_RXDMA) && defined(CONFIG_UART5_RS485))
#    error "RXDMA and RS-485 cannot be enabled at the same time for the same U[S]ART"
#  endif

/* For the L4, there are alternate DMA channels for USART1.
 * Logic in the board.h file make the DMA channel selection by defining
 * the following in the board.h file.
 */

#  if defined(CONFIG_USART1_RXDMA) && !defined(DMAMAP_USART1_RX)
#    error "USART1 DMA channel not defined (DMAMAP_USART1_RX)"
#  endif

/* UART2-5 have no alternate channels without DMAMUX */

#  ifndef CONFIG_STM32L4_HAVE_DMAMUX
#    define DMAMAP_USART2_RX  DMACHAN_USART2_RX
#    define DMAMAP_USART3_RX  DMACHAN_USART3_RX
#    define DMAMAP_UART4_RX   DMACHAN_UART4_RX
#    define DMAMAP_UART5_RX   DMACHAN_UART5_RX
#  endif

#  if defined(CONFIG_USART2_RXDMA) && !defined(DMAMAP_USART2_RX)
#    error "USART2 DMA channel not defined (DMAMAP_USART2_RX)"
#  endif

#  if defined(CONFIG_USART3_RXDMA) && !defined(DMAMAP_USART3_RX)
#    error "USART3 DMA channel not defined (DMAMAP_USART3_RX)"
#  endif

#  if defined(CONFIG_UART4_RXDMA) && !defined(DMAMAP_UART4_RX)
#    error "UART4 DMA channel not defined (DMAMAP_UART4_RX)"
#  endif

#  if defined(CONFIG_UART5_RXDMA) && !defined(DMAMAP_UART5_RX)
#    error "UART5 DMA channel not defined (DMAMAP_UART5_RX)"
#  endif

/* The DMA buffer size when using RX DMA to emulate a FIFO.
 *
 * When streaming data, the generic serial layer will be called
 * every time the FIFO receives half this number of bytes.
 *
 * If there ever is a STM32L4 with D-cache, the buffer size
 * should be an even multiple of ARMV7M_DCACHE_LINESIZE, so that it
 * can be individually invalidated.
 */

#  if !defined(CONFIG_STM32L4_SERIAL_RXDMA_BUFFER_SIZE) || \
      CONFIG_STM32L4_SERIAL_RXDMA_BUFFER_SIZE == 0
#    define RXDMA_BUFFER_SIZE 32
#  else
#    define RXDMA_BUFFER_SIZE ((CONFIG_STM32L4_SERIAL_RXDMA_BUFFER_SIZE + 31) & ~31)
#  endif

/* DMA priority */

#  ifndef CONFIG_USART_RXDMAPRIO
#    define CONFIG_USART_RXDMAPRIO  DMA_CCR_PRIMED
#  endif
#  if (CONFIG_USART_RXDMAPRIO & ~DMA_CCR_PL_MASK) != 0
#    error "Illegal value for CONFIG_USART_RXDMAPRIO"
#  endif

/* DMA control words */

#  define SERIAL_DMA_CONTROL_WORD      \
              (DMA_CCR_CIRC          | \
               DMA_CCR_MINC          | \
               DMA_CCR_PSIZE_8BITS   | \
               DMA_CCR_MSIZE_8BITS   | \
               CONFIG_USART_RXDMAPRIO)
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
#    define SERIAL_DMA_IFLOW_CONTROL_WORD \
              (DMA_CCR_MINC          | \
               DMA_CCR_PSIZE_8BITS   | \
               DMA_CCR_MSIZE_8BITS   | \
               CONFIG_USART_RXDMAPRIO)
#  endif

#endif

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_STM32L4_PM_SERIAL_ACTIVITY)
#  define CONFIG_STM32L4_PM_SERIAL_ACTIVITY  10
#endif
#if defined(CONFIG_PM)
#  define PM_IDLE_DOMAIN             0 /* Revisit */
#endif

/* Keep track if a Break was set
 *
 * Note:
 *
 * 1) This value is set in the priv->ie but never written to the control
 *    register. It must not collide with USART_CR1_USED_INTS or USART_CR3_EIE
 * 2) USART_CR3_EIE is also carried in the up_dev_s ie member.
 *
 * See stm32l4serial_restoreusartint where the masking is done.
 */

#ifdef CONFIG_STM32L4_SERIALBRK_BSDCOMPAT
#  define USART_CR1_IE_BREAK_INPROGRESS_SHFTS 15
#  define USART_CR1_IE_BREAK_INPROGRESS (1 << USART_CR1_IE_BREAK_INPROGRESS_SHFTS)
#endif

#ifdef USE_SERIALDRIVER
#ifdef HAVE_UART

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32l4_serial_s
{
  struct uart_dev_s dev;       /* Generic UART device */
  uint16_t          ie;        /* Saved interrupt mask bits value */
  uint16_t          sr;        /* Saved status bits */

  /* Has been initialized and HW is setup. */

  bool              initialized;

#ifdef CONFIG_PM
  bool              suspended; /* UART device has been suspended. */

  /* Interrupt mask value stored before suspending for stop mode. */

  uint16_t          suspended_ie;
#endif

  /* If termios are supported, then the following fields may vary at
   * runtime.
   */

#ifdef CONFIG_SERIAL_TERMIOS
  uint8_t           parity;    /* 0=none, 1=odd, 2=even */
  uint8_t           bits;      /* Number of bits (7 or 8) */
  bool              stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool              iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool              oflow;     /* output flow control (CTS) enabled */
#endif
  uint32_t          baud;      /* Configured baud */
#else
  const uint8_t     parity;    /* 0=none, 1=odd, 2=even */
  const uint8_t     bits;      /* Number of bits (7 or 8) */
  const bool        stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const bool        iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const bool        oflow;     /* output flow control (CTS) enabled */
#endif
  const uint32_t    baud;      /* Configured baud */
#endif

  const uint8_t     irq;       /* IRQ associated with this USART */
  const uint32_t    apbclock;  /* PCLK 1 or 2 frequency */
  const uint32_t    usartbase; /* Base address of USART registers */
  const uint32_t    tx_gpio;   /* U[S]ART TX GPIO pin configuration */
  const uint32_t    rx_gpio;   /* U[S]ART RX GPIO pin configuration */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const uint32_t    rts_gpio;  /* U[S]ART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t    cts_gpio;  /* U[S]ART CTS GPIO pin configuration */
#endif

#ifdef SERIAL_HAVE_RXDMA
  const unsigned int rxdma_channel; /* DMA channel assigned */
#endif

  /* RX DMA state */

#ifdef SERIAL_HAVE_RXDMA
  DMA_HANDLE        rxdma;     /* currently-open receive DMA stream */
  bool              rxenable;  /* DMA-based reception en/disable */
#ifdef CONFIG_PM
  bool              rxdmasusp; /* Rx DMA suspended */
#endif
  uint32_t          rxdmanext; /* Next byte in the DMA buffer to be read */
  char       *const rxfifo;    /* Receive DMA buffer */
#endif

#ifdef HAVE_RS485
  const uint32_t    rs485_dir_gpio;     /* U[S]ART RS-485 DIR GPIO pin configuration */
  const bool        rs485_dir_polarity; /* U[S]ART RS-485 DIR pin state for TX enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void stm32l4serial_setformat(FAR struct uart_dev_s *dev);
#endif
static int  stm32l4serial_setup(FAR struct uart_dev_s *dev);
static void stm32l4serial_shutdown(FAR struct uart_dev_s *dev);
static int  stm32l4serial_attach(FAR struct uart_dev_s *dev);
static void stm32l4serial_detach(FAR struct uart_dev_s *dev);
static int  up_interrupt(int irq, FAR void *context, FAR void *arg);
static int  stm32l4serial_ioctl(FAR struct file *filep, int cmd,
                                unsigned long arg);
#ifndef SERIAL_HAVE_ONLY_DMA
static int  stm32l4serial_receive(FAR struct uart_dev_s *dev,
                                  FAR unsigned int *status);
static void stm32l4serial_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool stm32l4serial_rxavailable(FAR struct uart_dev_s *dev);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool stm32l4serial_rxflowcontrol(FAR struct uart_dev_s *dev,
                                        unsigned int nbuffered, bool upper);
#endif
static void stm32l4serial_send(FAR struct uart_dev_s *dev, int ch);
static void stm32l4serial_txint(FAR struct uart_dev_s *dev, bool enable);
static bool stm32l4serial_txready(FAR struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_RXDMA
static int  stm32l4serial_dmasetup(FAR struct uart_dev_s *dev);
static void stm32l4serial_dmashutdown(FAR struct uart_dev_s *dev);
static int  stm32l4serial_dmareceive(FAR struct uart_dev_s *dev,
                                     FAR unsigned int *status);
static void stm32l4serial_dmareenable(struct stm32l4_serial_s *priv);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool stm32l4serial_dmaiflowrestart(struct stm32l4_serial_s *priv);
#endif
static void stm32l4serial_dmarxint(FAR struct uart_dev_s *dev, bool enable);
static bool stm32l4serial_dmarxavailable(struct uart_dev_s *dev);

static void stm32l4serial_dmarxcallback(DMA_HANDLE handle, uint8_t status,
                                        FAR void *arg);
#endif

#ifdef CONFIG_PM
static void stm32l4serial_setsuspend(struct uart_dev_s *dev, bool suspend);
static void stm32l4serial_pm_setsuspend(bool suspend);
static void stm32l4serial_pmnotify(FAR struct pm_callback_s *cb, int domain,
                                   enum pm_state_e pmstate);
static int  stm32l4serial_pmprepare(FAR struct pm_callback_s *cb, int domain,
                                    enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static const struct uart_ops_s g_uart_ops =
{
  .setup          = stm32l4serial_setup,
  .shutdown       = stm32l4serial_shutdown,
  .attach         = stm32l4serial_attach,
  .detach         = stm32l4serial_detach,
  .ioctl          = stm32l4serial_ioctl,
  .receive        = stm32l4serial_receive,
  .rxint          = stm32l4serial_rxint,
  .rxavailable    = stm32l4serial_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = stm32l4serial_rxflowcontrol,
#endif
  .send           = stm32l4serial_send,
  .txint          = stm32l4serial_txint,
  .txready        = stm32l4serial_txready,
  .txempty        = stm32l4serial_txready,
};
#endif

#ifdef SERIAL_HAVE_RXDMA
static const struct uart_ops_s g_uart_dma_ops =
{
  .setup          = stm32l4serial_dmasetup,
  .shutdown       = stm32l4serial_dmashutdown,
  .attach         = stm32l4serial_attach,
  .detach         = stm32l4serial_detach,
  .ioctl          = stm32l4serial_ioctl,
  .receive        = stm32l4serial_dmareceive,
  .rxint          = stm32l4serial_dmarxint,
  .rxavailable    = stm32l4serial_dmarxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = stm32l4serial_rxflowcontrol,
#endif
  .send           = stm32l4serial_send,
  .txint          = stm32l4serial_txint,
  .txready        = stm32l4serial_txready,
  .txempty        = stm32l4serial_txready,
};
#endif

/* I/O buffers */

#ifdef CONFIG_STM32L4_USART1_SERIALDRIVER
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
# ifdef CONFIG_USART1_RXDMA
static char g_usart1rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32L4_USART2_SERIALDRIVER
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
# ifdef CONFIG_USART2_RXDMA
static char g_usart2rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32L4_USART3_SERIALDRIVER
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
# ifdef CONFIG_USART3_RXDMA
static char g_usart3rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32L4_UART4_SERIALDRIVER
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
# ifdef CONFIG_UART4_RXDMA
static char g_uart4rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32L4_UART5_SERIALDRIVER
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
# ifdef CONFIG_UART5_RXDMA
static char g_uart5rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

/* This describes the state of the STM32 USART1 ports. */

#ifdef CONFIG_STM32L4_USART1_SERIALDRIVER
static struct stm32l4_serial_s g_usart1priv =
{
  .dev =
    {
#if CONSOLE_UART == 1
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART1_RXBUFSIZE,
        .buffer  = g_usart1rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART1_TXBUFSIZE,
        .buffer  = g_usart1txbuffer,
      },
#ifdef CONFIG_USART1_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart1priv,
    },

  .irq           = STM32L4_IRQ_USART1,
  .parity        = CONFIG_USART1_PARITY,
  .bits          = CONFIG_USART1_BITS,
  .stopbits2     = CONFIG_USART1_2STOP,
  .baud          = CONFIG_USART1_BAUD,
  .apbclock      = STM32L4_PCLK2_FREQUENCY,
  .usartbase     = STM32L4_USART1_BASE,
  .tx_gpio       = GPIO_USART1_TX,
  .rx_gpio       = GPIO_USART1_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART1_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART1_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART1_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART1_RTS,
#endif
#ifdef CONFIG_USART1_RXDMA
  .rxdma_channel = DMAMAP_USART1_RX,
  .rxfifo        = g_usart1rxfifo,
#endif

#ifdef CONFIG_USART1_RS485
  .rs485_dir_gpio = GPIO_USART1_RS485_DIR,
#  if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 USART2 port. */

#ifdef CONFIG_STM32L4_USART2_SERIALDRIVER
static struct stm32l4_serial_s g_usart2priv =
{
  .dev =
    {
#if CONSOLE_UART == 2
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART2_RXBUFSIZE,
        .buffer  = g_usart2rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART2_TXBUFSIZE,
        .buffer  = g_usart2txbuffer,
      },
#ifdef CONFIG_USART2_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart2priv,
    },

  .irq           = STM32L4_IRQ_USART2,
  .parity        = CONFIG_USART2_PARITY,
  .bits          = CONFIG_USART2_BITS,
  .stopbits2     = CONFIG_USART2_2STOP,
  .baud          = CONFIG_USART2_BAUD,
  .apbclock      = STM32L4_PCLK1_FREQUENCY,
  .usartbase     = STM32L4_USART2_BASE,
  .tx_gpio       = GPIO_USART2_TX,
  .rx_gpio       = GPIO_USART2_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART2_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART2_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART2_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART2_RTS,
#endif
#ifdef CONFIG_USART2_RXDMA
  .rxdma_channel = DMAMAP_USART2_RX,
  .rxfifo        = g_usart2rxfifo,
#endif

#ifdef CONFIG_USART2_RS485
  .rs485_dir_gpio = GPIO_USART2_RS485_DIR,
#  if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 USART3 port. */

#ifdef CONFIG_STM32L4_USART3_SERIALDRIVER
static struct stm32l4_serial_s g_usart3priv =
{
  .dev =
    {
#if CONSOLE_UART == 3
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART3_RXBUFSIZE,
        .buffer  = g_usart3rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART3_TXBUFSIZE,
        .buffer  = g_usart3txbuffer,
      },
#ifdef CONFIG_USART3_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart3priv,
    },

  .irq           = STM32L4_IRQ_USART3,
  .parity        = CONFIG_USART3_PARITY,
  .bits          = CONFIG_USART3_BITS,
  .stopbits2     = CONFIG_USART3_2STOP,
  .baud          = CONFIG_USART3_BAUD,
  .apbclock      = STM32L4_PCLK1_FREQUENCY,
  .usartbase     = STM32L4_USART3_BASE,
  .tx_gpio       = GPIO_USART3_TX,
  .rx_gpio       = GPIO_USART3_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART3_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART3_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART3_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART3_RTS,
#endif
#ifdef CONFIG_USART3_RXDMA
  .rxdma_channel = DMAMAP_USART3_RX,
  .rxfifo        = g_usart3rxfifo,
#endif

#ifdef CONFIG_USART3_RS485
  .rs485_dir_gpio = GPIO_USART3_RS485_DIR,
#  if (CONFIG_USART3_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 UART4 port. */

#ifdef CONFIG_STM32L4_UART4_SERIALDRIVER
static struct stm32l4_serial_s g_uart4priv =
{
  .dev =
    {
#if CONSOLE_UART == 4
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_UART4_RXBUFSIZE,
        .buffer  = g_uart4rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_UART4_TXBUFSIZE,
        .buffer  = g_uart4txbuffer,
      },
#ifdef CONFIG_UART4_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_uart4priv,
    },

  .irq           = STM32L4_IRQ_UART4,
  .parity        = CONFIG_UART4_PARITY,
  .bits          = CONFIG_UART4_BITS,
  .stopbits2     = CONFIG_UART4_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART4_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_UART4_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART4_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_UART4_RTS,
#endif
  .baud          = CONFIG_UART4_BAUD,
  .apbclock      = STM32L4_PCLK1_FREQUENCY,
  .usartbase     = STM32L4_UART4_BASE,
  .tx_gpio       = GPIO_UART4_TX,
  .rx_gpio       = GPIO_UART4_RX,
#ifdef CONFIG_UART4_RXDMA
  .rxdma_channel = DMAMAP_UART4_RX,
  .rxfifo        = g_uart4rxfifo,
#endif

#ifdef CONFIG_UART4_RS485
  .rs485_dir_gpio = GPIO_UART4_RS485_DIR,
#  if (CONFIG_UART4_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 UART5 port. */

#ifdef CONFIG_STM32L4_UART5_SERIALDRIVER
static struct stm32l4_serial_s g_uart5priv =
{
  .dev =
    {
#if CONSOLE_UART == 5
      .isconsole = true,
#endif
      .recv     =
      {
        .size   = CONFIG_UART5_RXBUFSIZE,
        .buffer = g_uart5rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_UART5_TXBUFSIZE,
        .buffer = g_uart5txbuffer,
      },
#ifdef CONFIG_UART5_RXDMA
      .ops      = &g_uart_dma_ops,
#else
      .ops      = &g_uart_ops,
#endif
      .priv     = &g_uart5priv,
    },

  .irq            = STM32L4_IRQ_UART5,
  .parity         = CONFIG_UART5_PARITY,
  .bits           = CONFIG_UART5_BITS,
  .stopbits2      = CONFIG_UART5_2STOP,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART5_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_UART5_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART5_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_UART5_RTS,
#endif
  .baud           = CONFIG_UART5_BAUD,
  .apbclock       = STM32L4_PCLK1_FREQUENCY,
  .usartbase      = STM32L4_UART5_BASE,
  .tx_gpio        = GPIO_UART5_TX,
  .rx_gpio        = GPIO_UART5_RX,
#ifdef CONFIG_UART5_RXDMA
  .rxdma_channel = DMAMAP_UART5_RX,
  .rxfifo        = g_uart5rxfifo,
#endif

#ifdef CONFIG_UART5_RS485
  .rs485_dir_gpio = GPIO_UART5_RS485_DIR,
#  if (CONFIG_UART5_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This table lets us iterate over the configured USARTs */

FAR static struct stm32l4_serial_s *
const g_uart_devs[STM32L4_NUSART + STM32L4_NUART] =
{
#ifdef CONFIG_STM32L4_USART1_SERIALDRIVER
  [0] = &g_usart1priv,
#endif
#ifdef CONFIG_STM32L4_USART2_SERIALDRIVER
  [1] = &g_usart2priv,
#endif
#ifdef CONFIG_STM32L4_USART3_SERIALDRIVER
  [2] = &g_usart3priv,
#endif
#ifdef CONFIG_STM32L4_UART4_SERIALDRIVER
  [3] = &g_uart4priv,
#endif
#ifdef CONFIG_STM32L4_UART5_SERIALDRIVER
  [4] = &g_uart5priv,
#endif
};

#ifdef CONFIG_PM
static struct
{
  struct pm_callback_s pm_cb;
  bool serial_suspended;
} g_serialpm =
  {
    .pm_cb.notify  = stm32l4serial_pmnotify,
    .pm_cb.prepare = stm32l4serial_pmprepare,
    .serial_suspended = false
  };
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4serial_getreg
 ****************************************************************************/

static inline
uint32_t stm32l4serial_getreg(FAR struct stm32l4_serial_s *priv, int offset)
{
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: stm32l4serial_putreg
 ****************************************************************************/

static inline void stm32l4serial_putreg(FAR struct stm32l4_serial_s *priv,
                                        int offset, uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: stm32l4serial_setusartint
 ****************************************************************************/

static inline
void stm32l4serial_setusartint(FAR struct stm32l4_serial_s *priv,
                               uint16_t ie)
{
  uint32_t cr;

  /* Save the interrupt mask */

  priv->ie = ie;

  /* And restore the interrupt state (see the interrupt
   * enable/usage table above)
   */

  cr = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
  cr &= ~(USART_CR1_USED_INTS);
  cr |= (ie & (USART_CR1_USED_INTS));
  stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, cr);

  cr = stm32l4serial_getreg(priv, STM32L4_USART_CR3_OFFSET);
  cr &= ~USART_CR3_EIE;
  cr |= (ie & USART_CR3_EIE);
  stm32l4serial_putreg(priv, STM32L4_USART_CR3_OFFSET, cr);
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static void stm32l4serial_restoreusartint(FAR struct stm32l4_serial_s *priv,
                                          uint16_t ie)
{
  irqstate_t flags;

  flags = enter_critical_section();

  stm32l4serial_setusartint(priv, ie);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32l4serial_disableusartint
 ****************************************************************************/

static void stm32l4serial_disableusartint(FAR struct stm32l4_serial_s *priv,
                                          FAR uint16_t *ie)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (ie)
    {
      uint32_t cr1;
      uint32_t cr3;

      /* USART interrupts:
       *
       * Enable             Status          Meaning            Usage
       * ------------------ --------------- ------------------ ----------
       * USART_CR1_IDLEIE   USART_ISR_IDLE   Idle Line
       *                                     Detected          (not used)
       * USART_CR1_RXNEIE   USART_ISR_RXNE   Received Data
       *                                     Ready to be Read
       * "              "   USART_ISR_ORE    Overrun Error
       *                                     Detected
       * USART_CR1_TCIE     USART_ISR_TC     Transmission
       *                                     Complete          (used only
       *                                                        for RS-485)
       * USART_CR1_TXEIE    USART_ISR_TXE    Transmit Data
       *                                     Register Empty
       * USART_CR1_PEIE     USART_ISR_PE     Parity Error
       *
       * USART_CR2_LBDIE    USART_ISR_LBD    Break Flag        (not used)
       * USART_CR3_EIE      USART_ISR_FE     Framing Error
       * "           "      USART_ISR_NF     Noise Flag
       * "           "      USART_ISR_ORE    Overrun Error
       *                                     Detected
       * USART_CR3_CTSIE    USART_ISR_CTS    CTS flag          (not used)
       */

      cr1 = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
      cr3 = stm32l4serial_getreg(priv, STM32L4_USART_CR3_OFFSET);

      /* Return the current interrupt mask value for the used interrupts.
       * Notice that this depends on the fact that none of the used interrupt
       * enable bits overlap. This logic would fail if we needed the break
       * interrupt!
       */

      *ie = (cr1 & (USART_CR1_USED_INTS)) | (cr3 & USART_CR3_EIE);
    }

  /* Disable all interrupts */

  stm32l4serial_setusartint(priv, 0);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32l4serial_dmanextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int stm32l4serial_dmanextrx(FAR struct stm32l4_serial_s *priv)
{
  size_t dmaresidual;

  dmaresidual = stm32l4_dmaresidual(priv->rxdma);

  return (RXDMA_BUFFER_SIZE - (int)dmaresidual);
}
#endif

/****************************************************************************
 * Name: stm32l4serial_setformat
 *
 * Description:
 *   Set the serial line format and speed.
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void stm32l4serial_setformat(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  uint32_t regval;

  /* This first implementation is for U[S]ARTs that support oversampling
   * by 8 in additional to the standard oversampling by 16.
   */

  uint32_t usartdiv8;
  uint32_t cr1;
  uint32_t brr;

  /* In case of oversampling by 8, the equation is:
   *
   *   baud      = 2 * fCK / usartdiv8
   *   usartdiv8 = 2 * fCK / baud
   */

  usartdiv8 = ((priv->apbclock << 1) + (priv->baud >> 1)) / priv->baud;

  /* Baud rate for standard USART (SPI mode included):
   *
   * In case of oversampling by 16, the equation is:
   *   baud       = fCK / usartdiv16
   *   usartdiv16 = fCK / baud
   *              = 2 * usartdiv8
   */

  /* Use oversamply by 8 only if the divisor is small.  But what is small? */

  cr1 = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
  brr = stm32l4serial_getreg(priv, STM32L4_USART_BRR_OFFSET);
  brr &= ~(USART_BRR_MANT_MASK | USART_BRR_FRAC_MASK);

  if (usartdiv8 > 100)
    {
      /* Use usartdiv16 */

      brr  |= (usartdiv8 + 1) >> 1;

      /* Clear oversampling by 8 to enable oversampling by 16 */

      cr1 &= ~USART_CR1_OVER8;
    }
  else
    {
      DEBUGASSERT(usartdiv8 >= 8);

      /* Perform mysterious operations on bits 0-3 */

      brr  |= ((usartdiv8 & 0xfff0) | ((usartdiv8 & 0x000f) >> 1));

      /* Set oversampling by 8 */

      cr1 |= USART_CR1_OVER8;
    }

  stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, cr1);
  stm32l4serial_putreg(priv, STM32L4_USART_BRR_OFFSET, brr);

  /* Configure parity mode */

  regval  = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M0 | USART_CR1_M1);

  if (priv->parity == 1)       /* Odd parity */
    {
      regval |= (USART_CR1_PCE | USART_CR1_PS);
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      regval |= USART_CR1_PCE;
    }

  /* Configure word length (parity uses one of configured bits)
   *
   * Default: 1 start, 8 data (no parity), n stop, OR
   *          1 start, 7 data + parity, n stop
   */

  if (priv->bits == 9 || (priv->bits == 8 && priv->parity != 0))
    {
      /* Select: 1 start, 8 data + parity, n stop, OR
       *         1 start, 9 data (no parity), n stop.
       */

      regval |= USART_CR1_M0;
    }
  else if (priv->bits == 7 && priv->parity == 0)
    {
      /* Select: 1 start, 7 data (no parity), n stop, OR
       */

      regval |= USART_CR1_M1;
    }

  /* Else Select: 1 start, 7 data + parity, n stop, OR
   *              1 start, 8 data (no parity), n stop.
   */

  stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, regval);

  /* Configure STOP bits */

  regval = stm32l4serial_getreg(priv, STM32L4_USART_CR2_OFFSET);
  regval &= ~(USART_CR2_STOP_MASK);

  if (priv->stopbits2)
    {
      regval |= USART_CR2_STOP2;
    }

  stm32l4serial_putreg(priv, STM32L4_USART_CR2_OFFSET, regval);

  /* Configure hardware flow control */

  regval  = stm32l4serial_getreg(priv, STM32L4_USART_CR3_OFFSET);
  regval &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) && !defined(CONFIG_STM32L4_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {
      regval |= USART_CR3_RTSE;
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->oflow && (priv->cts_gpio != 0))
    {
      regval |= USART_CR3_CTSE;
    }
#endif

  stm32l4serial_putreg(priv, STM32L4_USART_CR3_OFFSET, regval);
}
#endif /* CONFIG_SUPPRESS_UART_CONFIG */

/****************************************************************************
 * Name: stm32l4serial_setsuspend
 *
 * Description:
 *   Suspend or resume serial peripheral.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void stm32l4serial_setsuspend(struct uart_dev_s *dev, bool suspend)
{
  FAR struct stm32l4_serial_s *priv = (struct stm32l4_serial_s *)dev->priv;
#ifdef SERIAL_HAVE_RXDMA
  bool dmarestored = false;
#endif

  if (priv->suspended == suspend)
    {
      return;
    }

  priv->suspended = suspend;

  if (suspend)
    {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          /* Force RTS high to prevent further Rx. */

          stm32l4_configgpio((priv->rts_gpio & ~GPIO_MODE_MASK)
                             | (GPIO_OUTPUT | GPIO_OUTPUT_SET));
        }
#endif

      /* Disable interrupts to prevent Tx. */

      stm32l4serial_disableusartint(priv, &priv->suspended_ie);

      /* Wait last Tx to complete. */

      while ((stm32l4serial_getreg(priv, STM32L4_USART_ISR_OFFSET) &
              USART_ISR_TC) == 0);

#ifdef SERIAL_HAVE_RXDMA
      if (priv->dev.ops == &g_uart_dma_ops && !priv->rxdmasusp)
        {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          if (priv->iflow && priv->rxdmanext == RXDMA_BUFFER_SIZE)
            {
              /* Rx DMA in non-circular iflow mode and already stopped
               * at end of DMA buffer. No need to suspend.
               */
            }
          else
#endif
            {
              /* Suspend Rx DMA. */

              stm32l4_dmastop(priv->rxdma);
              priv->rxdmasusp = true;
            }
        }
#endif
    }
  else
    {
#ifdef SERIAL_HAVE_RXDMA
      if (priv->dev.ops == &g_uart_dma_ops && priv->rxdmasusp)
        {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          if (priv->iflow)
            {
              stm32l4serial_dmaiflowrestart(priv);
            }
          else
#endif
            {
              /* This USART does not have HW flow-control. Unconditionally
               * re-enable DMA (might loss unprocessed bytes received
               * to DMA buffer before suspending).
               */

              stm32l4serial_dmareenable(priv);
              priv->rxdmasusp = false;
            }

          dmarestored = true;
        }
#endif

      /* Re-enable interrupts to resume Tx. */

      stm32l4serial_restoreusartint(priv, priv->suspended_ie);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          /* Restore peripheral RTS control. */

          stm32l4_configgpio(priv->rts_gpio);
        }
#endif
    }

#ifdef SERIAL_HAVE_RXDMA
  if (dmarestored)
    {
      irqstate_t flags;

      flags = enter_critical_section();

      /* Perform initial Rx DMA buffer fetch to wake-up serial device
       * activity.
       */

      if (priv->rxdma != NULL)
        {
          stm32l4serial_dmarxcallback(priv->rxdma, 0, priv);
        }

      leave_critical_section(flags);
    }
#endif
}
#endif

/****************************************************************************
 * Name: stm32l4serial_pm_setsuspend
 *
 * Description:
 *   Suspend or resume serial peripherals for/from deep-sleep/stop modes.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void stm32l4serial_pm_setsuspend(bool suspend)
{
  int n;

  /* Already in desired state? */

  if (suspend == g_serialpm.serial_suspended)
    return;

  g_serialpm.serial_suspended = suspend;

  for (n = 0; n < STM32L4_NUSART + STM32L4_NUART; n++)
    {
      struct stm32l4_serial_s *priv = g_uart_devs[n];

      if (!priv || !priv->initialized)
        {
          continue;
        }

      stm32l4serial_setsuspend(&priv->dev, suspend);
    }
}
#endif

/****************************************************************************
 * Name: stm32l4serial_setapbclock
 *
 * Description:
 *   Enable or disable APB clock for the USART peripheral
 *
 * Input Parameters:
 *   dev - A reference to the UART driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void stm32l4serial_setapbclock(FAR struct uart_dev_s *dev, bool on)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  uint32_t rcc_en;
  uint32_t regaddr;

  /* Determine which USART to configure */

  switch (priv->usartbase)
    {
    default:
      return;
#ifdef CONFIG_STM32L4_USART1_SERIALDRIVER
    case STM32L4_USART1_BASE:
      rcc_en = RCC_APB2ENR_USART1EN;
      regaddr = STM32L4_RCC_APB2ENR;
      break;
#endif
#ifdef CONFIG_STM32L4_USART2_SERIALDRIVER
    case STM32L4_USART2_BASE:
      rcc_en = RCC_APB1ENR1_USART2EN;
      regaddr = STM32L4_RCC_APB1ENR1;
      break;
#endif
#ifdef CONFIG_STM32L4_USART3_SERIALDRIVER
    case STM32L4_USART3_BASE:
      rcc_en = RCC_APB1ENR1_USART3EN;
      regaddr = STM32L4_RCC_APB1ENR1;
      break;
#endif
#ifdef CONFIG_STM32L4_UART4_SERIALDRIVER
    case STM32L4_UART4_BASE:
      rcc_en = RCC_APB1ENR1_UART4EN;
      regaddr = STM32L4_RCC_APB1ENR1;
      break;
#endif
#ifdef CONFIG_STM32L4_UART5_SERIALDRIVER
    case STM32L4_UART5_BASE:
      rcc_en = RCC_APB1ENR1_UART5EN;
      regaddr = STM32L4_RCC_APB1ENR1;
      break;
#endif
    }

  /* Enable/disable APB 1/2 clock for USART */

  if (on)
    {
      modifyreg32(regaddr, 0, rcc_en);
    }
  else
    {
      modifyreg32(regaddr, rcc_en, 0);
    }
}

/****************************************************************************
 * Name: stm32l4serial_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int stm32l4serial_setup(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t regval;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled in stm32l4_lowsetup().
   */

  /* Enable USART APB1/2 clock */

  stm32l4serial_setapbclock(dev, true);

  /* Configure pins for USART use */

  stm32l4_configgpio(priv->tx_gpio);
  stm32l4_configgpio(priv->rx_gpio);

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio != 0)
    {
      stm32l4_configgpio(priv->cts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio != 0)
    {
      uint32_t config = priv->rts_gpio;

#ifdef CONFIG_STM32L4_FLOWCONTROL_BROKEN
      /* Instead of letting hw manage this pin, we will bitbang */

      config = (config & ~GPIO_MODE_MASK) | GPIO_OUTPUT;
#endif
      stm32l4_configgpio(config);
    }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      stm32l4_configgpio(priv->rs485_dir_gpio);
      stm32l4_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
    }
#endif

  /* Configure CR2 */

  /* Clear STOP, CLKEN, CPOL, CPHA, LBCL, and interrupt enable bits */

  regval  = stm32l4serial_getreg(priv, STM32L4_USART_CR2_OFFSET);
  regval &= ~(USART_CR2_STOP_MASK | USART_CR2_CLKEN | USART_CR2_CPOL |
              USART_CR2_CPHA | USART_CR2_LBCL | USART_CR2_LBDIE);

  /* Configure STOP bits */

  if (priv->stopbits2)
    {
      regval |= USART_CR2_STOP2;
    }

  stm32l4serial_putreg(priv, STM32L4_USART_CR2_OFFSET, regval);

  /* Configure CR1 */

  /* Clear TE, REm and all interrupt enable bits */

  regval  = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_TE | USART_CR1_RE | USART_CR1_ALLINTS);

  stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, regval);

  /* Configure CR3 */

  /* Clear CTSE, RTSE, and all interrupt enable bits */

  regval  = stm32l4serial_getreg(priv, STM32L4_USART_CR3_OFFSET);
  regval &= ~(USART_CR3_CTSIE | USART_CR3_CTSE | USART_CR3_RTSE |
              USART_CR3_EIE);

  stm32l4serial_putreg(priv, STM32L4_USART_CR3_OFFSET, regval);

  /* Configure the USART line format and speed. */

  stm32l4serial_setformat(dev);

  /* Enable Rx, Tx, and the USART */

  regval      = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
  regval     |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
  stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, regval);

#endif /* CONFIG_SUPPRESS_UART_CONFIG */

  /* Set up the cached interrupt enables value */

  priv->ie    = 0;

  /* Mark device as initialized. */

  priv->initialized = true;

  return OK;
}

/****************************************************************************
 * Name: stm32l4serial_dmasetup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int stm32l4serial_dmasetup(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  int result;
  uint32_t regval;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = stm32l4serial_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

  /* Acquire the DMA channel.  This should always succeed. */

  priv->rxdma = stm32l4_dmachannel(priv->rxdma_channel);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Configure for non-circular DMA reception into the RX FIFO */

      stm32l4_dmasetup(priv->rxdma,
                     priv->usartbase + STM32L4_USART_RDR_OFFSET,
                     (uint32_t)priv->rxfifo,
                     RXDMA_BUFFER_SIZE,
                     SERIAL_DMA_IFLOW_CONTROL_WORD);
    }
  else
#endif
    {
      /* Configure for circular DMA reception into the RX FIFO */

      stm32l4_dmasetup(priv->rxdma,
                     priv->usartbase + STM32L4_USART_RDR_OFFSET,
                     (uint32_t)priv->rxfifo,
                     RXDMA_BUFFER_SIZE,
                     SERIAL_DMA_CONTROL_WORD);
    }

  /* Reset our DMA shadow pointer to match the address just
   * programmed above.
   */

  priv->rxdmanext = 0;

  /* Enable receive DMA for the UART */

  regval  = stm32l4serial_getreg(priv, STM32L4_USART_CR3_OFFSET);
  regval |= USART_CR3_DMAR;
  stm32l4serial_putreg(priv, STM32L4_USART_CR3_OFFSET, regval);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Start the DMA channel, and arrange for callbacks at the full point
       * in the FIFO. After buffer gets full, hardware flow-control kicks
       * in and DMA transfer is stopped.
       */

      stm32l4_dmastart(priv->rxdma, stm32l4serial_dmarxcallback,
                       (void *)priv, false);
    }
  else
#endif
    {
      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      stm32l4_dmastart(priv->rxdma, stm32l4serial_dmarxcallback,
                       (void *)priv, true);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32l4serial_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void stm32l4serial_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  uint32_t regval;

  /* Mark device as uninitialized. */

  priv->initialized = false;

  /* Disable all interrupts */

  stm32l4serial_disableusartint(priv, NULL);

  /* Disable USART APB1/2 clock */

  stm32l4serial_setapbclock(dev, false);

  /* Disable Rx, Tx, and the UART */

  regval  = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
  stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, regval);

  /* Release pins. "If the serial-attached device is powered down, the TX
   * pin causes back-powering, potentially confusing the device to the point
   * of complete lock-up."
   *
   * REVISIT:  Is unconfiguring the pins appropriate for all device?  If not,
   * then this may need to be a configuration option.
   */

  stm32l4_unconfiggpio(priv->tx_gpio);
  stm32l4_unconfiggpio(priv->rx_gpio);

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio != 0)
    {
      stm32l4_unconfiggpio(priv->cts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio != 0)
    {
      stm32l4_unconfiggpio(priv->rts_gpio);
    }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      stm32l4_unconfiggpio(priv->rs485_dir_gpio);
    }
#endif
}

/****************************************************************************
 * Name: stm32l4serial_dmashutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void stm32l4serial_dmashutdown(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;

  /* Perform the normal UART shutdown */

  stm32l4serial_shutdown(dev);

  /* Stop the DMA channel */

  stm32l4_dmastop(priv->rxdma);

  /* Release the DMA channel */

  stm32l4_dmafree(priv->rxdma);
  priv->rxdma = NULL;
}
#endif

/****************************************************************************
 * Name: stm32l4serial_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling). The RX
 *   and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int stm32l4serial_attach(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, priv);
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
 * Name: stm32l4serial_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void stm32l4serial_detach(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
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

static int up_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stm32l4_serial_s *priv = (FAR struct stm32l4_serial_s *)arg;
  int  passes;
  bool handled;

  DEBUGASSERT(priv != NULL);

  /* Report serial activity to the power management logic */

#if defined(CONFIG_PM) && CONFIG_STM32L4_PM_SERIAL_ACTIVITY > 0
  pm_activity(PM_IDLE_DOMAIN, CONFIG_STM32L4_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the masked USART status word. */

      priv->sr = stm32l4serial_getreg(priv, STM32L4_USART_ISR_OFFSET);

      /* USART interrupts:
       *
       * Enable             Status          Meaning             Usage
       * ------------------ --------------- ------------------- ----------
       * USART_CR1_IDLEIE   USART_ISR_IDLE   Idle Line
       *                                     Detected           (not used)
       * USART_CR1_RXNEIE   USART_ISR_RXNE   Received Data
       *                                     Ready to be Read
       * "              "   USART_ISR_ORE    Overrun Error
       *                                     Detected
       * USART_CR1_TCIE     USART_ISR_TC     Transmission
       *                                     Complete           (used only
       *                                                         for RS-485)
       * USART_CR1_TXEIE    USART_ISR_TXE    Transmit Data
       *                                     Register Empty
       * USART_CR1_PEIE     USART_ISR_PE     Parity Error
       *
       * USART_CR2_LBDIE    USART_ISR_LBD    Break Flag         (not used)
       * USART_CR3_EIE      USART_ISR_FE     Framing Error
       * "           "      USART_ISR_NE     Noise Error
       * "           "      USART_ISR_ORE    Overrun Error
       *                                     Detected
       * USART_CR3_CTSIE    USART_ISR_CTS    CTS flag           (not used)
       *
       * NOTE: Some of these status bits must be cleared by explicitly
       * writing one to the ICR register: USART_ICR_CTSCF, USART_ICR_LBDCF.
       * Note of those are currently being used.
       */

#ifdef HAVE_RS485
      /* Transmission of whole buffer is over - TC is set, TXEIE is cleared.
       * Note - this should be first, to have the most recent TC bit value
       * from SR register - sending data affects TC, but without refresh we
       * will not know that...
       */

      if ((priv->sr & USART_ISR_TC) != 0 &&
          (priv->ie & USART_CR1_TCIE) != 0 &&
          (priv->ie & USART_CR1_TXEIE) == 0)
        {
          stm32l4_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
          stm32l4serial_restoreusartint(priv, priv->ie & ~USART_CR1_TCIE);
        }
#endif

      /* Handle incoming, receive bytes. */

      if ((priv->sr & USART_ISR_RXNE) != 0 &&
          (priv->ie & USART_CR1_RXNEIE) != 0)
        {
          /* Received data ready... process incoming bytes.  NOTE the check
           * for RXNEIE:  We cannot call uart_recvchards of RX interrupts
           * are disabled.
           */

          uart_recvchars(&priv->dev);
          handled = true;
        }

      /* We may still have to read from the DR register to clear any pending
       * error conditions.
       */

      else if ((priv->sr &
                (USART_ISR_ORE | USART_ISR_NF | USART_ISR_FE)) != 0)
        {
          /* These errors are cleared by writing the corresponding bit to the
           * interrupt clear register (ICR).
           */

          stm32l4serial_putreg(priv, STM32L4_USART_ICR_OFFSET,
                               (USART_ICR_NCF | USART_ICR_ORECF |
                                USART_ICR_FECF));
        }

      /* Handle outgoing, transmit bytes */

      if ((priv->sr & USART_ISR_TXE) != 0 &&
          (priv->ie & USART_CR1_TXEIE) != 0)
        {
          /* Transmit data register empty ... process outgoing bytes */

          uart_xmitchars(&priv->dev);
          handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32l4serial_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int stm32l4serial_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  FAR struct inode      *inode = filep->f_inode;
  FAR struct uart_dev_s *dev   = inode->i_private;
#endif
#if defined(CONFIG_SERIAL_TERMIOS)
  FAR struct stm32l4_serial_s   *priv  =
      (FAR struct stm32l4_serial_s *)dev->priv;
#endif
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        FAR struct stm32l4_serial_s *user =
            (FAR struct stm32l4_serial_s *)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct stm32l4_serial_s));
          }
      }
      break;
#endif

#ifdef CONFIG_STM32L4_USART_SINGLEWIRE
    case TIOCSSINGLEWIRE:
      {
        uint32_t cr1;
        uint32_t cr1_ue;
        irqstate_t flags;

        flags = enter_critical_section();

        /* Get the original state of UE */

        cr1    = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
        cr1_ue = cr1 & USART_CR1_UE;
        cr1   &= ~USART_CR1_UE;

        /* Disable UE, HDSEL can only be written when UE=0 */

        stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, cr1);

        /* Change the TX port to be open-drain/push-pull and enable/disable
         * half-duplex mode.
         */

        uint32_t cr = stm32l4serial_getreg(priv, STM32L4_USART_CR3_OFFSET);

        if ((arg & SER_SINGLEWIRE_ENABLED) != 0)
          {
            uint32_t gpio_val = (arg & SER_SINGLEWIRE_PUSHPULL) ==
                                 SER_SINGLEWIRE_PUSHPULL ?
                                 GPIO_PUSHPULL : GPIO_OPENDRAIN;
            gpio_val |=
              (arg & SER_SINGLEWIRE_PULL_MASK) == SER_SINGLEWIRE_PULLUP ?
                                                  GPIO_PULLUP : GPIO_FLOAT;

            gpio_val |=
              (arg & SER_SINGLEWIRE_PULL_MASK) == SER_SINGLEWIRE_PULLDOWN ?
                                                  GPIO_PULLDOWN : GPIO_FLOAT;

            stm32l4_configgpio((priv->tx_gpio & ~(GPIO_PUPD_MASK |
                                                  GPIO_OPENDRAIN)) |
                                                  gpio_val);

            cr |= USART_CR3_HDSEL;
          }
        else
          {
            stm32l4_configgpio((priv->tx_gpio & ~(GPIO_PUPD_MASK |
                                                  GPIO_OPENDRAIN)) |
                                                  GPIO_PUSHPULL);

            cr &= ~USART_CR3_HDSEL;
          }

        stm32l4serial_putreg(priv, STM32L4_USART_CR3_OFFSET, cr);

        /* Re-enable UE if appropriate */

        stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, cr1 | cr1_ue);
        leave_critical_section(flags);
      }
     break;
#endif

#ifdef CONFIG_STM32L4_USART_INVERT
    case TIOCSINVERT:
      {
        uint32_t cr1;
        uint32_t cr1_ue;
        irqstate_t flags;

        flags = enter_critical_section();

        /* Get the original state of UE */

        cr1    = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
        cr1_ue = cr1 & USART_CR1_UE;
        cr1   &= ~USART_CR1_UE;

        /* Disable UE, {R,T}XINV can only be written when UE=0 */

        stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, cr1);

        /* Enable/disable signal inversion. */

        uint32_t cr = stm32l4serial_getreg(priv, STM32L4_USART_CR2_OFFSET);

        if (arg & SER_INVERT_ENABLED_RX)
          {
            cr |= USART_CR2_RXINV;
          }
        else
          {
            cr &= ~USART_CR2_RXINV;
          }

        if (arg & SER_INVERT_ENABLED_TX)
          {
            cr |= USART_CR2_TXINV;
          }
        else
          {
            cr &= ~USART_CR2_TXINV;
          }

        stm32l4serial_putreg(priv, STM32L4_USART_CR2_OFFSET, cr);

        /* Re-enable UE if appropriate */

        stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, cr1 | cr1_ue);
        leave_critical_section(flags);
      }
     break;
#endif

#ifdef CONFIG_STM32L4_USART_SWAP
    case TIOCSSWAP:
      {
        uint32_t cr1;
        uint32_t cr1_ue;
        irqstate_t flags;

        flags = enter_critical_section();

        /* Get the original state of UE */

        cr1    = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
        cr1_ue = cr1 & USART_CR1_UE;
        cr1   &= ~USART_CR1_UE;

        /* Disable UE, SWAP can only be written when UE=0 */

        stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, cr1);

        /* Enable/disable Swap mode. */

        uint32_t cr = stm32l4serial_getreg(priv, STM32L4_USART_CR2_OFFSET);

        if (arg == SER_SWAP_ENABLED)
          {
            cr |= USART_CR2_SWAP;
          }
        else
          {
            cr &= ~USART_CR2_SWAP;
          }

        stm32l4serial_putreg(priv, STM32L4_USART_CR2_OFFSET, cr);

        /* Re-enable UE if appropriate */

        stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET, cr1 | cr1_ue);
        leave_critical_section(flags);
      }
     break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        FAR struct termios *termiosp = (FAR struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Note that since we only support 8/9 bit modes and
         * there is no way to report 9-bit mode, we always claim 8.
         */

        termiosp->c_cflag =
          ((priv->parity != 0) ? PARENB : 0) |
          ((priv->parity == 1) ? PARODD : 0) |
          ((priv->stopbits2) ? CSTOPB : 0) |
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          ((priv->oflow) ? CCTS_OFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          ((priv->iflow) ? CRTS_IFLOW : 0) |
#endif
          CS8;

        cfsetispeed(termiosp, priv->baud);

        /* TODO: CCTS_IFLOW, CCTS_OFLOW */
      }
      break;

    case TCSETS:
      {
        FAR struct termios *termiosp = (FAR struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Perform some sanity checks before accepting any changes */

        if (((termiosp->c_cflag & CSIZE) != CS8)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#endif
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

        priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
        priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

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

        stm32l4serial_setformat(dev);
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_STM32L4_USART_BREAKS
#  ifdef CONFIG_STM32L4_SERIALBRK_BSDCOMPAT
    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags;
        uint32_t tx_break;

        flags = enter_critical_section();

        /* Disable any further tx activity */

        priv->ie |= USART_CR1_IE_BREAK_INPROGRESS;

        stm32l4serial_txint(dev, false);

        /* Configure TX as a GPIO output pin and Send a break signal */

        tx_break = GPIO_OUTPUT |
                   (~(GPIO_MODE_MASK | GPIO_OUTPUT_SET) & priv->tx_gpio);
        stm32l4_configgpio(tx_break);

        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;

        flags = enter_critical_section();

        /* Configure TX back to U(S)ART */

        stm32l4_configgpio(priv->tx_gpio);

        priv->ie &= ~USART_CR1_IE_BREAK_INPROGRESS;

        /* Enable further tx activity */

        stm32l4serial_txint(dev, true);

        leave_critical_section(flags);
      }
      break;
#  else
    case TIOCSBRK:  /* No BSD compatibility: Turn break on for M bit times */
      {
        uint32_t cr1;
        irqstate_t flags;

        flags = enter_critical_section();
        cr1   = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
        stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET,
                             cr1 | USART_CR1_SBK);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* No BSD compatibility: May turn off break too soon */
      {
        uint32_t cr1;
        irqstate_t flags;

        flags = enter_critical_section();
        cr1   = stm32l4serial_getreg(priv, STM32L4_USART_CR1_OFFSET);
        stm32l4serial_putreg(priv, STM32L4_USART_CR1_OFFSET,
                             cr1 & ~USART_CR1_SBK);
        leave_critical_section(flags);
      }
      break;
#  endif
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: stm32l4serial_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static int stm32l4serial_receive(FAR struct uart_dev_s *dev,
                                 FAR unsigned int *status)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  uint32_t rdr;

  /* Get the Rx byte */

  rdr      = stm32l4serial_getreg(priv, STM32L4_USART_RDR_OFFSET);

  /* Get the Rx byte plux error information.  Return those in status */

  *status  = priv->sr << 16 | rdr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return rdr & 0xff;
}
#endif

/****************************************************************************
 * Name: stm32l4serial_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static void stm32l4serial_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  irqstate_t flags;
  uint16_t ie;

  /* USART receive interrupts:
   *
   * Enable             Status          Meaning                Usage
   * ------------------ --------------- ------------------     ----------
   * USART_CR1_IDLEIE   USART_ISR_IDLE   Idle Line
   *                                     Detected              (not used)
   * USART_CR1_RXNEIE   USART_ISR_RXNE   Received Data
   *                                     Ready to be Read
   * "              "   USART_ISR_ORE    Overrun Error
   *                                     Detected
   * USART_CR1_PEIE     USART_ISR_PE     Parity Error
   *
   * USART_CR2_LBDIE    USART_ISR_LBD    Break Flag            (not used)
   * USART_CR3_EIE      USART_ISR_FE     Framing Error
   * "           "      USART_ISR_NF     Noise Flag
   * "           "      USART_ISR_ORE    Overrun Error
   *                                     Detected
   */

  flags = enter_critical_section();
  ie = priv->ie;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
#ifdef CONFIG_USART_ERRINTS
      ie |= (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR3_EIE);
#else
      ie |= USART_CR1_RXNEIE;
#endif
#endif
    }
  else
    {
      ie &= ~(USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR3_EIE);
    }

  /* Then set the new interrupt state */

  stm32l4serial_restoreusartint(priv, ie);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: stm32l4serial_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static bool stm32l4serial_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;

  return ((stm32l4serial_getreg(priv, STM32L4_USART_ISR_OFFSET) &
           USART_ISR_RXNE) != 0);
}
#endif

/****************************************************************************
 * Name: stm32l4serial_rxflowcontrol
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
static bool stm32l4serial_rxflowcontrol(FAR struct uart_dev_s *dev,
                                        unsigned int nbuffered, bool upper)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;

#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS) && \
    defined(CONFIG_STM32L4_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {
      /* Assert/de-assert nRTS set it high resume/stop sending */

      stm32l4_gpiowrite(priv->rts_gpio, upper);

      if (upper)
        {
          /* With heavy Rx traffic, RXNE might be set and data pending.
           * Returning 'true' in such case would cause RXNE left unhandled
           * and causing interrupt storm. Sending end might be also be slow
           * to react on nRTS, and returning 'true' here would prevent
           * processing that data.
           *
           * Therefore, return 'false' so input data is still being processed
           * until sending end reacts on nRTS signal and stops sending more.
           */

          return false;
        }

      return upper;
    }

#else
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

          uart_disablerxint(dev);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input is
           * received.
           */

          uart_enablerxint(dev);
        }
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: stm32l4serial_dmareceive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int stm32l4serial_dmareceive(FAR struct uart_dev_s *dev,
                                    FAR unsigned int *status)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  int c = 0;

  if (stm32l4serial_dmanextrx(priv) != priv->rxdmanext)
    {
      c = priv->rxfifo[priv->rxdmanext];

      priv->rxdmanext++;
      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          if (priv->iflow)
            {
              /* RX DMA buffer full. RX paused, RTS line pulled up to prevent
               * more input data from other end.
               */
            }
          else
#endif
            {
              priv->rxdmanext = 0;
            }
        }
    }

  return c;
}
#endif

/****************************************************************************
 * Name: stm32l4serial_dmareenable
 *
 * Description:
 *   Call to re-enable RX DMA.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA)
static void stm32l4serial_dmareenable(FAR struct stm32l4_serial_s *priv)
{
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Configure for non-circular DMA reception into the RX FIFO */

      stm32l4_dmasetup(priv->rxdma,
                       priv->usartbase + STM32L4_USART_RDR_OFFSET,
                       (uint32_t)priv->rxfifo,
                       RXDMA_BUFFER_SIZE,
                       SERIAL_DMA_IFLOW_CONTROL_WORD);
    }
  else
#endif
    {
      /* Configure for circular DMA reception into the RX FIFO */

      stm32l4_dmasetup(priv->rxdma,
                       priv->usartbase + STM32L4_USART_RDR_OFFSET,
                       (uint32_t)priv->rxfifo,
                       RXDMA_BUFFER_SIZE,
                       SERIAL_DMA_CONTROL_WORD);
    }

  /* Reset our DMA shadow pointer to match the address just
   * programmed above.
   */

  priv->rxdmanext = 0;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Start the DMA channel, and arrange for callbacks at the full point
       * in the FIFO. After buffer gets full, hardware flow-control kicks
       * in and DMA transfer is stopped.
       */

      stm32l4_dmastart(priv->rxdma, stm32l4serial_dmarxcallback,
                       (void *)priv, false);
    }
  else
#endif
    {
      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      stm32l4_dmastart(priv->rxdma, stm32l4serial_dmarxcallback,
                       (void *)priv, true);
    }

#ifdef CONFIG_PM
  /* Clear DMA suspended flag. */

  priv->rxdmasusp = false;
#endif
}
#endif

/****************************************************************************
 * Name: stm32l4serial_dmaiflowrestart
 *
 * Description:
 *   Call to restart RX DMA for input flow-controlled USART
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) && defined(CONFIG_SERIAL_IFLOWCONTROL)
static bool stm32l4serial_dmaiflowrestart(struct stm32l4_serial_s *priv)
{
  if (!priv->rxenable)
    {
      /* Rx not enabled by upper layer. */

      return false;
    }

  if (priv->rxdmanext != RXDMA_BUFFER_SIZE)
    {
#ifdef CONFIG_PM
      if (priv->rxdmasusp)
        {
          /* Rx DMA in suspended state. */

          if (stm32l4serial_dmarxavailable(&priv->dev))
            {
              /* DMA buffer has unprocessed data, do not re-enable yet. */

              return false;
            }
        }
      else
#endif
        {
          return false;
        }
    }

  /* DMA is stopped or suspended and DMA buffer does not have pending data,
   * re-enabling without data loss is now safe.
   */

  stm32l4serial_dmareenable(priv);

  return true;
}
#endif

/****************************************************************************
 * Name: stm32l4serial_dmarxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void stm32l4serial_dmarxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;

  /* En/disable DMA reception.
   *
   * Note that it is not safe to check for available bytes and immediately
   * pass them to uart_recvchars as that could potentially recurse back
   * to us again.  Instead, bytes must wait until the next up_dma_poll or
   * DMA event.
   */

  priv->rxenable = enable;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Re-enable RX DMA. */

      stm32l4serial_dmaiflowrestart(priv);
    }
#endif
}
#endif

/****************************************************************************
 * Name: stm32l4serial_dmarxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static bool stm32l4serial_dmarxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (stm32l4serial_dmanextrx(priv) != priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: stm32l4serial_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void stm32l4serial_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      stm32l4_gpiowrite(priv->rs485_dir_gpio, priv->rs485_dir_polarity);
    }
#endif

  stm32l4serial_putreg(priv, STM32L4_USART_TDR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: stm32l4serial_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void stm32l4serial_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;

  irqstate_t flags;

  /* USART transmit interrupts:
   *
   * Enable             Status          Meaning            Usage
   * ------------------ --------------- ----------------   ----------
   * USART_CR1_TCIE     USART_ISR_TC     Transmission
   *                                     Complete          (used only
   *                                                       for RS-485)
   * USART_CR1_TXEIE    USART_ISR_TXE    Transmit Data
   *                                     Register Empty
   * USART_CR3_CTSIE    USART_ISR_CTS    CTS flag          (not used)
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uint16_t ie = priv->ie | USART_CR1_TXEIE;

      /* If RS-485 is supported on this U[S]ART, then also enable the
       * transmission complete interrupt.
       */

#  ifdef HAVE_RS485
      if (priv->rs485_dir_gpio != 0)
        {
          ie |= USART_CR1_TCIE;
        }
#  endif

#  ifdef CONFIG_STM32L4_SERIALBRK_BSDCOMPAT
      if (priv->ie & USART_CR1_IE_BREAK_INPROGRESS)
        {
          return;
        }
#  endif

      stm32l4serial_restoreusartint(priv, ie);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      stm32l4serial_restoreusartint(priv, priv->ie & ~USART_CR1_TXEIE);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32l4serial_txready
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool stm32l4serial_txready(FAR struct uart_dev_s *dev)
{
  FAR struct stm32l4_serial_s *priv =
      (FAR struct stm32l4_serial_s *)dev->priv;
  return ((stm32l4serial_getreg(priv, STM32L4_USART_ISR_OFFSET) &
           USART_ISR_TXE) != 0);
}

/****************************************************************************
 * Name: stm32l4serial_dmarxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void stm32l4serial_dmarxcallback(DMA_HANDLE handle, uint8_t status,
                                        FAR void *arg)
{
  FAR struct stm32l4_serial_s *priv = (FAR struct stm32l4_serial_s *)arg;

  if (priv->rxenable && stm32l4serial_dmarxavailable(&priv->dev))
    {
      uart_recvchars(&priv->dev);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          /* Re-enable RX DMA. */

          stm32l4serial_dmaiflowrestart(priv);
        }
#endif
    }

  /* Get the masked USART status word to check and clear error flags.
   *
   * When wake-up from low power mode was not fast enough, UART is resumed
   * too late and sometimes exactly when character was coming over UART,
   * resulting to frame error.

   * If error flag is not cleared, Rx DMA will be stuck. Clearing errors
   * will release Rx DMA.
   */

  priv->sr = stm32l4serial_getreg(priv, STM32L4_USART_ISR_OFFSET);

  if ((priv->sr & (USART_ISR_ORE | USART_ISR_NF | USART_ISR_FE)) != 0)
    {
      stm32l4serial_putreg(priv, STM32L4_USART_ICR_OFFSET,
                           (USART_ICR_NCF | USART_ICR_ORECF |
                            USART_ICR_FECF));
    }
}
#endif

/****************************************************************************
 * Name: stm32l4serial_pmnotify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void stm32l4serial_pmnotify(FAR struct pm_callback_s *cb, int domain,
                                   enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case PM_NORMAL:
        {
          stm32l4serial_pm_setsuspend(false);
        }
        break;

      case PM_IDLE:
        {
          stm32l4serial_pm_setsuspend(false);
        }
        break;

      case PM_STANDBY:
        {
          /* TODO: Alternative configuration and logic for enabling serial in
           *       Stop 1 mode with HSI16 missing. Current logic allows
           *       suspending serial peripherals for Stop 0/1/2 when serial
           *       Rx/Tx buffers are empty (checked in pmprepare).
           */

          stm32l4serial_pm_setsuspend(true);
        }
        break;

      case PM_SLEEP:
        {
          stm32l4serial_pm_setsuspend(true);
        }
        break;

      default:

        /* Should not get here */

        break;
    }
}
#endif

/****************************************************************************
 * Name: stm32l4serial_pmprepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int stm32l4serial_pmprepare(FAR struct pm_callback_s *cb, int domain,
                                   enum pm_state_e pmstate)
{
  int n;

  /* Logic to prepare for a reduced power state goes here. */

  switch (pmstate)
    {
    case PM_NORMAL:
    case PM_IDLE:
      break;

    case PM_STANDBY:
    case PM_SLEEP:

#ifdef SERIAL_HAVE_RXDMA
      /* Flush Rx DMA buffers before checking state of serial device
       * buffers.
       */

      stm32l4_serial_dma_poll();
#endif

      /* Check if any of the active ports have data pending on Tx/Rx
       * buffers.
       */

      for (n = 0; n < STM32L4_NUSART + STM32L4_NUART; n++)
        {
          struct stm32l4_serial_s *priv = g_uart_devs[n];

          if (!priv || !priv->initialized)
            {
              /* Not active, skip. */

              continue;
            }

          if (priv->suspended)
            {
              /* Port already suspended, skip. */

              continue;
            }

          /* Check if port has data pending (Rx & Tx). */

          if (priv->dev.xmit.head != priv->dev.xmit.tail)
            {
              return ERROR;
            }

          if (priv->dev.recv.head != priv->dev.recv.tail)
            {
              return ERROR;
            }
        }
      break;

    default:

      /* Should not get here */

      break;
    }

  return OK;
}
#endif

#endif /* HAVE_UART */
#endif /* USE_SERIALDRIVER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

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
#ifdef HAVE_UART
  unsigned i;

  /* Disable all USART interrupts */

  for (i = 0; i < STM32L4_NUSART + STM32L4_NUART; i++)
    {
      if (g_uart_devs[i])
        {
          stm32l4serial_disableusartint(g_uart_devs[i], NULL);
        }
    }

  /* Configure whichever one is the console */

#if CONSOLE_UART > 0
  stm32l4serial_setup(&g_uart_devs[CONSOLE_UART - 1]->dev);
#endif
#endif /* HAVE UART */
}
#endif /* USE_EARLYSERIALINIT */

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
#ifdef HAVE_UART
  char devname[16];
  unsigned i;
  unsigned minor = 0;
#ifdef CONFIG_PM
  int ret;
#endif

  /* Register to receive power management callbacks */

#ifdef CONFIG_PM
  ret = pm_register(&g_serialpm.pm_cb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

  /* Register the console */

#if CONSOLE_UART > 0
  uart_register("/dev/console", &g_uart_devs[CONSOLE_UART - 1]->dev);

#ifndef CONFIG_STM32L4_SERIAL_DISABLE_REORDERING
  /* If not disabled, register the console UART to ttyS0 and exclude
   * it from initializing it further down
   */

  uart_register("/dev/ttyS0", &g_uart_devs[CONSOLE_UART - 1]->dev);
  minor = 1;
#endif

#ifdef SERIAL_HAVE_CONSOLE_DMA
  /* If we need to re-initialise the console to enable DMA do that here. */

  stm32l4serial_dmasetup(&g_uart_devs[CONSOLE_UART - 1]->dev);
#endif
#endif /* CONSOLE_UART > 0 */

  /* Register all remaining USARTs */

  strcpy(devname, "/dev/ttySx");

  for (i = 0; i < STM32L4_NUSART + STM32L4_NUART; i++)
    {
      /* Don't create a device for non-configured ports. */

      if (g_uart_devs[i] == 0)
        {
          continue;
        }

#ifndef CONFIG_STM32L4_SERIAL_DISABLE_REORDERING
      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->dev.isconsole)
        {
          continue;
        }
#endif

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + minor++;
      uart_register(devname, &g_uart_devs[i]->dev);
    }
#endif /* HAVE UART */
}

/****************************************************************************
 * Name: stm32l4_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void stm32l4_serial_dma_poll(void)
{
    irqstate_t flags;

    flags = enter_critical_section();

#ifdef CONFIG_USART1_RXDMA
  if (g_usart1priv.rxdma != NULL)
    {
      stm32l4serial_dmarxcallback(g_usart1priv.rxdma, 0, &g_usart1priv);
    }
#endif

#ifdef CONFIG_USART2_RXDMA
  if (g_usart2priv.rxdma != NULL)
    {
      stm32l4serial_dmarxcallback(g_usart2priv.rxdma, 0, &g_usart2priv);
    }
#endif

#ifdef CONFIG_USART3_RXDMA
  if (g_usart3priv.rxdma != NULL)
    {
      stm32l4serial_dmarxcallback(g_usart3priv.rxdma, 0, &g_usart3priv);
    }
#endif

#ifdef CONFIG_UART4_RXDMA
  if (g_uart4priv.rxdma != NULL)
    {
      stm32l4serial_dmarxcallback(g_uart4priv.rxdma, 0, &g_uart4priv);
    }
#endif

#ifdef CONFIG_UART5_RXDMA
  if (g_uart5priv.rxdma != NULL)
    {
      stm32l4serial_dmarxcallback(g_uart5priv.rxdma, 0, &g_uart5priv);
    }
#endif

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#if CONSOLE_UART > 0
  struct stm32l4_serial_s *priv = g_uart_devs[CONSOLE_UART - 1];
  uint16_t ie;

  stm32l4serial_disableusartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  stm32l4serial_restoreusartint(priv, ie);
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
#if CONSOLE_UART > 0
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
