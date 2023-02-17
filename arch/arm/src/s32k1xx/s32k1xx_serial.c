/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_serial.c
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
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/power/pm.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/s32k1xx_lpuart.h"
#include "s32k1xx_edma.h"
#include "hardware/s32k1xx_dmamux.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_config.h"
#include "s32k1xx_pin.h"
#include "s32k1xx_lowputc.h"
#include "s32k1xx_serial.h"

#include "s32k1xx_periphclocks.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which LPUART with be tty0/console and which tty1-7?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered LPUART.
 */

/* First pick the console and ttys0.  This could be any of LPUART0-2 */

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart0priv /* LPUART0 is console */
#  define TTYS0_DEV           g_lpuart0priv /* LPUART0 is ttyS0 */
#  define UART1_ASSIGNED      1
#  if defined(CONFIG_LPUART0_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART0_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart1priv /* LPUART1 is console */
#  define TTYS0_DEV           g_lpuart1priv /* LPUART1 is ttyS0 */
#  define UART2_ASSIGNED      1
#  if defined(CONFIG_LPUART1_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART1_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart2priv /* LPUART2 is console */
#  define TTYS0_DEV           g_lpuart2priv /* LPUART2 is ttyS0 */
#  define UART2_ASSIGNED      1
#  if defined(CONFIG_LPUART2_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART2_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#else
#  undef CONSOLE_DEV                      /* No console */
#  if defined(CONFIG_S32K1XX_LPUART0)
#    define TTYS0_DEV         g_lpuart0priv /* LPUART0 is ttyS0 */
#    define UART1_ASSIGNED    1
#  elif defined(CONFIG_S32K1XX_LPUART1)
#    define TTYS0_DEV         g_lpuart1priv /* LPUART1 is ttyS0 */
#    define UART2_ASSIGNED    1
#  elif defined(CONFIG_S32K1XX_LPUART2)
#    define TTYS0_DEV         g_lpuart2priv /* LPUART2 is ttyS0 */
#    define UART3_ASSIGNED    1
#  endif
#endif

/* Pick ttys1.  This could be any of LPUART0-2 excluding the console UART.
 * One of LPUART0-8 could be the console; one of UART0-2 has already been
 * assigned to ttys0.
 */

#if defined(CONFIG_S32K1XX_LPUART0) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_lpuart0priv /* LPUART0 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_S32K1XX_LPUART1) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_lpuart1priv /* LPUART1 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_S32K1XX_LPUART2) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_lpuart2priv /* LPUART2 is ttyS1 */
#  define UART3_ASSIGNED      1
#endif

#if defined(SERIAL_HAVE_CONSOLE_RXDMA) || defined(SERIAL_HAVE_CONSOLE_TXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA
#endif

/* Pick ttys2.  This could be one of LPUART0-2. It can't be LPUART0 because
 * that was either assigned as ttyS0 or ttys1.  One of LPUART0-2 could be the
 * console.  One of UART1-2 has already been assigned to ttys0 or ttyS1.
 */

#if defined(CONFIG_S32K1XX_LPUART1) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_lpuart1priv /* LPUART1 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_S32K1XX_LPUART2) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_lpuart2priv /* LPUART2 is ttyS2 */
#  define UART3_ASSIGNED      1
#endif

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_S32K1XX_PM_SERIAL_ACTIVITY)
#  define CONFIG_S32K1XX_PM_SERIAL_ACTIVITY 10
#endif

#if defined(CONFIG_PM_SERIAL0_STANDBY) || defined(CONFIG_PM_SERIAL0_SLEEP)
#   define CONFIG_PM_SERIAL0
#endif
#if defined(CONFIG_PM_SERIAL1_STANDBY) || defined(CONFIG_PM_SERIAL1_SLEEP)
#   define CONFIG_PM_SERIAL1
#endif
#if defined(CONFIG_PM_SERIAL2_STANDBY) || defined(CONFIG_PM_SERIAL2_SLEEP)
#   define CONFIG_PM_SERIAL2
#endif

#if !defined(CONFIG_S32K1XX_SERIAL_RXDMA_BUFFER_SIZE)
#  define CONFIG_S32K1XX_SERIAL_RXDMA_BUFFER_SIZE 32
#endif

#define RXDMA_BUFFER_SIZE   CONFIG_S32K1XX_SERIAL_RXDMA_BUFFER_SIZE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct s32k1xx_uart_s
{
  struct uart_dev_s dev;    /* Generic UART device */
  uint32_t uartbase;        /* Base address of UART registers */
  uint32_t baud;            /* Configured baud */
  uint32_t ie;              /* Saved enabled interrupts */
  uint8_t  irq;             /* IRQ associated with this UART */
  uint8_t  parity;          /* 0=none, 1=odd, 2=even */
  uint8_t  bits;            /* Number of bits (7 or 8) */
#if defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)
  uint8_t  inviflow:1;      /* Invert RTS sense */
  const uint32_t rts_gpio;  /* U[S]ART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t cts_gpio;  /* U[S]ART CTS GPIO pin configuration */
#endif

  uint8_t  stopbits2:1;     /* 1: Configure with 2 stop bits vs 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t  iflow:1;         /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t  oflow:1;         /* output flow control (CTS) enabled */
#endif
#ifdef CONFIG_SERIAL_RS485CONTROL
  uint8_t rs485mode:1;      /* We are in RS485 (RTS on TX) mode */
#endif
  /* TX DMA state */

#ifdef SERIAL_HAVE_TXDMA
  const unsigned int dma_txreqsrc;  /* DMAMUX source of TX DMA request */
  DMACH_HANDLE       txdma;         /* currently-open trasnmit DMA stream */
  sem_t              txdmasem;      /* Indicate TX DMA completion */
#endif

  /* RX DMA state */

#ifdef SERIAL_HAVE_RXDMA
  const unsigned int dma_rxreqsrc;  /* DMAMUX source of RX DMA request */
  DMACH_HANDLE       rxdma;         /* currently-open receive DMA stream */
  bool               rxenable;      /* DMA-based reception en/disable */
  uint32_t           rxdmanext;     /* Next byte in the DMA buffer to be read */
  char *const        rxfifo;        /* Receive DMA buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t s32k1xx_serialin(struct s32k1xx_uart_s *priv,
                                      uint32_t offset);
static inline void s32k1xx_serialout(struct s32k1xx_uart_s *priv,
                                   uint32_t offset, uint32_t value);
static inline void s32k1xx_disableuartint(struct s32k1xx_uart_s *priv,
                                        uint32_t *ie);
static inline void s32k1xx_restoreuartint(struct s32k1xx_uart_s *priv,
                                        uint32_t ie);

static int  s32k1xx_setup(struct uart_dev_s *dev);
static void s32k1xx_shutdown(struct uart_dev_s *dev);
static int  s32k1xx_attach(struct uart_dev_s *dev);
static void s32k1xx_detach(struct uart_dev_s *dev);
static int  s32k1xx_interrupt(int irq, void *context, void *arg);
static int  s32k1xx_ioctl(struct file *filep, int cmd, unsigned long arg);
#if !defined(SERIAL_HAVE_ONLY_RXDMA)
static int  s32k1xx_receive(struct uart_dev_s *dev, unsigned int *status);
static void s32k1xx_rxint(struct uart_dev_s *dev, bool enable);
static bool s32k1xx_rxavailable(struct uart_dev_s *dev);
#endif
#if !defined(SERIAL_HAVE_ONLY_TXDMA)
static void s32k1xx_txint(struct uart_dev_s *dev, bool enable);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool s32k1xx_rxflowcontrol(struct uart_dev_s *dev,
                                  unsigned int nbuffered, bool upper);
#endif
static void s32k1xx_send(struct uart_dev_s *dev, int ch);
static bool s32k1xx_txready(struct uart_dev_s *dev);
#ifdef SERIAL_HAVE_TXDMA
static void s32k1xx_dma_send(struct uart_dev_s *dev);
static void s32k1xx_dma_txint(struct uart_dev_s *dev, bool enable);
static void s32k1xx_dma_txavailable(struct uart_dev_s *dev);
static void s32k1xx_dma_txcallback(DMACH_HANDLE handle, void *arg, bool done,
                                   int result);
#endif

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int  s32k1xx_dma_setup(struct uart_dev_s *dev);
static void s32k1xx_dma_shutdown(struct uart_dev_s *dev);
#endif

#ifdef SERIAL_HAVE_RXDMA
static int  s32k1xx_dma_receive(struct uart_dev_s *dev,
                                unsigned int *status);
#ifdef CONFIG_PM
static void s32k1xx_dma_reenable(struct s32k1xx_uart_s *priv);
#endif
static void s32k1xx_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool s32k1xx_dma_rxavailable(struct uart_dev_s *dev);

static void s32k1xx_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                                   int result);
#endif

static bool s32k1xx_txempty(struct uart_dev_s *dev);

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int dowmin,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

#if !defined(SERIAL_HAVE_ONLY_TXDMA) && !defined(SERIAL_HAVE_ONLY_RXDMA)
static const struct uart_ops_s g_lpuart_ops =
{
  .setup          = s32k1xx_setup,
  .shutdown       = s32k1xx_shutdown,
  .attach         = s32k1xx_attach,
  .detach         = s32k1xx_detach,
  .ioctl          = s32k1xx_ioctl,
  .receive        = s32k1xx_receive,
  .rxint          = s32k1xx_rxint,
  .rxavailable    = s32k1xx_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = s32k1xx_rxflowcontrol,
#endif
  .send           = s32k1xx_send,
  .txint          = s32k1xx_txint,
  .txready        = s32k1xx_txready,
  .txempty        = s32k1xx_txempty,
};
#endif

#if defined(SERIAL_HAVE_RXDMA) && defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_lpuart_rxtxdma_ops =
{
  .setup          = s32k1xx_dma_setup,
  .shutdown       = s32k1xx_dma_shutdown,
  .attach         = s32k1xx_attach,
  .detach         = s32k1xx_detach,
  .ioctl          = s32k1xx_ioctl,
  .receive        = s32k1xx_dma_receive,
  .rxint          = s32k1xx_dma_rxint,
  .rxavailable    = s32k1xx_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = s32k1xx_rxflowcontrol,
#endif
  .send           = s32k1xx_send,
  .txint          = s32k1xx_dma_txint,
  .txready        = s32k1xx_txready,
  .txempty        = s32k1xx_txempty,
  .dmatxavail     = s32k1xx_dma_txavailable,
  .dmasend        = s32k1xx_dma_send,
};
#endif
#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_RXDMA)
static const struct uart_ops_s g_lpuart_rxdma_ops =
{
  .setup          = s32k1xx_dma_setup,
  .shutdown       = s32k1xx_dma_shutdown,
  .attach         = s32k1xx_attach,
  .detach         = s32k1xx_detach,
  .ioctl          = s32k1xx_ioctl,
  .receive        = s32k1xx_dma_receive,
  .rxint          = s32k1xx_dma_rxint,
  .rxavailable    = s32k1xx_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = s32k1xx_rxflowcontrol,
#endif
  .send           = s32k1xx_send,
  .txint          = s32k1xx_txint,
  .txready        = s32k1xx_txready,
  .txempty        = s32k1xx_txempty,
};
#endif

#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_lpuart_txdma_ops =
{
    .setup          = s32k1xx_dma_setup,
    .shutdown       = s32k1xx_dma_shutdown,
    .attach         = s32k1xx_attach,
    .detach         = s32k1xx_detach,
    .ioctl          = s32k1xx_ioctl,
    .receive        = s32k1xx_receive,
    .rxint          = s32k1xx_rxint,
    .rxavailable    = s32k1xx_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = s32k1xx_rxflowcontrol,
#endif
    .send           = s32k1xx_send,
    .txint          = s32k1xx_dma_txint,
    .txready        = s32k1xx_txready,
    .txempty        = s32k1xx_txempty,
    .dmatxavail     = s32k1xx_dma_txavailable,
    .dmasend        = s32k1xx_dma_send,
};
#endif

/* Avoid unused warning */
#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_RXDMA)
const struct uart_ops_s *g_o0 = &g_lpuart_rxdma_ops;
#endif
#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_TXDMA)
const struct uart_ops_s *g_o1 = &g_lpuart_txdma_ops;
#endif

/* I/O buffers */

#ifdef CONFIG_LPUART0_RXDMA
static char g_lpuart0rxfifo[RXDMA_BUFFER_SIZE];
#endif

# ifdef CONFIG_LPUART1_RXDMA
static char g_lpuart1rxfifo[RXDMA_BUFFER_SIZE];
#endif

#ifdef CONFIG_LPUART2_RXDMA
static char g_lpuart2rxfifo[RXDMA_BUFFER_SIZE];
#endif

/* I/O buffers */

#ifdef CONFIG_S32K1XX_LPUART0
static char g_lpuart0rxbuffer[CONFIG_LPUART0_RXBUFSIZE];
static char g_lpuart0txbuffer[CONFIG_LPUART0_TXBUFSIZE];
#endif

#ifdef CONFIG_S32K1XX_LPUART1
static char g_lpuart1rxbuffer[CONFIG_LPUART1_RXBUFSIZE];
static char g_lpuart1txbuffer[CONFIG_LPUART1_TXBUFSIZE];
#endif

#ifdef CONFIG_S32K1XX_LPUART2
static char g_lpuart2rxbuffer[CONFIG_LPUART2_RXBUFSIZE];
static char g_lpuart2txbuffer[CONFIG_LPUART2_TXBUFSIZE];
#endif

/* This describes the state of the S32K1XX lpuart0 port. */

#ifdef CONFIG_S32K1XX_LPUART0
static struct s32k1xx_uart_s g_lpuart0priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART0_RXBUFSIZE,
        .buffer     = g_lpuart0rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART0_TXBUFSIZE,
        .buffer     = g_lpuart0txbuffer,
      },
#  if defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART1_RXDMA) && !defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_txdma_ops,
#  else
        .ops       = &g_lpuart_ops,
#  endif
        .priv         = &g_lpuart0priv,
      },
  .uartbase     = S32K1XX_LPUART0_BASE,
  .baud         = CONFIG_LPUART0_BAUD,
  .irq          = S32K1XX_IRQ_LPUART0,
  .parity       = CONFIG_LPUART0_PARITY,
  .bits         = CONFIG_LPUART0_BITS,
  .stopbits2    = CONFIG_LPUART0_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART0_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = PIN_LPUART0_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART0_IFLOWCONTROL)
  .iflow        = 1,
#  endif
#  if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART0_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART0_IFLOWCONTROL)))
  .rts_gpio     = PIN_LPUART0_RTS,
#  endif

#  if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART0_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART0_RS485RTSCONTROL)
  .rs485mode    = 1,
#  endif
#  ifdef CONFIG_LPUART0_TXDMA
  .dma_txreqsrc = S32K1XX_DMACHAN_LPUART0_TX,
#  endif
#  ifdef CONFIG_LPUART0_RXDMA
  .dma_rxreqsrc = S32K1XX_DMACHAN_LPUART0_RX,
  .rxfifo        = g_lpuart0rxfifo,
#  endif
};
#endif

/* This describes the state of the S32K1XX lpuart1 port. */

#ifdef CONFIG_S32K1XX_LPUART1
static struct s32k1xx_uart_s g_lpuart1priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART1_RXBUFSIZE,
        .buffer     = g_lpuart1rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART1_TXBUFSIZE,
        .buffer     = g_lpuart1txbuffer,
      },
#    if defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
          .ops       = &g_lpuart_rxtxdma_ops,
#    elif defined(CONFIG_LPUART1_RXDMA) && !defined(CONFIG_LPUART1_TXDMA)
          .ops       = &g_lpuart_rxdma_ops,
#    elif !defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
          .ops       = &g_lpuart_txdma_ops,
#    else
          .ops       = &g_lpuart_ops,
#    endif
          .priv           = &g_lpuart1priv,
  },

  .uartbase     = S32K1XX_LPUART1_BASE,
  .baud         = CONFIG_LPUART1_BAUD,
  .irq          = S32K1XX_IRQ_LPUART1,
  .parity       = CONFIG_LPUART1_PARITY,
  .bits         = CONFIG_LPUART1_BITS,
  .stopbits2    = CONFIG_LPUART1_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART1_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = PIN_LPUART1_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)
  .iflow        = 1,
#  endif
#  if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)))
  .rts_gpio     = PIN_LPUART1_RTS,
#  endif
#  if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART1_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)
  .rs485mode    = 1,
#  endif
#  ifdef CONFIG_LPUART1_TXDMA
  .dma_txreqsrc = S32K1XX_DMACHAN_LPUART1_TX,
#  endif
#  ifdef CONFIG_LPUART1_RXDMA
  .dma_rxreqsrc = S32K1XX_DMACHAN_LPUART1_RX,
  .rxfifo        = g_lpuart1rxfifo,
#  endif
};
#endif

#ifdef CONFIG_S32K1XX_LPUART2
static struct s32k1xx_uart_s g_lpuart2priv =
{
  .dev =
    {
      .recv         =
      {
        .size       = CONFIG_LPUART2_RXBUFSIZE,
        .buffer     = g_lpuart2rxbuffer,
      },
      .xmit         =
      {
        .size       = CONFIG_LPUART2_TXBUFSIZE,
        .buffer     = g_lpuart2txbuffer,
      },
#  if defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART1_RXDMA) && !defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
        .ops       = &g_lpuart_txdma_ops,
#  else
        .ops       = &g_lpuart_ops,
#  endif
        .priv           = &g_lpuart2priv,
  },

  .uartbase     = S32K1XX_LPUART2_BASE,
  .baud         = CONFIG_LPUART2_BAUD,
  .irq          = S32K1XX_IRQ_LPUART2,
  .parity       = CONFIG_LPUART2_PARITY,
  .bits         = CONFIG_LPUART2_BITS,
  .stopbits2    = CONFIG_LPUART2_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART2_OFLOWCONTROL)
  .oflow        = 1,
  .cts_gpio     = PIN_LPUART2_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)
  .iflow        = 1,
#  endif
#  if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)) \
   || (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)))
  .rts_gpio     = PIN_LPUART2_RTS,
#  endif
#  if (((defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL))) \
    && defined(CONFIG_LPUART2_INVERTIFLOWCONTROL))
  .inviflow     = 1,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)
  .rs485mode    = 1,
#  endif
#  ifdef CONFIG_LPUART2_TXDMA
  .dma_txreqsrc = S32K1XX_DMACHAN_LPUART2_TX,
#  endif
#  ifdef CONFIG_LPUART2_RXDMA
  .dma_rxreqsrc = S32K1XX_DMACHAN_LPUART2_RX,
  .rxfifo        = g_lpuart2rxfifo,
# endif
};
#endif

#ifdef CONFIG_PM
static  struct pm_callback_s g_serial_pmcb =
{
  .notify       = up_pm_notify,
  .prepare      = up_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_serialin
 ****************************************************************************/

static inline uint32_t s32k1xx_serialin(struct s32k1xx_uart_s *priv,
                                      uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: s32k1xx_serialout
 ****************************************************************************/

static inline void s32k1xx_serialout(struct s32k1xx_uart_s *priv,
                                     uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: s32k1xx_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int s32k1xx_dma_nextrx(struct s32k1xx_uart_s *priv)
{
  int dmaresidual = s32k1xx_dmach_getcount(priv->rxdma);

  return RXDMA_BUFFER_SIZE - dmaresidual;
}
#endif

/****************************************************************************
 * Name: s32k1xx_disableuartint
 ****************************************************************************/

static inline void s32k1xx_disableuartint(struct s32k1xx_uart_s *priv,
                                          uint32_t *ie)
{
  irqstate_t flags;
  uint32_t regval;

  flags  = spin_lock_irqsave(NULL);
  regval = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);

  /* Return the current Rx and Tx interrupt state */

  if (ie != NULL)
    {
      *ie = regval & LPUART_ALL_INTS;
    }

  regval &= ~LPUART_ALL_INTS;
  s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: s32k1xx_restoreuartint
 ****************************************************************************/

static inline void s32k1xx_restoreuartint(struct s32k1xx_uart_s *priv,
                                        uint32_t ie)
{
  irqstate_t flags;
  uint32_t regval;

  /* Enable/disable any interrupts that are currently disabled but should be
   * enabled/disabled.
   */

  flags   = spin_lock_irqsave(NULL);
  regval  = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= ie;
  s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: s32k1xx_dma_setup
 *
 * Description:
 *   Configure the LPUART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int s32k1xx_dma_setup(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev;
#if defined(SERIAL_HAVE_RXDMA)
  struct s32k1xx_edma_xfrconfig_s config;
#endif
  int result;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = s32k1xx_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

#if defined(SERIAL_HAVE_TXDMA)
  /* Acquire the Tx DMA channel.  This should always succeed. */

  if (priv->dma_txreqsrc != 0)
    {
      if (priv->txdma == NULL)
        {
          priv->txdma = s32k1xx_dmach_alloc(priv->dma_txreqsrc |
                                            DMAMUX_CHCFG_ENBL, 0);
          if (priv->txdma == NULL)
            {
              return -EBUSY;
            }

          nxsem_init(&priv->txdmasem, 0, 1);
          nxsem_set_protocol(&priv->txdmasem, SEM_PRIO_NONE);
        }

      /* Enable Tx DMA for the UART */

      modifyreg32(priv->uartbase + S32K1XX_LPUART_BAUD_OFFSET,
                  0, LPUART_BAUD_TDMAE);
    }
#endif

#if defined(SERIAL_HAVE_RXDMA)
  /* Acquire the Rx DMA channel.  This should always succeed. */

  if (priv->dma_rxreqsrc != 0)
    {
      if (priv->rxdma == NULL)
        {
          priv->rxdma = s32k1xx_dmach_alloc(priv->dma_rxreqsrc |
                                            DMAMUX_CHCFG_ENBL, 0);

          if (priv->rxdma == NULL)
            {
              return -EBUSY;
            }
        }
      else
        {
          s32k1xx_dmach_stop(priv->rxdma);
        }

      /* Configure for circular DMA reception into the RX FIFO */

      config.saddr  = priv->uartbase + S32K1XX_LPUART_DATA_OFFSET;
      config.daddr  = (uint32_t)priv->rxfifo;
      config.soff   = 0;
      config.doff   = 1;
      config.iter   = RXDMA_BUFFER_SIZE;
      config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE |
                      EDMA_CONFIG_LOOPDEST |
                      EDMA_CONFIG_INTHALF  |
                      EDMA_CONFIG_INTMAJOR;
      config.ssize  = EDMA_8BIT;
      config.dsize  = EDMA_8BIT;
      config.nbytes = 1;
    #ifdef CONFIG_KINETIS_EDMA_ELINK
      config.linkch = 0;
    #endif

      s32k1xx_dmach_xfrsetup(priv->rxdma , &config);

      /* Reset our DMA shadow pointer and Rx data availability count to
       * match the address just programmed above.
       */

      priv->rxdmanext = 0;

      /* Enable receive Rx DMA for the UART */

      modifyreg32(priv->uartbase + S32K1XX_LPUART_BAUD_OFFSET,
                  0, LPUART_BAUD_RDMAE);

      /* Enable itnerrupt on Idel and errors */

      modifyreg32(priv->uartbase + S32K1XX_LPUART_CTRL_OFFSET, 0,
                  LPUART_CTRL_PEIE |
                  LPUART_CTRL_FEIE |
                  LPUART_CTRL_NEIE |
                  LPUART_CTRL_ILIE);

      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      s32k1xx_dmach_start(priv->rxdma, s32k1xx_dma_rxcallback, (void *)priv);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: s32k1xx_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int s32k1xx_setup(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
#ifndef CONFIG_SUPPRESS_LPUART_CONFIG
  struct uart_config_s config =
    {
      0
    };

  int ret;

  /* Configure the UART */

  config.baud       = priv->baud;       /* Configured baud */
  config.parity     = priv->parity;     /* 0=none, 1=odd, 2=even */
  config.bits       = priv->bits;       /* Number of bits (5-9) */
  config.stopbits2  = priv->stopbits2;  /* true: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  config.usects     = priv->oflow;      /* Flow control on outbound side */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  /* Flow control on inbound side if not GPIO based */

  if ((priv->rts_gpio & _PIN_MODE_MASK) != _PIN_MODE_GPIO)
    {
      config.userts = priv->iflow;
    }

#endif
#ifdef CONFIG_SERIAL_RS485CONTROL
  config.users485   = priv->rs485mode;  /* Switch into RS485 mode */
#endif
#if defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)
  config.invrts     = priv->inviflow;   /* Inversion of outbound flow control */
#endif

  /* configure the LPUART */

  ret = s32k1xx_lpuart_configure(priv->uartbase, &config);

  /* get the current interrupt bits and place them in ie */

  /* (used to use the interrupts)  */

  priv->ie = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET) & \
             LPUART_ALL_INTS;
  return ret;

#else
  priv->ie = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET) & \
             LPUART_ALL_INTS;
  return OK;
#endif
}

/****************************************************************************
 * Name: s32k1xx_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void s32k1xx_shutdown(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;

  /* Disable the UART */

  /* set the reset bit  */

  s32k1xx_serialout(priv, S32K1XX_LPUART_GLOBAL_OFFSET, LPUART_GLOBAL_RST);

  /* clear the reset bit again */

  s32k1xx_serialout(priv, S32K1XX_LPUART_GLOBAL_OFFSET, 0);
}

/****************************************************************************
 * Name: s32k1xx_dma_shutdown
 *
 * Description:
 *   Disable the LPUART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static void s32k1xx_dma_shutdown(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev;

  /* Perform the normal UART shutdown */

  s32k1xx_shutdown(dev);

#if defined(SERIAL_HAVE_RXDMA)
  /* Stop the RX DMA channel */

  if (priv->dma_rxreqsrc != 0)
    {
      s32k1xx_dmach_stop(priv->rxdma);

      /* Release the RX DMA channel */

      s32k1xx_dmach_free(priv->rxdma);
      priv->rxdma = NULL;
    }
#endif

#if defined(SERIAL_HAVE_TXDMA)
  /* Stop the TX DMA channel */

  if (priv->dma_txreqsrc != 0)
    {
      s32k1xx_dmach_stop(priv->txdma);

      /* Release the TX DMA channel */

      s32k1xx_dmach_free(priv->txdma);
      priv->txdma = NULL;
      nxsem_destroy(&priv->txdmasem);
    }
#endif
}
#endif

/****************************************************************************
 * Name: s32k1xx_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate
 *   in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int s32k1xx_attach(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, s32k1xx_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void s32k1xx_detach(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: s32k1xx_interrupt (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int s32k1xx_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct s32k1xx_uart_s *priv;
  uint32_t usr;
  int passes = 0;
  bool handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct s32k1xx_uart_s *)dev->priv;

#if defined(CONFIG_PM) && CONFIG_S32K1XX_PM_SERIAL_ACTIVITY > 0
  /* Report serial activity to the power management logic */

  pm_activity(PM_IDLE_DOMAIN, CONFIG_S32K1XX_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the current UART status and check for loop
       * termination conditions
       */

      usr  = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
      usr &= (LPUART_STAT_RDRF | LPUART_STAT_TDRE | LPUART_STAT_OR |
              LPUART_STAT_FE | LPUART_STAT_NF | LPUART_STAT_PF |
              LPUART_STAT_IDLE);

      /* Clear serial overrun, parity and framing errors */

      if ((usr & LPUART_STAT_OR) != 0)
        {
          s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET,
                            LPUART_STAT_OR);
        }

      if ((usr & LPUART_STAT_NF) != 0)
        {
          s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET,
                            LPUART_STAT_NF);
        }

      if ((usr & LPUART_STAT_PF) != 0)
        {
          s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET,
                            LPUART_STAT_PF);
        }

      if ((usr & LPUART_STAT_FE) != 0)
        {
          s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET,
                            LPUART_STAT_FE);
        }

      if ((usr & (LPUART_STAT_FE | LPUART_STAT_PF | LPUART_STAT_NF)) != 0)
        {
          /* Discard data */

          s32k1xx_serialin(priv, S32K1XX_LPUART_DATA_OFFSET);
        }

#ifdef SERIAL_HAVE_RXDMA
      /* The line going to idle, deliver any fractions of RX data */

      if ((usr & LPUART_STAT_IDLE) != 0)
        {
          s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET,
                            LPUART_STAT_IDLE);
          s32k1xx_dma_rxcallback(priv->rxdma, priv, false, LPUART_STAT_IDLE);
        }
#endif

      /* Handle incoming, receive bytes */

      if ((usr & LPUART_STAT_RDRF) != 0 &&
          (priv->ie & LPUART_CTRL_RIE) != 0)
        {
          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((usr & LPUART_STAT_TDRE) != 0 &&
          (priv->ie & LPUART_CTRL_TIE) != 0)
        {
          uart_xmitchars(dev);
          handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int s32k1xx_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || defined(CONFIG_SERIAL_TERMIOS)
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  irqstate_t flags;
#endif
  int ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct s32k1xx_uart_s *user = (struct s32k1xx_uart_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct s32k1xx_uart_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;

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

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        termiosp->c_cflag |= ((priv->oflow) ? CCTS_OFLOW : 0);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        termiosp->c_cflag |= ((priv->iflow) ? CRTS_IFLOW : 0);
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
        struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
        uint32_t baud;
        uint32_t ie;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;

        if ((!termiosp)
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

        /* Verify that all settings are valid before committing */

        if (ret == OK)
          {
            /* Commit */

            priv->baud      = baud;
            priv->parity    = parity;
            priv->bits      = nbits;
            priv->stopbits2 = stop2;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow     = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow     = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
            /* effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            flags = spin_lock_irqsave(NULL);
            s32k1xx_disableuartint(priv, &ie);
            ret = dev->ops->setup(dev);

            /* Restore the interrupt state */

            s32k1xx_restoreuartint(priv, ie);
            priv->ie = ie;
            spin_unlock_irqrestore(NULL, flags);
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_S32K1XX_LPUART_INVERT
    case TIOCSINVERT:
      {
        uint32_t ctrl;
        uint32_t stat;
        uint32_t regval;
        irqstate_t flags;
        struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;

        flags  = spin_lock_irqsave(NULL);
        ctrl   = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);
        stat   = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
        regval = ctrl;

        /* {R|T}XINV bit field can only be written when the receiver is
        * disabled (RE=0).
        */

        regval &= ~LPUART_CTRL_RE;

        s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);

        /* Enable/disable signal inversion. */

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

        s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET, stat);
        s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, ctrl);

        spin_unlock_irqrestore(NULL, flags);
      }
      break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static int s32k1xx_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  uint32_t rxd;

  rxd     = s32k1xx_serialin(priv, S32K1XX_LPUART_DATA_OFFSET);
  *status = rxd >> LPUART_DATA_STATUS_SHIFT;
  return (rxd & LPUART_DATA_MASK) >> LPUART_DATA_SHIFT;
}
#endif

/****************************************************************************
 * Name: s32k1xx_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static void s32k1xx_rxint(struct uart_dev_s *dev, bool enable)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  irqstate_t flags;
  uint32_t regval;

  /* Enable interrupts for data available at Rx */

  flags = spin_lock_irqsave(NULL);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_RIE | LPUART_CTRL_FEIE | LPUART_CTRL_ORIE;
#endif
    }
  else
    {
      priv->ie &= ~(LPUART_CTRL_RIE | LPUART_CTRL_FEIE | LPUART_CTRL_ORIE);
    }

  regval  = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= priv->ie;
  s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(NULL, flags);
}
#endif

/****************************************************************************
 * Name: s32k1xx_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static bool s32k1xx_rxavailable(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  uint32_t regval;

  /* Return true is data is ready in the Rx FIFO */

  regval = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_RDRF) != 0);
}
#endif

/****************************************************************************
 * Name: s32k1xx_rxflowcontrol
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
static bool s32k1xx_rxflowcontrol(struct uart_dev_s *dev,
                                  unsigned int nbuffered, bool upper)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev;
  bool use_swhs = false;

#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS)
  use_swhs = (priv->rts_gpio & _PIN_MODE_MASK) == _PIN_MODE_GPIO;
#endif

  if (use_swhs && priv->iflow && (priv->rts_gpio != 0))
    {
      /* Assert/de-assert nRTS set it high resume/stop sending */

      s32k1xx_gpiowrite(priv->rts_gpio, upper);

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
  else
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

  return false;
}
#endif

/****************************************************************************
 * Name: s32k1xx_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the LPUART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int s32k1xx_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev;
  uint32_t nextrx = s32k1xx_dma_nextrx(priv);
  int c = 0;

  /* Check if more data is available */

  if (nextrx != priv->rxdmanext)
    {
      /* Now read from the DMA buffer */

      c = priv->rxfifo[priv->rxdmanext];

      priv->rxdmanext++;

      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
          priv->rxdmanext = 0;
        }
    }

  /* NOTE:  If no data is available, then we would return NULL which is,
   * of course, valid binary data.  The protocol is that the upper half
   * driver must call s32k1xx_dma_rxavailable prior to calling this
   * function to assure that this never happens.
   */

  return c;
}
#endif

/****************************************************************************
 * Name: s32k1xx_dma_reenable
 *
 * Description:
 *   Call to re-enable RX DMA.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) && defined(CONFIG_PM)
static void s32k1xx_dma_reenable(struct s32k1xx_uart_s *priv)
{
  struct s32k1xx_edma_xfrconfig_s config;

  /* Stop an reset the RX DMA */

  s32k1xx_dmach_stop(priv->rxdma);

  /* Configure for circular DMA reception into the RX FIFO */

  config.saddr  = priv->uartbase + S32K1XX_LPUART_DATA_OFFSET;
  config.daddr  = (uint32_t) priv->rxfifo;
  config.soff   = 0;
  config.doff   = 1;
  config.iter   = RXDMA_BUFFER_SIZE;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE |
                  EDMA_CONFIG_LOOPDEST |
                  EDMA_CONFIG_INTHALF |
                  EDMA_CONFIG_INTMAJOR;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = 1;
#ifdef CONFIG_KINETIS_EDMA_ELINK
  config.linkch = 0;
#endif

  s32k1xx_dmach_xfrsetup(priv->rxdma, &config);

  /* Reset our DMA shadow pointer and Rx data availability count to match
   * the address just programmed above.
   */

  priv->rxdmanext = 0;

  /* Start the DMA channel, and arrange for callbacks at the half and
   * full points in the FIFO.  This ensures that we have half a FIFO
   * worth of time to claim bytes before they are overwritten.
   */

  s32k1xx_dmach_start(priv->rxdma, s32k1xx_dma_rxcallback, (void *)priv);

  /* Clear DMA suspended flag. */

  priv->rxdmasusp  = false;
}
#endif

/****************************************************************************
 * Name: s32k1xx_dma_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void s32k1xx_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev;

  /* Enable/disable DMA reception.
   *
   * Note that it is not safe to check for available bytes and immediately
   * pass them to uart_recvchars as that could potentially recurse back
   * to us again.  Instead, bytes must wait until the next up_dma_poll or
   * DMA event.
   */

  priv->rxenable = enable;
}
#endif

/****************************************************************************
 * Name: s32k1xx_dma_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static bool s32k1xx_dma_rxavailable(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (s32k1xx_dma_nextrx(priv) != priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: s32k1xx_dma_txcallback
 *
 * Description:
 *   This function clears dma buffer at complete of DMA transfer and wakes up
 *   threads waiting for space in buffer.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void s32k1xx_dma_txcallback(DMACH_HANDLE handle, void *arg, bool done,
                                   int result)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)arg;
  /* Update 'nbytes' indicating number of bytes actually transferred by DMA.
   * This is important to free TX buffer space by 'uart_xmitchars_done'.
   */

  priv->dev.dmatx.nbytes = priv->dev.dmatx.length + priv->dev.dmatx.nlength;

  /* Adjust the pointers */

  uart_xmitchars_done(&priv->dev);

  /* Release waiter */

  nxsem_post(&priv->txdmasem);
}
#endif

/****************************************************************************
 * Name: s32k1xx_dma_txavailable
 *
 * Description:
 *        Informs DMA that Tx data is available and is ready for transfer.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void s32k1xx_dma_txavailable(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev;

  /* Only send when the DMA is idle */

  nxsem_wait(&priv->txdmasem);

  uart_xmitchars_dma(dev);
}
#endif

/****************************************************************************
 * Name: s32k1xx_dma_send
 *
 * Description:
 *   Called (usually) from the interrupt level to start DMA transfer.
 *   (Re-)Configures DMA Stream updating buffer and buffer length.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void s32k1xx_dma_send(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev;
  struct s32k1xx_edma_xfrconfig_s config;

  /* We need to stop DMA before reconfiguration */

  s32k1xx_dmach_stop(priv->txdma);

  /* Reset the number sent */

  dev->dmatx.nbytes = 0;

  /* Make use of setup function to update buffer and its length for next
   * transfer
   */

  config.iter   = dev->dmatx.length;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = sizeof(dev->dmatx.buffer[0]);
  config.saddr  = (uint32_t)dev->dmatx.buffer;
  config.daddr  = priv->uartbase + S32K1XX_LPUART_DATA_OFFSET;
  config.soff   = sizeof(dev->dmatx.buffer[0]);
  config.doff   = 0;
#ifdef CONFIG_S32K1XX_EDMA_ELINK
  config.linkch  = 0;
#endif

  /* Setup first half */

  s32k1xx_dmach_xfrsetup(priv->txdma, &config);

  /* Is this a split transfer? */

  if (dev->dmatx.nbuffer)
    {
      config.iter   = priv->dev.dmatx.nlength;
      config.saddr  = (uint32_t)priv->dev.dmatx.nbuffer;

      s32k1xx_dmach_xfrsetup(priv->txdma, &config);
    }

  /* Start transmission with the callback on DMA completion */

  s32k1xx_dmach_start(priv->txdma, s32k1xx_dma_txcallback, (void *)priv);
}
#endif

/****************************************************************************
 * Name: s32k1xx_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void s32k1xx_send(struct uart_dev_s *dev, int ch)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  s32k1xx_serialout(priv, S32K1XX_LPUART_DATA_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: s32k1xx_dma_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts from the UART.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void s32k1xx_dma_txint(struct uart_dev_s *dev, bool enable)
{
  /* Nothing to do. */

  /* In case of DMA transfer we do not want to make use of UART interrupts.
   * Instead, we use DMA interrupts that are activated once during boot
   * sequence. Furthermore we can use s32k1xx_dma_txcallback() to handle
   * stuff at half DMA transfer or after transfer completion (depending on
   * the configuration).
   */
}
#endif

/****************************************************************************
 * Name: s32k1xx_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#if !defined(SERIAL_HAVE_ONLY_TXDMA)
static void s32k1xx_txint(struct uart_dev_s *dev, bool enable)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  irqstate_t flags;
  uint32_t regval;

  /* Enable interrupt for TX complete */

  flags = spin_lock_irqsave(NULL);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_TIE;
#endif
    }
  else
    {
      priv->ie &= ~LPUART_CTRL_TIE;
    }

  regval  = s32k1xx_serialin(priv, S32K1XX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= priv->ie;
  s32k1xx_serialout(priv, S32K1XX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(NULL, flags);
}
#endif

/****************************************************************************
 * Name: s32k1xx_txready
 *
 * Description:
 *   Return true if the transmit is completed
 *
 ****************************************************************************/

static bool s32k1xx_txready(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  uint32_t regval;

  regval = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_TC) != 0);
}

/****************************************************************************
 * Name: s32k1xx_txempty
 *
 * Description:
 *   Return true if the transmit reg is empty
 *
 ****************************************************************************/

static bool s32k1xx_txempty(struct uart_dev_s *dev)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)dev->priv;
  uint32_t regval;

  regval = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);
  return ((regval & LPUART_STAT_TDRE) != 0);
}

/****************************************************************************
 * Name: s32k1xx_dma_rxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void s32k1xx_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                                   int result)
{
  struct s32k1xx_uart_s *priv = (struct s32k1xx_uart_s *)arg;
  uint32_t sr;

  if (priv->rxenable && s32k1xx_dma_rxavailable(&priv->dev))
    {
      uart_recvchars(&priv->dev);
    }

  /* Get the masked LPUART status word to check and clear error flags.
   *
   * When wake-up from low power mode was not fast enough, UART is resumed
   * too late and sometimes exactly when character was coming over UART,
   * resulting to frame error.
   * If error flag is not cleared, Rx DMA will be stuck. Clearing errors
   * will release Rx DMA.
   */

  sr = s32k1xx_serialin(priv, S32K1XX_LPUART_STAT_OFFSET);

  if ((sr & (LPUART_STAT_OR | LPUART_STAT_NF | LPUART_STAT_FE)) != 0)
    {
      s32k1xx_serialout(priv, S32K1XX_LPUART_STAT_OFFSET,
                        sr & (LPUART_STAT_OR |
                              LPUART_STAT_NF |
                              LPUART_STAT_FE));
    }
}
#endif

/****************************************************************************
 * Name: up_pm_notify
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
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  unsigned int count = 0;   /* the amount of peripheral clocks to change */

  peripheral_clock_source_t clock_source;

  #ifdef CONFIG_PM_SERIAL0
    struct s32k1xx_uart_s *priv0 = g_lpuart0priv.dev.priv;
  #endif
  #ifdef CONFIG_PM_SERIAL1
    struct s32k1xx_uart_s *priv1 = g_lpuart1priv.dev.priv;
  #endif
  #ifdef CONFIG_PM_SERIAL2
    struct s32k1xx_uart_s *priv2 = g_lpuart2priv.dev.priv;
  #endif

  uint32_t ret_reg = 0;

  /* check if the transition is from the IDLE domain to the NORMAL domain */

  /* or the mode is already done */

  if (((pm_querystate(PM_IDLE_DOMAIN) == PM_IDLE) &&
    (pmstate == PM_NORMAL)) ||
    (((pm_querystate(PM_IDLE_DOMAIN) == pmstate))))
    {
      /* return */

      return;
    }

  /* check which PM it is  */

  switch (pmstate)
  {
    /* in case it needs to change to the RUN mode */

    case PM_NORMAL:
    {
      /* Logic for PM_NORMAL goes here */

      /* set the right clock source to go back to RUN mode */

      clock_source = CLK_SRC_SPLL_DIV2;

      count = 1;
    }
    break;
    default:
    {
      /* don't do anything, just return OK */
    }
    break;
  }

  /* check if something needs to change */

  if (count)
  {
    #ifdef CONFIG_PM_SERIAL0

      /* make the peripheral clock config struct */

      const struct peripheral_clock_config_s clock_config0 =
      {
        .clkname  =   LPUART0_CLK,
        .clkgate  =   true,
        .clksrc   =   clock_source,
        .frac     =   MULTIPLY_BY_ONE,
        .divider  =   1,
      };

      /* read the FIFO register */

      ret_reg = getreg32(priv0->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

      /* make the value */

      ret_reg |= (LPUART_FIFO_RXFLUSH + LPUART_FIFO_TXFLUSH);

      /* write the new value */

      putreg32(ret_reg, priv0->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

      /* shutdown the LPUART1 (soft reset) */

      s32k1xx_shutdown(&g_lpuart0priv);

      /* change the clock config for the new mode */

      s32k1xx_periphclocks(count, &clock_config0);

      /* shutdown the LPUART1 (soft reset) */

      s32k1xx_shutdown(&g_lpuart0priv);

      /* set up the LPUART1 again for the new mode */

      s32k1xx_setup(&g_lpuart0priv);

      /* enable the interrupts */

      s32k1xx_rxint(&g_lpuart0priv, true);
      s32k1xx_txint(&g_lpuart0priv, true);

    #endif
    #ifdef CONFIG_PM_SERIAL1

      /* make the peripheral clock config struct */

      const struct peripheral_clock_config_s clock_config1 =
      {
        .clkname  =   LPUART1_CLK,
        .clkgate  =   true,
        .clksrc   =   clock_source,
        .frac     =   MULTIPLY_BY_ONE,
        .divider  =   1,
      };

      /* read the FIFO register */

      ret_reg = getreg32(priv1->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

      /* make the value */

      ret_reg |= (LPUART_FIFO_RXFLUSH + LPUART_FIFO_TXFLUSH);

      /* write the new value */

      putreg32(ret_reg, priv1->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

      /* shutdown the LPUART1 (soft reset) */

      s32k1xx_shutdown(&g_lpuart1priv);

      /* change the clock config for the new mode */

      s32k1xx_periphclocks(count, &clock_config1);

      /* shutdown the LPUART1 (soft reset) */

      s32k1xx_shutdown(&g_lpuart1priv);

      /* set up the LPUART1 again for the new mode */

      s32k1xx_setup(&g_lpuart1priv);

      /* enable the interrupts */

      s32k1xx_rxint(&g_lpuart1priv, true);
      s32k1xx_txint(&g_lpuart1priv, true);

    #endif
    #ifdef CONFIG_PM_SERIAL2

      /* make the peripheral clock config struct */

      const struct peripheral_clock_config_s clock_config2 =
      {
        .clkname  =   LPUART2_CLK,
        .clkgate  =   true,
        .clksrc   =   clock_source,
        .frac     =   MULTIPLY_BY_ONE,
        .divider  =   1,
      };

      /* read the FIFO register */

      ret_reg = getreg32(priv2->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

      /* make the value */

      ret_reg |= (LPUART_FIFO_RXFLUSH + LPUART_FIFO_TXFLUSH);

      /* write the new value */

      putreg32(ret_reg, priv2->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

      /* shutdown the LPUART1 (soft reset) */

      s32k1xx_shutdown(&g_lpuart2priv);

      /* change the clock config for the new mode */

      s32k1xx_periphclocks(count, &clock_config2);

      /* shutdown the LPUART1 (soft reset) */

      s32k1xx_shutdown(&g_lpuart2priv);

      /* set up the LPUART1 again for the new mode */

      s32k1xx_setup(&g_lpuart2priv);

      /* enable the interrupts */

      s32k1xx_rxint(&g_lpuart2priv, true);
      s32k1xx_txint(&g_lpuart2priv, true);

    #endif
  }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
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
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  unsigned int count = 1;   /* the amount of peripheral clocks to change */

  peripheral_clock_source_t clock_source;

  #ifdef CONFIG_PM_SERIAL0
    struct s32k1xx_uart_s *priv0 =
        (struct s32k1xx_uart_s *)g_lpuart0priv.dev.priv;
  #endif
  #ifdef CONFIG_PM_SERIAL1
    struct s32k1xx_uart_s *priv1 =
        (struct s32k1xx_uart_s *)g_lpuart1priv.dev.priv;
  #endif
  #ifdef CONFIG_PM_SERIAL2
    struct s32k1xx_uart_s *priv2 =
        (struct s32k1xx_uart_s *)g_lpuart2priv.dev.priv;
  #endif

  uint32_t ret_reg = 0;

  /* check if the transition to the mode is already done */

  if (pm_querystate(PM_IDLE_DOMAIN) == pmstate)
    {
      /* return */

      return OK;
    }

  /* check which PM it is  */

  switch (pmstate)
  {
    /* in case it needs to prepare for VLPR mode */

    case PM_STANDBY:
    {
      /* Logic for PM_STANDBY goes here */

      /* set the right clock source */

      clock_source = CLK_SRC_SIRC_DIV2;
    }
    break;

    /* in case it needs to prepare for sleep mode */

    case PM_SLEEP:
    {
      /* Logic for PM_SLEEP goes here */

      /* set the right clock source */

      clock_source = CLK_SRC_SIRC_DIV2;
    }
    break;
    default:
    {
      /* don't do anything, just return OK */

      return OK;
    }
    break;
  }

  #ifdef CONFIG_PM_SERIAL0

    /* make the peripheral clock config struct */

    const struct peripheral_clock_config_s clock_config0 =
    {
      .clkname  =   LPUART0_CLK,
      .clkgate  =   true,
      .clksrc   =   clock_source,
      .frac     =   MULTIPLY_BY_ONE,
      .divider  =   1,
    };

    /* read the FIFO register */

    ret_reg = getreg32(priv0->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

    /* make the value */

    ret_reg |= (LPUART_FIFO_RXFLUSH + LPUART_FIFO_TXFLUSH);

    /* write the new value */

    putreg32(ret_reg, priv0->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

    /* shutdown the LPUART1 (soft reset) */

    s32k1xx_shutdown(&g_lpuart0priv);

    /* change the clock config for the new mode */

    s32k1xx_periphclocks(count, &clock_config0);

    /* shutdown the LPUART1 (soft reset) */

    s32k1xx_shutdown(&g_lpuart0priv);

    /* set up the LPUART1 again for the new mode */

    s32k1xx_setup(&g_lpuart0priv);

    /* enable the interrupts */

    s32k1xx_rxint(&g_lpuart0priv, true);
    s32k1xx_txint(&g_lpuart0priv, true);

  #endif
  #ifdef CONFIG_PM_SERIAL1

    /* make the peripheral clock config struct */

    const struct peripheral_clock_config_s clock_config1 =
    {
      .clkname  =   LPUART1_CLK,
      .clkgate  =   true,
      .clksrc   =   clock_source,
      .frac     =   MULTIPLY_BY_ONE,
      .divider  =   1,
    };

    /* read the FIFO register */

    ret_reg = getreg32(priv1->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

    /* make the value */

    ret_reg |= (LPUART_FIFO_RXFLUSH + LPUART_FIFO_TXFLUSH);

    /* write the new value */

    putreg32(ret_reg, priv1->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

    /* shutdown the LPUART1 (soft reset) */

    s32k1xx_shutdown(&g_lpuart1priv);

    /* change the clock config for the new mode */

    s32k1xx_periphclocks(count, &clock_config1);

    /* shutdown the LPUART1 (soft reset) */

    s32k1xx_shutdown(&g_lpuart1priv);

    /* set up the LPUART1 again for the new mode */

    s32k1xx_setup(&g_lpuart1priv);

    /* enable the interrupts */

    s32k1xx_rxint(&g_lpuart1priv, true);
    s32k1xx_txint(&g_lpuart1priv, true);

  #endif
  #ifdef CONFIG_PM_SERIAL2

    /* make the peripheral clock config struct */

    const struct peripheral_clock_config_s clock_config2 =
    {
      .clkname  =   LPUART2_CLK,
      .clkgate  =   true,
      .clksrc   =   clock_source,
      .frac     =   MULTIPLY_BY_ONE,
      .divider  =   1,
    };

    /* read the FIFO register */

    ret_reg = getreg32(priv2->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

    /* make the value */

    ret_reg |= (LPUART_FIFO_RXFLUSH + LPUART_FIFO_TXFLUSH);

    /* write the new value */

    putreg32(ret_reg, priv2->uartbase + S32K1XX_LPUART_FIFO_OFFSET);

    /* shutdown the LPUART1 (soft reset) */

    s32k1xx_shutdown(&g_lpuart2priv);

    /* change the clock config for the new mode */

    s32k1xx_periphclocks(count, &clock_config2);

    /* shutdown the LPUART1 (soft reset) */

    s32k1xx_shutdown(&g_lpuart2priv);

    /* set up the LPUART1 again for the new mode */

    s32k1xx_setup(&g_lpuart2priv);

    /* enable the interrupts */

    s32k1xx_rxint(&g_lpuart2priv, true);
    s32k1xx_txint(&g_lpuart2priv, true);

  #endif

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void s32k1xx_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function s32k1xx_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.dev.isconsole = true;
  s32k1xx_setup(&CONSOLE_DEV.dev);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that s32k1xx_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef CONFIG_PM
  #if defined(CONFIG_PM_SERIAL_STANDBY) || defined(CONFIG_PM_SERIAL_SLEEP)

  int ret;

  /* Register to receive power management callbacks */

  ret = pm_register(&g_serial_pmcb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
  #endif
#endif

#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV.dev);
#  if defined(SERIAL_HAVE_CONSOLE_DMA)
  s32k1xx_dma_setup(&CONSOLE_DEV.dev);
#  endif
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV.dev);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV.dev);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV.dev);
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
#ifdef CONSOLE_DEV
  struct s32k1xx_uart_s *priv =
      (struct s32k1xx_uart_s *)CONSOLE_DEV.dev.priv;
  uint32_t ie;

  s32k1xx_disableuartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      s32k1xx_lowputc('\r');
    }

  s32k1xx_lowputc(ch);
  s32k1xx_restoreuartint(priv, ie);
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
#if CONSOLE_LPUART > 0
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
