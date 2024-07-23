/****************************************************************************
 * arch/risc-v/src/hpm6000/hpm_serial.c
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
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/power/pm.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hardware/hpm_uart.h"
#include "hpm_gpio.h"
#include "hpm_config.h"
#include "hpm_lowputc.h"
#include "hpm_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */
#ifdef USE_SERIALDRIVER
/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_UART_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0priv     /* UART0 is console */
#    define TTYS0_DEV       g_uart0priv     /* UART0 is ttyS0 */
#    define SERIAL_CONSOLE  1
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1priv     /* UART1 is console */
#    define TTYS0_DEV       g_uart1priv     /* UART1 is ttyS0 */
#    define SERIAL_CONSOLE  2
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2priv     /* UART2 is console */
#    define TTYS0_DEV       g_uart2priv     /* UART2 is ttyS0 */
#    define SERIAL_CONSOLE  3
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3priv     /* UART3 is console */
#    define TTYS0_DEV       g_uart3priv     /* UART3 is ttyS0 */
#    define SERIAL_CONSOLE  4
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart4priv     /* UART4 is console */
#    define TTYS0_DEV       g_uart4priv     /* UART4 is ttyS0 */
#    define SERIAL_CONSOLE  5
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart5priv     /* UART5 is console */
#    define TTYS0_DEV       g_uart5priv     /* UART5 is ttyS0 */
#    define SERIAL_CONSOLE  6
#  elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart6priv     /* UART6 is console */
#    define TTYS0_DEV       g_uart6priv     /* UART6 is ttyS0 */
#    define SERIAL_CONSOLE  7
#  elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart7priv     /* UART7 is console */
#    define TTYS0_DEV       g_uart7priv     /* UART7 is ttyS0 */
#    define SERIAL_CONSOLE  8
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART0_SERIAL_CONSOLE
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#  undef  CONFIG_UART3_SERIAL_CONSOLE
#  undef  CONFIG_UART4_SERIAL_CONSOLE
#  undef  CONFIG_UART5_SERIAL_CONSOLE
#  undef  CONFIG_UART6_SERIAL_CONSOLE
#  undef  CONFIG_UART7_SERIAL_CONSOLE
#  if defined(CONFIG_HPM_UART0)
#    define TTYS0_DEV       g_uart0priv     /* UART0 is ttyS0 */
#    define SERIAL_CONSOLE  1
#  elif defined(CONFIG_HPM_UART1)
#    define TTYS0_DEV       g_uart1priv     /* UART1 is ttyS0 */
#    define SERIAL_CONSOLE  2
#  elif defined(CONFIG_HPM_UART2)
#    define TTYS0_DEV       g_uart2priv     /* UART2 is ttyS0 */
#    define SERIAL_CONSOLE  3
#  elif defined(CONFIG_HPM_UART3)
#    define TTYS0_DEV       g_uart3priv     /* UART3 is ttyS0 */
#    define SERIAL_CONSOLE  4
#  elif defined(CONFIG_HPM_UART4)
#    define TTYS0_DEV       g_uart4priv     /* UART4 is ttyS0 */
#    define SERIAL_CONSOLE  5
#  elif defined(CONFIG_HPM_UART5)
#    define TTYS0_DEV       g_uart5priv     /* UART5 is ttyS0 */
#    define SERIAL_CONSOLE  6
#  elif defined(CONFIG_HPM_UART6)
#    define TTYS0_DEV       g_uart6priv     /* UART6 is ttyS0 */
#    define SERIAL_CONSOLE  7
#  elif defined(CONFIG_HPM_UART7)
#    define TTYS0_DEV       g_uart7priv     /* UART7 is ttyS0 */
#    define SERIAL_CONSOLE  8
#  else
#    undef  TTYS0_DEV
#  endif
#endif

/* Pick ttys1.  This could be any of UART1-8 excluding the console UART.
 * One of UART1-8 could be the console; one of UART1-8 has already been
 * assigned to ttys0.
 */
#if defined(CONFIG_HPM_UART0) && (SERIAL_CONSOLE != 1)
#  define TTYS1_DEV           g_uart0priv /* UART0 is ttyS1 */
#  define SERIAL_CONSOLE      1
#elif defined(CONFIG_HPM_UART1) && (SERIAL_CONSOLE != 2)
#  define TTYS1_DEV           g_uart1priv /* LPUART1 is ttyS1 */
#  define SERIAL_CONSOLE      2
#elif defined(CONFIG_HPM_UART2) && (SERIAL_CONSOLE != 3)
#  define TTYS1_DEV           g_uart2priv /* LPUART2 is ttyS1 */
#  define SERIAL_CONSOLE      3
#elif defined(CONFIG_HPM_UART3) && (SERIAL_CONSOLE != 4)
#  define TTYS1_DEV           g_uart3priv /* LPUART3 is ttyS1 */
#  define SERIAL_CONSOLE      4
#elif defined(CONFIG_HPM_UART4) && (SERIAL_CONSOLE != 5)
#  define TTYS1_DEV           g_uart4priv /* LPUART4 is ttyS1 */
#  define SERIAL_CONSOLE      5
#elif defined(CONFIG_HPM_UART5) && (SERIAL_CONSOLE != 6)
#  define TTYS1_DEV           g_uart5priv /* LPUART5 is ttyS1 */
#  define SERIAL_CONSOLE      6
#elif defined(CONFIG_HPM_UART6) && (SERIAL_CONSOLE != 7)
#  define TTYS1_DEV           g_uart6priv /* LPUART6 is ttyS1 */
#  define SERIAL_CONSOLE      7
#elif defined(CONFIG_HPM_UART7) && (SERIAL_CONSOLE != 8)
#  define TTYS1_DEV           g_uart7priv /* LPUART7 is ttyS1 */
#  define SERIAL_CONSOLE      8
#endif

#if defined(CONFIG_HPM_UART1) && (SERIAL_CONSOLE != 2)
#  define TTYS2_DEV           g_uart1priv /* LPUART1 is ttyS1 */
#  define SERIAL_CONSOLE      2
#elif defined(CONFIG_HPM_UART2) && (SERIAL_CONSOLE != 3)
#  define TTYS2_DEV           g_uart2priv /* LPUART2 is ttyS1 */
#  define SERIAL_CONSOLE      3
#elif defined(CONFIG_HPM_UART3) && (SERIAL_CONSOLE != 4)
#  define TTYS2_DEV           g_uart3priv /* LPUART3 is ttyS1 */
#  define SERIAL_CONSOLE      4
#elif defined(CONFIG_HPM_UART4) && (SERIAL_CONSOLE != 5)
#  define TTYS2_DEV           g_uart4priv /* LPUART4 is ttyS1 */
#  define SERIAL_CONSOLE      5
#elif defined(CONFIG_HPM_UART5) && (SERIAL_CONSOLE != 6)
#  define TTYS2_DEV           g_uart5priv /* LPUART5 is ttyS1 */
#  define SERIAL_CONSOLE      6
#elif defined(CONFIG_HPM_UART6) && (SERIAL_CONSOLE != 7)
#  define TTYS2_DEV           g_uart6priv /* LPUART6 is ttyS1 */
#  define SERIAL_CONSOLE      7
#elif defined(CONFIG_HPM_UART7) && (SERIAL_CONSOLE != 8)
#  define TTYS2_DEV           g_uart7priv /* LPUART7 is ttyS1 */
#  define SERIAL_CONSOLE      8
#endif

#if defined(CONFIG_HPM_UART2) && (SERIAL_CONSOLE != 3)
#  define TTYS3_DEV           g_uart2priv /* LPUART2 is ttyS1 */
#  define SERIAL_CONSOLE      3
#elif defined(CONFIG_HPM_UART3) && (SERIAL_CONSOLE != 4)
#  define TTYS3_DEV           g_uart3priv /* LPUART3 is ttyS1 */
#  define SERIAL_CONSOLE      4
#elif defined(CONFIG_HPM_UART4) && (SERIAL_CONSOLE != 5)
#  define TTYS3_DEV           g_uart4priv /* LPUART4 is ttyS1 */
#  define SERIAL_CONSOLE      5
#elif defined(CONFIG_HPM_UART5) && (SERIAL_CONSOLE != 6)
#  define TTYS3_DEV           g_uart5priv /* LPUART5 is ttyS1 */
#  define SERIAL_CONSOLE      6
#elif defined(CONFIG_HPM_UART6) && (SERIAL_CONSOLE != 7)
#  define TTYS3_DEV           g_uart6priv /* LPUART6 is ttyS1 */
#  define SERIAL_CONSOLE      7
#elif defined(CONFIG_HPM_UART7) && (SERIAL_CONSOLE != 8)
#  define TTYS3_DEV           g_uart7priv /* LPUART7 is ttyS1 */
#  define SERIAL_CONSOLE      8
#endif

#if defined(CONFIG_HPM_UART3) && (SERIAL_CONSOLE != 4)
#  define TTYS4_DEV           g_uart3priv /* LPUART3 is ttyS1 */
#  define SERIAL_CONSOLE      4
#elif defined(CONFIG_HPM_UART4) && (SERIAL_CONSOLE != 5)
#  define TTYS4_DEV           g_uart4priv /* LPUART4 is ttyS1 */
#  define SERIAL_CONSOLE      5
#elif defined(CONFIG_HPM_UART5) && (SERIAL_CONSOLE != 6)
#  define TTYS4_DEV           g_uart5priv /* LPUART5 is ttyS1 */
#  define SERIAL_CONSOLE      6
#elif defined(CONFIG_HPM_UART6) && (SERIAL_CONSOLE != 7)
#  define TTYS4_DEV           g_uart6priv /* LPUART6 is ttyS1 */
#  define SERIAL_CONSOLE      7
#elif defined(CONFIG_HPM_UART7) && (SERIAL_CONSOLE != 8)
#  define TTYS4_DEV           g_uart7priv /* LPUART7 is ttyS1 */
#  define SERIAL_CONSOLE      8
#endif

#if defined(CONFIG_HPM_UART4) && (SERIAL_CONSOLE != 5)
#  define TTYS5_DEV           g_uart4priv /* LPUART4 is ttyS1 */
#  define SERIAL_CONSOLE      5
#elif defined(CONFIG_HPM_UART5) && (SERIAL_CONSOLE != 6)
#  define TTYS5_DEV           g_uart5priv /* LPUART5 is ttyS1 */
#  define SERIAL_CONSOLE      6
#elif defined(CONFIG_HPM_UART6) && (SERIAL_CONSOLE != 7)
#  define TTYS5_DEV           g_uart6priv /* LPUART6 is ttyS1 */
#  define SERIAL_CONSOLE      7
#elif defined(CONFIG_HPM_UART7) && (SERIAL_CONSOLE != 8)
#  define TTYS5_DEV           g_uart7priv /* LPUART7 is ttyS1 */
#  define SERIAL_CONSOLE      8
#endif

#if defined(CONFIG_HPM_UART5) && (SERIAL_CONSOLE != 6)
#  define TTYS6_DEV           g_uart5priv /* LPUART5 is ttyS1 */
#  define SERIAL_CONSOLE      6
#elif defined(CONFIG_HPM_UART6) && (SERIAL_CONSOLE != 7)
#  define TTYS6_DEV           g_uart6priv /* LPUART6 is ttyS1 */
#  define SERIAL_CONSOLE      7
#elif defined(CONFIG_HPM_UART7) && (SERIAL_CONSOLE != 8)
#  define TTYS6_DEV           g_uart7priv /* LPUART7 is ttyS1 */
#  define SERIAL_CONSOLE      8
#endif

#if defined(CONFIG_HPM_UART6) && (SERIAL_CONSOLE != 7)
#  define TTYS7_DEV           g_uart6priv /* LPUART6 is ttyS1 */
#  define SERIAL_CONSOLE      7
#elif defined(CONFIG_HPM_UART7) && (SERIAL_CONSOLE != 8)
#  define TTYS7_DEV           g_uart7priv /* LPUART7 is ttyS1 */
#  define SERIAL_CONSOLE      8
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of riscv_earlyserialinit(), riscv_serialinit(), and
 * up_putc().
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hpm_uart_s
{
  struct uart_dev_s dev;
  uint32_t  uartbase;          /* Base address of UART registers */
  uint32_t  baud;              /* Configured baud */
  uint32_t  irq;               /* IRQ associated with this UART */
  uint32_t  im;                /* Interrupt mask state */
  uint8_t   parity;            /* 0=none, 1=odd, 2=even */
  uint8_t   bits;              /* Number of bits (7 or 8) */
  bool      stopbits2;         /* true: Configure with 2 stop bits instead of 1 */
#if defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)
  uint8_t  inviflow:1;         /* Invert RTS sense */
  const uint32_t rts_gpio;     /* UART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t cts_gpio;     /* UART CTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t  iflow:1;         /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t  oflow:1;         /* output flow control (CTS) enabled */
#endif
#ifdef CONFIG_SERIAL_RS485CONTROL
  uint8_t rs485mode:1;      /* We are in RS485 (RTS on TX) mode */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline
uint32_t hpm_serialin(struct hpm_uart_s *priv, uint32_t offset);
static inline
void hpm_serialout(struct hpm_uart_s *priv, uint32_t offset, uint32_t value);
#if defined (CONFIG_SERIAL_TERMIOS) || defined (CONSOLE_DEV)
static inline void hpm_disableuartint(struct hpm_uart_s *priv, uint32_t *ie);
static inline void hpm_restoreuartint(struct hpm_uart_s *priv, uint32_t ie);
#endif

/* Serial driver methods */

static int  hpm_setup(struct uart_dev_s *dev);
static void hpm_shutdown(struct uart_dev_s *dev);
static int  hpm_attach(struct uart_dev_s *dev);
static void hpm_detach(struct uart_dev_s *dev);
static int  hpm_interrupt(int irq, void *context, void *arg);
static int  hpm_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  hpm_receive(struct uart_dev_s *dev, unsigned int *status);
static void hpm_rxint(struct uart_dev_s *dev, bool enable);
static bool hpm_rxavailable(struct uart_dev_s *dev);
static void hpm_txint(struct uart_dev_s *dev, bool enable);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool hpm_rxflowcontrol(struct uart_dev_s *dev,
                              unsigned int nbuffered, bool upper);
#endif
static void hpm_send(struct uart_dev_s *dev, int ch);
static bool hpm_txready(struct uart_dev_s *dev);
static bool hpm_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = hpm_setup,
  .shutdown       = hpm_shutdown,
  .attach         = hpm_attach,
  .detach         = hpm_detach,
  .ioctl          = hpm_ioctl,
  .receive        = hpm_receive,
  .rxint          = hpm_rxint,
  .rxavailable    = hpm_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = hpm_rxflowcontrol,
#endif
  .send           = hpm_send,
  .txint          = hpm_txint,
  .txready        = hpm_txready,
  .txempty        = hpm_txempty,
};

/* I/O buffers */

#ifdef CONFIG_HPM_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_HPM_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_HPM_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_HPM_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

#ifdef CONFIG_HPM_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

#ifdef CONFIG_HPM_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
#endif

#ifdef CONFIG_HPM_UART6
static char g_uart6rxbuffer[CONFIG_UART6_RXBUFSIZE];
static char g_uart6txbuffer[CONFIG_UART6_TXBUFSIZE];
#endif

#ifdef CONFIG_HPM_UART7
static char g_uart7rxbuffer[CONFIG_UART7_RXBUFSIZE];
static char g_uart7txbuffer[CONFIG_UART7_TXBUFSIZE];
#endif

#ifdef CONFIG_HPM_UART0
static struct hpm_uart_s g_uart0priv =
{
  .dev =
    {
      .recv        =
      {
        .size      = CONFIG_UART0_RXBUFSIZE,
        .buffer    = g_uart0rxbuffer,
      },
      .xmit        =
      {
        .size      = CONFIG_UART0_TXBUFSIZE,
        .buffer    = g_uart0txbuffer,
      },
      .ops         = &g_uart_ops,
    },
  .uartbase        = HPM_UART0_BASE,
  .irq             = HPM_IRQ_UART0,
  .im              = 0,
  .baud            = CONFIG_UART0_BAUD,
  .parity          = CONFIG_UART0_PARITY,
  .bits            = CONFIG_UART0_BITS,
  .stopbits2       = CONFIG_UART0_2STOP,
};
#endif

#ifdef CONFIG_HPM_UART1
static struct hpm_uart_s g_uart1priv =
{
  .dev =
    {
      .recv  =
      {
        .size    = CONFIG_UART1_RXBUFSIZE,
        .buffer  = g_uart1rxbuffer,
      },
      .xmit  =
      {
        .size      = CONFIG_UART1_TXBUFSIZE,
        .buffer    = g_uart1txbuffer,
      },
      .ops         = &g_uart_ops,
    },
  .uartbase  = HPM_UART1_BASE,
  .irq       = HPM_IRQ_UART1,
  .im        = 0,
  .baud      = CONFIG_UART1_BAUD,
  .parity    = CONFIG_UART1_PARITY,
  .bits      = CONFIG_UART1_BITS,
  .stopbits2 = CONFIG_UART1_2STOP,
};
#endif

#ifdef CONFIG_HPM_UART2
static struct hpm_uart_s g_uart2priv =
{
  .dev =
    {
      .recv  =
      {
        .size    = CONFIG_UART2_RXBUFSIZE,
        .buffer  = g_uart2rxbuffer,
      },
      .xmit  =
      {
        .size      = CONFIG_UART2_TXBUFSIZE,
        .buffer    = g_uart2txbuffer,
      },
      .ops         = &g_uart_ops,
    },
  .uartbase  = HPM_UART2_BASE,
  .irq       = HPM_IRQ_UART2,
  .im        = 0,
  .baud      = CONFIG_UART2_BAUD,
  .parity    = CONFIG_UART2_PARITY,
  .bits      = CONFIG_UART2_BITS,
  .stopbits2 = CONFIG_UART2_2STOP,
};
#endif

#ifdef CONFIG_HPM_UART3
static struct hpm_uart_s g_uart3priv =
{
  .dev =
    {
      .recv  =
      {
        .size    = CONFIG_UART3_RXBUFSIZE,
        .buffer  = g_uart3rxbuffer,
      },
      .xmit  =
      {
        .size      = CONFIG_UART3_TXBUFSIZE,
        .buffer    = g_uart3txbuffer,
      },
      .ops         = &g_uart_ops,
    },
  .uartbase  = HPM_UART3_BASE,
  .irq       = HPM_IRQ_UART3,
  .im        = 0,
  .baud      = CONFIG_UART3_BAUD,
  .parity    = CONFIG_UART3_PARITY,
  .bits      = CONFIG_UART3_BITS,
  .stopbits2 = CONFIG_UART3_2STOP,
};
#endif

#ifdef CONFIG_HPM_UART4
static struct hpm_uart_s g_uart4priv =
{
  .dev =
    {
      .recv  =
      {
        .size    = CONFIG_UART4_RXBUFSIZE,
        .buffer  = g_uart4rxbuffer,
      },
      .xmit  =
      {
        .size      = CONFIG_UART4_TXBUFSIZE,
        .buffer    = g_uart4txbuffer,
      },
      .ops         = &g_uart_ops,
    },
  .uartbase  = HPM_UART4_BASE,
  .irq       = HPM_IRQ_UART4,
  .im        = 0,
  .baud      = CONFIG_UART4_BAUD,
  .parity    = CONFIG_UART4_PARITY,
  .bits      = CONFIG_UART4_BITS,
  .stopbits2 = CONFIG_UART4_2STOP,
};
#endif

#ifdef CONFIG_HPM_UART5
static struct hpm_uart_s g_uart5priv =
{
  .dev =
    {
      .recv  =
      {
        .size    = CONFIG_UART5_RXBUFSIZE,
        .buffer  = g_uart5rxbuffer,
      },
      .xmit  =
      {
        .size      = CONFIG_UART5_TXBUFSIZE,
        .buffer    = g_uart5txbuffer,
      },
      .ops         = &g_uart_ops,
    },
  .uartbase  = HPM_UART5_BASE,
  .irq       = HPM_IRQ_UART5,
  .im        = 0,
  .baud      = CONFIG_UART5_BAUD,
  .parity    = CONFIG_UART5_PARITY,
  .bits      = CONFIG_UART5_BITS,
  .stopbits2 = CONFIG_UART5_2STOP,
};
#endif

#ifdef CONFIG_HPM_UART6
static struct hpm_uart_s g_uart6priv =
{
  .dev =
    {
      .recv  =
      {
        .size    = CONFIG_UART6_RXBUFSIZE,
        .buffer  = g_uart6rxbuffer,
      },
      .xmit  =
      {
        .size      = CONFIG_UART6_TXBUFSIZE,
        .buffer    = g_uart6txbuffer,
      },
      .ops         = &g_uart_ops,
    },
  .uartbase  = HPM_UART6_BASE,
  .irq       = HPM_IRQ_UART6,
  .im        = 0,
  .baud      = CONFIG_UART6_BAUD,
  .parity    = CONFIG_UART6_PARITY,
  .bits      = CONFIG_UART6_BITS,
  .stopbits2 = CONFIG_UART6_2STOP,
};
#endif

#ifdef CONFIG_HPM_UART7
static struct hpm_uart_s g_uart7priv =
{
  .dev =
    {
      .recv  =
      {
        .size    = CONFIG_UART7_RXBUFSIZE,
        .buffer  = g_uart7rxbuffer,
      },
      .xmit  =
      {
        .size      = CONFIG_UART7_TXBUFSIZE,
        .buffer    = g_uart7txbuffer,
      },
      .ops         = &g_uart_ops,
    },
  .uartbase  = HPM_UART7_BASE,
  .irq       = HPM_IRQ_UART7,
  .im        = 0,
  .baud      = CONFIG_UART7_BAUD,
  .parity    = CONFIG_UART7_PARITY,
  .bits      = CONFIG_UART7_BITS,
  .stopbits2 = CONFIG_UART7_2STOP,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_serialin
 ****************************************************************************/

static inline uint32_t hpm_serialin(struct hpm_uart_s *priv, uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: hpm_serialout
 ****************************************************************************/

static inline
void hpm_serialout(struct hpm_uart_s *priv, uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: hpm_disableuartint
 ****************************************************************************/

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONSOLE_DEV)
static inline void hpm_disableuartint(struct hpm_uart_s *priv, uint32_t *ie)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();
  regval = hpm_serialin(priv, HPM_UART_IER_OFFSET);

  /* Return the current Rx and Tx interrupt state */

  if (ie != NULL)
    {
      *ie = regval & UART_ALL_INTS;
    }

  regval &= ~UART_ALL_INTS;

  hpm_serialout(priv, HPM_UART_IER_OFFSET, regval);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: hpm_restoreuartint
 ****************************************************************************/

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONSOLE_DEV)
static inline void hpm_restoreuartint(struct hpm_uart_s *priv, uint32_t im)
{
  uint32_t regval;
  irqstate_t flags;

  /* Enable/disable any interrupts that are currently disabled but should be
   * enabled/disabled.
   */

  flags = enter_critical_section();
  regval = hpm_serialin(priv, HPM_UART_IER_OFFSET);
  regval &= ~UART_ALL_INTS;
  regval |= im;
  hpm_serialout(priv, HPM_UART_IER_OFFSET, regval);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: hpm_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int hpm_setup(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  int ret;
  struct uart_config_s config =
  {
    0
  };

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

  if ((priv->rts_gpio & GPIO_MODE_MASK) == GPIO_PERIPH)
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

  ret = hpm_uart_configure(priv->uartbase, &config);

  priv->im = hpm_serialin(priv, HPM_UART_IER_OFFSET) & UART_ALL_INTS;
  return ret;

#else
  priv->im = hpm_serialin(priv, HPM_UART_IER_OFFSET) & UART_ALL_INTS;

  return OK;
#endif /* CONFIG_SUPPRESS_UART_CONFIG */
}

/****************************************************************************
 * Name: hpm_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void hpm_shutdown(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;

  /* Disable interrupts */

  hpm_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: hpm_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int hpm_attach(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;
  int ret;

  ret = irq_attach(priv->irq, hpm_interrupt, dev);

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
 * Name: hpm_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void hpm_detach(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: hpm_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int hpm_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;
  uint32_t irq_id;

  irq_id = hpm_serialin(priv, HPM_UART_IIR_OFFSET) & UART_IIR_INTRID_MASK;

  if ((irq_id & UART_IIR_INTRID_RX_AVAILE) == UART_IIR_INTRID_RX_AVAILE)
    {
      uart_recvchars(dev);
    }

  if ((irq_id & UART_IIR_INTRID_TX_AVAILE) == UART_IIR_INTRID_TX_AVAILE)
    {
      uart_xmitchars(dev);
    }

  if ((irq_id & UART_IIR_INTRID_RX_TIMEOUT) == UART_IIR_INTRID_RX_TIMEOUT)
    {
      /* TODO: timeout logic */
    }

  if ((irq_id & UART_IIR_INTRID_RX_STATE) == UART_IIR_INTRID_RX_STATE)
    {
      /* TODO: line break */
    }

  if ((irq_id & UART_IIR_INTRID_MODEM_STATE) == UART_IIR_INTRID_MODEM_STATE)
    {
      /* TODO: modem state */
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int hpm_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;
  irqstate_t flags;
#endif

  int ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct hpm_uart_s *user = (struct hpm_uart_s *)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct hpm_uart_s));
          }
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

        termiosp->c_cflag =
          ((priv->parity != 0) ? PARENB : 0) |
          ((priv->parity == 1) ? PARODD : 0) |
          ((priv->stopbits2) ? CSTOPB : 0) |
          ((priv->bits == 5) ? CS5 : 0) |
          ((priv->bits == 6) ? CS6 : 0) |
          ((priv->bits == 7) ? CS7 : 0) |
          ((priv->bits == 8) ? CS8 : 0);

        /* Return flow control */

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        termiosp->c_cflag |= ((priv->oflow) ? CCTS_OFLOW : 0);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        termiosp->c_cflag |= ((priv->iflow) ? CRTS_IFLOW : 0);
#endif

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        uint32_t baud;
        uint32_t im;
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

        baud = cfgetispeed(termiosp);

        if (termiosp->c_cflag & PARENB)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        stop2 = (termiosp->c_cflag & CSTOPB) != 0;

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
            nbits = 0;
            ret = -EINVAL;
            break;
          }

        if (ret == OK)
          {
            priv->baud = baud;
            priv->parity = parity;
            priv->bits = nbits;
            priv->stopbits2 = stop2;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

            /* Effect the changes immediately - note that we do not implement
             * TCSADRAIN / TCSAFLUSH
             */

            flags = enter_critical_section();
            hpm_disableuartint(priv, &im);
            ret = dev->ops->setup(dev);

            /* Restore the interrupt state */

            hpm_restoreuartint(priv, im);
            priv->im = im;
            leave_critical_section(flags);
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_IMXRT_LPUART_SINGLEWIRE
    case TIOCSSINGLEWIRE:
    break;
#endif
#ifdef CONFIG_IMXRT_LPUART_INVERT
    case TIOCSINVERT:
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
 * Name: hpm_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int hpm_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;
  uint32_t rxdata = 0;

  if ((hpm_serialin(priv, HPM_UART_LSR_OFFSET) & UART_LSR_DR) == UART_LSR_DR)
    {
      rxdata = hpm_serialin(priv, HPM_UART_RBR_OFFSET) & UART_RBR_RBR_MASK;
      *status = 0;
    }
  else
    {
      *status = -1;
    }

  return (int)rxdata;
}

/****************************************************************************
 * Name: hpm_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void hpm_rxint(struct uart_dev_s *dev, bool enable)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_IER_ERBI;
#endif
    }
  else
    {
      priv->im &= ~UART_IER_ERBI;
    }

  hpm_serialout(priv, HPM_UART_IER_OFFSET, priv->im);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool hpm_rxavailable(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;

  return ((hpm_serialin(priv, HPM_UART_LSR_OFFSET) & UART_LSR_DR) != 0)
         ? true : false;
}

/****************************************************************************
 * Name: hpm_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void hpm_send(struct uart_dev_s *dev, int ch)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;

  while ((hpm_serialin(priv, HPM_UART_LSR_OFFSET)
          & UART_LSR_THRE) == 0)
    {
    }

  hpm_serialout(priv, HPM_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: hpm_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void hpm_txint(struct uart_dev_s *dev, bool enable)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_IER_ETHEI;
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~UART_IER_ETHEI;
    }

  hpm_serialout(priv, HPM_UART_IER_OFFSET, priv->im);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_txready
 *
 * Description:
 *   Return true if the tranmsit data register is not full
 *
 ****************************************************************************/

static bool hpm_txready(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;

  /* Return TRUE if the TX FIFO is not full */

  return (hpm_serialin(priv, HPM_UART_LSR_OFFSET) & UART_LSR_THRE)
         ? true : false;
}

/****************************************************************************
 * Name: hpm_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool hpm_txempty(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = (struct hpm_uart_s *)dev;

  /* Return TRUE if the TX wartermak is pending */

  return (hpm_serialin(priv, HPM_UART_LSR_OFFSET) & UART_LSR_TEMT)
         ? true : false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock iniialization
 *   performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.dev.isconsole = true;
  hpm_setup(&CONSOLE_DEV.dev);
#endif
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
  /* Register the console */

#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV.dev);
#endif

  /* Register all UARTs NOTE: we don't reorganize the numbering */

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV.dev);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV.dev);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV.dev);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV.dev);
#endif
#ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS4_DEV.dev);
#endif
#ifdef TTYS5_DEV
  uart_register("/dev/ttyS5", &TTYS5_DEV.dev);
#endif
#ifdef TTYS6_DEV
  uart_register("/dev/ttyS6", &TTYS6_DEV.dev);
#endif
#ifdef TTYS7_DEV
  uart_register("/dev/ttyS7", &TTYS7_DEV.dev);
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
  struct hpm_uart_s *priv = (struct hpm_uart_s *)CONSOLE_DEV.dev.priv;
  uint32_t im;

  hpm_disableuartint(priv, &im);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      hpm_lowputc('\r');
    }

  hpm_lowputc(ch);
  hpm_restoreuartint(priv, im);
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
#ifdef HAVE_UART_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
