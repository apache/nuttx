/****************************************************************************
 * drivers/serial/uart_16550.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/uart_16550.h>

#include <arch/board/board.h>

#ifdef CONFIG_16550_UART

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct u16550_s
{
  uart_addrwidth_t uartbase;  /* Base address of UART registers */
#ifndef CONFIG_16550_SUPRESS_CONFIG
  uint32_t         baud;      /* Configured baud */
  uint32_t         uartclk;   /* UART clock frequency */
#endif
  uart_datawidth_t ier;       /* Saved IER value */
  uint8_t          irq;       /* IRQ associated with this UART */
#ifndef CONFIG_16550_SUPRESS_CONFIG
  uint8_t          parity;    /* 0=none, 1=odd, 2=even */
  uint8_t          bits;      /* Number of bits (7 or 8) */
  bool             stopbits2; /* true: Configure with 2 stop bits instead of 1 */
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  bool             flow;      /* flow control (RTS/CTS) enabled */
#endif
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  u16550_setup(FAR struct uart_dev_s *dev);
static void u16550_shutdown(FAR struct uart_dev_s *dev);
static int  u16550_attach(FAR struct uart_dev_s *dev);
static void u16550_detach(FAR struct uart_dev_s *dev);
static int  u16550_interrupt(int irq, FAR void *context, FAR void *arg);
static int  u16550_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  u16550_receive(FAR struct uart_dev_s *dev, unsigned int *status);
static void u16550_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool u16550_rxavailable(FAR struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool u16550_rxflowcontrol(struct uart_dev_s *dev,
                                 unsigned int nbuffered, bool upper);
#endif
#ifdef CONFIG_SERIAL_TXDMA
static void u16550_dmasend(FAR struct uart_dev_s *dev);
static void u16550_dmatxavail(FAR struct uart_dev_s *dev);
#endif
#ifdef CONFIG_SERIAL_RXDMA
static void u16550_dmareceive(FAR struct uart_dev_s *dev);
static void u16550_dmarxfree(FAR struct uart_dev_s *dev);
#endif
static void u16550_send(FAR struct uart_dev_s *dev, int ch);
static void u16550_txint(FAR struct uart_dev_s *dev, bool enable);
static bool u16550_txready(FAR struct uart_dev_s *dev);
static bool u16550_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
#ifdef CONFIG_SERIAL_TXDMA
  .dmasend        = u16550_dmasend,
#endif
#ifdef CONFIG_SERIAL_RXDMA
  .dmareceive     = u16550_dmareceive,
  .dmarxfree      = u16550_dmarxfree,
#endif
#ifdef CONFIG_SERIAL_TXDMA
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

/* This describes the state of the 16550 uart0 port. */

#ifdef CONFIG_16550_UART0
static struct u16550_s g_uart0priv =
{
  .uartbase       = CONFIG_16550_UART0_BASE,
#ifndef CONFIG_16550_SUPRESS_CONFIG
  .baud           = CONFIG_16550_UART0_BAUD,
  .uartclk        = CONFIG_16550_UART0_CLOCK,
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
  .uartbase       = CONFIG_16550_UART1_BASE,
#ifndef CONFIG_16550_SUPRESS_CONFIG
  .baud           = CONFIG_16550_UART1_BAUD,
  .uartclk        = CONFIG_16550_UART1_CLOCK,
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
  .uartbase       = CONFIG_16550_UART2_BASE,
#ifndef CONFIG_16550_SUPRESS_CONFIG
  .baud           = CONFIG_16550_UART2_BAUD,
  .uartclk        = CONFIG_16550_UART2_CLOCK,
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
  .uartbase       = CONFIG_16550_UART3_BASE,
#ifndef CONFIG_16550_SUPRESS_CONFIG
  .baud           = CONFIG_16550_UART3_BAUD,
  .uartclk        = CONFIG_16550_UART3_CLOCK,
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

/****************************************************************************
 * Name: u16550_serialin
 ****************************************************************************/

static inline uart_datawidth_t u16550_serialin(FAR struct u16550_s *priv,
                                               int offset)
{
#ifdef CONFIG_SERIAL_UART_ARCH_MMIO
  return *((FAR volatile uart_datawidth_t *)priv->uartbase + offset);
#else
  return uart_getreg(priv->uartbase, offset);
#endif
}

/****************************************************************************
 * Name: u16550_serialout
 ****************************************************************************/

static inline void u16550_serialout(FAR struct u16550_s *priv, int offset,
                                    uart_datawidth_t value)
{
#ifdef CONFIG_SERIAL_UART_ARCH_MMIO
  *((FAR volatile uart_datawidth_t *)priv->uartbase + offset) = value;
#else
  uart_putreg(priv->uartbase, offset, value);
#endif
}

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
 * Name: u16550_restoreuartint
 ****************************************************************************/

static inline void u16550_restoreuartint(FAR struct u16550_s *priv,
                                         uint32_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
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
  return (priv->uartclk + (priv->baud << 3)) / (priv->baud << 4);
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
  uint16_t div;
  uint32_t lcr;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  uint32_t mcr;
#endif

  /* Clear fifos */

  u16550_serialout(priv, UART_FCR_OFFSET,
                   (UART_FCR_RXRST | UART_FCR_TXRST));

  /* Set trigger */

  u16550_serialout(priv, UART_FCR_OFFSET,
                   (UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_8));

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

  /* Enter DLAB=1 */

  u16550_serialout(priv, UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  /* Set the BAUD divisor */

  div = u16550_divisor(priv);
  u16550_serialout(priv, UART_DLM_OFFSET, div >> 8);
  u16550_serialout(priv, UART_DLL_OFFSET, div & 0xff);

  /* Clear DLAB */

  u16550_serialout(priv, UART_LCR_OFFSET, lcr);

  /* Configure the FIFOs */

  u16550_serialout(priv, UART_FCR_OFFSET,
                   (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST | UART_FCR_RXRST |
                    UART_FCR_FIFOEN));

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

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, u16550_interrupt, dev);
#ifndef CONFIG_ARCH_NOINTC
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
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

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: u16550_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate u16550_s structure in order to call these functions.
 *
 ****************************************************************************/

static int u16550_interrupt(int irq, FAR void *context, FAR void *arg)
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
  ret = uart_ioctl(filep, cmd, arg);

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
      /* Disable Rx interrupt to prevent more data being from
       * peripheral if the RX buffer is near full. When hardware
       * RTS is enabled, this will prevent more data from coming
       * in. Otherwise, enable Rx interrupt to make sure that more
       * input is received.
       */

      u16550_rxint(dev, !upper);
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

#ifdef CONFIG_SERIAL_TXDMA
static void u16550_dmasend(FAR struct uart_dev_s *dev)
{
}
#endif

#ifdef CONFIG_SERIAL_RXDMA
static void u16550_dmareceive(FAR struct uart_dev_s *dev)
{
}

static void u16550_dmarxfree(FAR struct uart_dev_s *dev)
{
}
#endif

#ifdef CONFIG_SERIAL_TXDMA
static void u16550_dmatxavail(FAR struct uart_dev_s *dev)
{
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
 * Name: u16550_putc
 *
 * Description:
 *   Write one character to the UART (polled)
 *
 ****************************************************************************/

#ifdef HAVE_16550_CONSOLE
static void u16550_putc(FAR struct u16550_s *priv, int ch)
{
  while ((u16550_serialin(priv, UART_LSR_OFFSET) & UART_LSR_THRE) == 0);
  u16550_serialout(priv, UART_THR_OFFSET, (uart_datawidth_t)ch);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
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

void up_earlyserialinit(void)
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
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
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
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

#ifdef HAVE_16550_CONSOLE
int up_putc(int ch)
{
  FAR struct u16550_s *priv = (FAR struct u16550_s *)CONSOLE_DEV.priv;
  uart_datawidth_t ier;

  u16550_disableuartint(priv, &ier);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      u16550_putc(priv, '\r');
    }

  u16550_putc(priv, ch);
  u16550_restoreuartint(priv, ier);
  return ch;
}
#endif

#endif /* CONFIG_16550_UART */
