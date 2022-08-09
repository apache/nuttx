/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_uart.c
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
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include "ameba_uart.h"
#include <arch/board/board.h>
#ifdef CONFIG_AMEBA_UART

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum
{
  UART_0        = 0,  /* !< 1-byte */
  UART_1        = 1,  /* !< 4-byte/8-byte(for 8195B/8710C) */
  UART_2        = 2,  /* !< 8-byte/16-byte(for 8195B/8710C) */
  UART_3        = 3,  /* !< 14-byte/30-bytes(for 8195B/8710C) */
  MAX_UART_NUM  = 4
};

enum
{
  UART_PIN_TX         = 0,
  UART_PIN_RX         = 1,
  UART_PIN_RTS        = 2,
  UART_PIN_CTS        = 3
};

enum
{
    PIN_PULLNONE  = 0,
    PIN_PULLDOWN  = 1,
    PIN_PULLUP    = 2,
    PIN_PULLDEFAULT = PIN_PULLNONE
};

enum
{
  FIFOLV1BYTE   = 0,  /* !< 1-byte */
  FIFOLVQUARTER = 1,  /* !< 4-byte/8-byte(for 8195B/8710C) */
  FIFOLVHALF    = 2,  /* !< 8-byte/16-byte(for 8195B/8710C) */
  FIFOLVFULL    = 3   /* !< 14-byte/30-bytes(for 8195B/8710C) */
};

enum
{
  FLOWCONTROLNONE,  /* !<none RTS/CTS */
  FLOWCONTROLRTS,   /* !<RTS enable */
  FLOWCONTROLCTS,   /* !<CTS enable */
  FLOWCONTROLRTSCTS /* !<RTS/CTS enable */
};

struct ameba_s
{
  hal_uart_adapter_t  adapter;
  uint32_t tx;
  uint32_t rx;
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  uint32_t            baud;      /* Configured baud */
#endif
  uart_datawidth_t    ier;       /* Saved IER value */
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  uint8_t             parity;    /* 0=none, 1=odd, 2=even */
  uint8_t             bits;      /* Number of bits (7 or 8) */
  bool                stopbits2; /* true: Configure with 2 stop bits instead of 1 */
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  bool                flow;      /* flow control (RTS/CTS) enabled */
#endif
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  ameba_setup(struct uart_dev_s *dev);
static void ameba_shutdown(struct uart_dev_s *dev);
static int  ameba_attach(struct uart_dev_s *dev);
static void ameba_detach(struct uart_dev_s *dev);
static void ameba_interrupt(uint32_t id, uint32_t event);
static int  ameba_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  ameba_receive(struct uart_dev_s *dev, uint32_t *status);
static void ameba_rxint(struct uart_dev_s *dev, bool enable);
static bool ameba_rxavailable(struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool ameba_rxflowcontrol(struct uart_dev_s *dev,
                                unsigned int nbuffered,
                                bool upper);
#endif
#ifdef CONFIG_SERIAL_TXDMA
static void ameba_dmasend(struct uart_dev_s *dev);
static void ameba_dmatxavail(struct uart_dev_s *dev);
#endif
#ifdef CONFIG_SERIAL_RXDMA
static void ameba_dmareceive(struct uart_dev_s *dev);
static void ameba_dmarxfree(struct uart_dev_s *dev);
#endif
static void ameba_send(struct uart_dev_s *dev, int ch);
static void ameba_txint(struct uart_dev_s *dev, bool enable);
static bool ameba_txready(struct uart_dev_s *dev);
static bool ameba_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern const hal_uart_func_stubs_t hal_uart_stubs;
extern const hal_gpio_func_stubs_t hal_gpio_stubs;
extern void hal_syson_wakeup_uart_func_reset(void);
static const struct uart_ops_s g_uart_ops =
{
  .setup          = ameba_setup,
  .shutdown       = ameba_shutdown,
  .attach         = ameba_attach,
  .detach         = ameba_detach,
  .ioctl          = ameba_ioctl,
  .receive        = ameba_receive,
  .rxint          = ameba_rxint,
  .rxavailable    = ameba_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = ameba_rxflowcontrol,
#endif
#ifdef CONFIG_SERIAL_TXDMA
  .dmasend        = ameba_dmasend,
#endif
#ifdef CONFIG_SERIAL_RXDMA
  .dmareceive     = ameba_dmareceive,
  .dmarxfree      = ameba_dmarxfree,
#endif
#ifdef CONFIG_SERIAL_TXDMA
  .dmatxavail     = ameba_dmatxavail,
#endif
  .send           = ameba_send,
  .txint          = ameba_txint,
  .txready        = ameba_txready,
  .txempty        = ameba_txempty,
};

/* I/O buffers */

#ifdef CONFIG_AMEBA_UART0
static char g_uart0rxbuffer[CONFIG_AMEBA_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_AMEBA_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_AMEBA_UART1
static char g_uart1rxbuffer[CONFIG_AMEBA_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_AMEBA_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_AMEBA_UART2
static char g_uart2rxbuffer[CONFIG_AMEBA_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_AMEBA_UART2_TXBUFSIZE];
#endif
#ifdef CONFIG_AMEBA_UART3
static char g_uart3rxbuffer[CONFIG_AMEBA_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_AMEBA_UART3_TXBUFSIZE];
#endif

/* This describes the state of the ameba uart0 port. */

#ifdef CONFIG_AMEBA_UART0
static struct ameba_s g_uart0priv =
{
  .tx             = CONFIG_AMEBA_UART0_TX_PIN,
  .rx             = CONFIG_AMEBA_UART0_RX_PIN,
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  .baud           = CONFIG_AMEBA_UART0_BAUD,
#endif
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  .parity         = CONFIG_AMEBA_UART0_PARITY,
  .bits           = CONFIG_AMEBA_UART0_BITS,
  .stopbits2      = CONFIG_AMEBA_UART0_2STOP,
#if defined(CONFIG_AMEBA_UART0_IFLOWCONTROL) || defined(CONFIG_AMEBA_UART0_OFLOWCONTROL)
  .flow           = true,
#endif
#endif
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_AMEBA_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_AMEBA_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};

#endif

/* This describes the state of the ameba uart1 port. */

#ifdef CONFIG_AMEBA_UART1
static struct ameba_s g_uart1priv =
{
  .tx             = CONFIG_AMEBA_UART1_TX_PIN,
  .rx             = CONFIG_AMEBA_UART1_RX_PIN,
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  .baud           = CONFIG_AMEBA_UART1_BAUD,
#endif
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  .parity         = CONFIG_AMEBA_UART1_PARITY,
  .bits           = CONFIG_AMEBA_UART1_BITS,
  .stopbits2      = CONFIG_AMEBA_UART1_2STOP,
#if defined(CONFIG_AMEBA_UART1_IFLOWCONTROL) || defined(CONFIG_AMEBA_UART1_OFLOWCONTROL)
  .flow           = true,
#endif
#endif
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_AMEBA_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_AMEBA_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};

#endif

/* This describes the state of the ameba uart1 port. */

#ifdef CONFIG_AMEBA_UART2
static struct ameba_s g_uart2priv =
{
  .tx             = CONFIG_AMEBA_UART2_TX_PIN,
  .rx             = CONFIG_AMEBA_UART2_RX_PIN,
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  .baud           = CONFIG_AMEBA_UART2_BAUD,
#endif
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  .parity         = CONFIG_AMEBA_UART2_PARITY,
  .bits           = CONFIG_AMEBA_UART2_BITS,
  .stopbits2      = CONFIG_AMEBA_UART2_2STOP,
#if defined(CONFIG_AMEBA_UART2_IFLOWCONTROL) || defined(CONFIG_AMEBA_UART2_OFLOWCONTROL)
  .flow           = true,
#endif
#endif
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_AMEBA_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_AMEBA_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};

#endif

/* This describes the state of the ameba uart1 port. */

#ifdef CONFIG_AMEBA_UART3
static struct ameba_s g_uart3priv =
{
  .tx             = CONFIG_AMEBA_UART3_TX_PIN,
  .rx             = CONFIG_AMEBA_UART3_RX_PIN,
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  .baud           = CONFIG_AMEBA_UART3_BAUD,
#endif
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  .parity         = CONFIG_AMEBA_UART3_PARITY,
  .bits           = CONFIG_AMEBA_UART3_BITS,
  .stopbits2      = CONFIG_AMEBA_UART3_2STOP,
#if defined(CONFIG_AMEBA_UART3_IFLOWCONTROL) || defined(CONFIG_AMEBA_UART3_OFLOWCONTROL)
  .flow           = true,
#endif
#endif
};

static uart_dev_t g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_AMEBA_UART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_AMEBA_UART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};

#endif

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#ifdef CONFIG_AMEBA_SERIAL_DISABLE_REORDERING
#  if defined(CONFIG_AMEBA_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port    /* UART0=console */
#  elif defined(CONFIG_AMEBA_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port    /* UART1=console */
#  elif defined(CONFIG_AMEBA_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2port    /* UART2=console */
#  elif defined(CONFIG_AMEBA_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3port    /* UART3=console */
#  endif
#  ifdef CONFIG_AMEBA_UART0
#    define TTYS0_DEV       g_uart0port
#  endif
#  ifdef CONFIG_AMEBA_UART1
#    define TTYS1_DEV       g_uart1port
#  endif
#  ifdef CONFIG_AMEBA_UART2
#    define TTYS2_DEV       g_uart2port
#  endif
#  ifdef CONFIG_AMEBA_UART3
#    define TTYS3_DEV       g_uart3port
#  endif
#else  /* CONFIG_AMEBA_SERIAL_DISABLE_REORDERING */

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#  if defined(CONFIG_AMEBA_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port    /* UART0=console */
#    define TTYS0_DEV       g_uart0port    /* UART0=ttyS0 */
#    ifdef CONFIG_AMEBA_UART1
#      define TTYS1_DEV     g_uart1port    /* UART0=ttyS0;UART1=ttyS1 */
#      ifdef CONFIG_AMEBA_UART2
#        define TTYS2_DEV   g_uart2port    /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2 */
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS3_DEV g_uart3port    /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS2_DEV g_uart3port    /* UART0=ttyS0;UART1=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART0=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_AMEBA_UART2
#        define TTYS1_DEV   g_uart2port    /* UART0=ttyS0;UART2=ttyS1;No ttyS3 */
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS2_DEV g_uart3port    /* UART0=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART0=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS1_DEV g_uart3port    /* UART0=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* UART0=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#          undef TTYS2_DEV                 /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_AMEBA_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port    /* UART1=console */
#    define TTYS0_DEV       g_uart1port    /* UART1=ttyS0 */
#    ifdef CONFIG_AMEBA_UART0
#      define TTYS1_DEV     g_uart0port    /* UART1=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_AMEBA_UART2
#        define TTYS2_DEV   g_uart2port    /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2 */
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS3_DEV g_uart3port    /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS2_DEV g_uart3port    /* UART1=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART1=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_AMEBA_UART2
#        define TTYS1_DEV   g_uart2port    /* UART1=ttyS0;UART2=ttyS1 */
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS2_DEV g_uart3port    /* UART1=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART1=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS1_DEV   g_uart3port  /* UART1=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* UART1=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                   /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_AMEBA_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2port    /* UART2=console */
#    define TTYS0_DEV       g_uart2port    /* UART2=ttyS0 */
#    ifdef CONFIG_AMEBA_UART0
#      define TTYS1_DEV     g_uart0port    /* UART2=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_AMEBA_UART1
#        define TTYS2_DEV   g_uart1port    /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS3_DEV g_uart3port    /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS2_DEV g_uart3port    /* UART2=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART2=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_AMEBA_UART1
#        define TTYS1_DEV   g_uart1port    /* UART2=ttyS0;UART1=ttyS1 */
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS2_DEV g_uart3port    /* UART2=ttyS0;UART1=ttyS1;UART3=ttyS2 */
#        else
#          undef TTYS2_DEV                 /* UART2=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_AMEBA_UART3
#          define TTYS1_DEV g_uart3port    /* UART2=ttyS0;UART3=ttyS1;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* UART2=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                   /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_AMEBA_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3port    /* UART3=console */
#    define TTYS0_DEV       g_uart3port    /* UART3=ttyS0 */
#    ifdef CONFIG_AMEBA_UART0
#      define TTYS1_DEV     g_uart0port    /* UART3=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_AMEBA_UART1
#        define TTYS2_DEV   g_uart1port    /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#        ifdef CONFIG_AMEBA_UART2
#          define TTYS3_DEV g_uart2port    /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2;UART2=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_AMEBA_UART2
#          define TTYS2_DEV g_uart2port    /* UART3=ttyS0;UART0=ttyS1;UART2=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART3=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#          undef TTYS3_DEV                 /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_AMEBA_UART1
#        define TTYS1_DEV   g_uart1port    /* UART3=ttyS0;UART1=ttyS1 */
#        ifdef CONFIG_AMEBA_UART2
#          define TTYS2_DEV g_uart2port    /* UART3=ttyS0;UART1=ttyS1;UART2=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* UART3=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_AMEBA_UART2
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
#endif /* CONFIG_AMEBA_SERIAL_DISABLE_REORDERING */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int ameba_setup(struct uart_dev_s *dev)
{
  uint32_t status = OK;
#ifndef CONFIG_AMEBA_SUPRESS_CONFIG
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  int uart_idx;
  uart_idx = hal_uart_stubs.hal_uart_pin_to_idx(priv->rx, UART_PIN_RX);
  if (uart_idx != hal_uart_stubs.hal_uart_pin_to_idx(priv->tx, UART_PIN_TX)
      || uart_idx > MAX_UART_PORT)
    {
      return -EINVAL;
    }

  if (!priv->adapter.is_inited)
    {
      if (uart_idx <= UART_2)
        {
          hal_gpio_stubs.hal_gpio_pull_ctrl(priv->rx, PIN_PULLUP);
        }

      if (uart_idx == UART_0)
        {
          hal_syson_wakeup_uart_func_reset();
        }

      else if (uart_idx == UART_2)
        {
          priv->adapter.is_inited = true;
        }

      status = hal_uart_stubs.hal_uart_init(&priv->adapter,
                                            priv->tx, priv->rx, NULL);
      if (status != OK)
        {
          return status;
        }

      if (uart_idx < UART_2)
        {
          hal_pinmux_register(priv->tx, (PID_UART0 + uart_idx));
          hal_pinmux_register(priv->rx, (PID_UART0 + uart_idx));
        }

      if (uart_idx == UART_3)
        {
          priv->adapter.base_addr->fcr_b.rxfifo_trigger_level = FIFOLVHALF;
          hal_uart_stubs.hal_uart_set_flow_control(&priv->adapter,
              FLOWCONTROLRTS);
        }

      else
        {
          priv->adapter.base_addr->fcr_b.rxfifo_trigger_level = FIFOLV1BYTE;
          hal_uart_stubs.hal_uart_set_flow_control(&priv->adapter,
              FLOWCONTROLNONE);
        }
    }

  status = hal_uart_stubs.hal_uart_set_baudrate(&priv->adapter, priv->baud);
  if (status != OK)
    {
      return status;
    }

  status = hal_uart_stubs.hal_uart_set_format(&priv->adapter,
           priv->bits, priv->parity, priv->stopbits2 ? 2 : 1);
#endif
  return status;
}

/****************************************************************************
 * Name: ameba_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void ameba_shutdown(struct uart_dev_s *dev)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  uint32_t uart_idx = priv->adapter.uart_idx;
  if (uart_idx == UART_2)
    {
      priv->adapter.is_inited = false;
    }

  hal_uart_stubs.hal_uart_deinit(&priv->adapter);
  if (uart_idx > UART_2)
    {
      return;
    }

  hal_pinmux_unregister(priv->adapter.tx_pin, (PID_UART0 + uart_idx));
  hal_pinmux_unregister(priv->adapter.rx_pin, (PID_UART0 + uart_idx));
}

/****************************************************************************
 * Name: ameba_attach
 *
 * Description:
 *  Configure the UART to operation in interrupt driven mode.  This method is
 *  called when the serial port is opened.  Normally, this is just after the
 *  the setup() method is called, however, the serial console may operate in
 *  a non-interrupt driven mode during the boot phase.
 *
 *  RX and TX interrupts are not enabled when by the attach method (unless
 *  the hardware supports multiple levels of interrupt enabling).  The RX and
 *  TX interrupts are not enabled until the txint() and rxint() methods are
 *  called
 *
 ****************************************************************************/

static int ameba_attach(struct uart_dev_s *dev)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  hal_uart_stubs.hal_uart_txtd_hook(&priv->adapter,
                                    ameba_interrupt, (uintptr_t)dev, 0);
  hal_uart_stubs.hal_uart_rxind_hook(&priv->adapter,
                                     ameba_interrupt, (uintptr_t)dev, 1);
  return OK;
}

/****************************************************************************
 * Name: ameba_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void ameba_detach(struct uart_dev_s *dev)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  hal_uart_stubs.hal_uart_txtd_hook(&priv->adapter,
                                    NULL, (uintptr_t)NULL, 0);
  hal_uart_stubs.hal_uart_rxind_hook(&priv->adapter,
                                     NULL, (uintptr_t)NULL, 0);
}

/****************************************************************************
 * Name: ameba_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate ameba_s structure in order to call these functions.
 *
 ****************************************************************************/

static void ameba_interrupt(uint32_t id, uint32_t event)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)id;
  if (event == 0)
    {
      uart_xmitchars(dev);
    }

  else
    {
      uart_recvchars(dev);
    }
}

/****************************************************************************
 * Name: ameba_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int ameba_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
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
#if defined(CONFIG_SERIAL_TERMIOS) && !defined(CONFIG_AMEBA_SUPRESS_CONFIG)
    case TCGETS:
    {
      struct termios *termiosp = (struct termios *)arg;
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
      struct termios *termiosp = (struct termios *)arg;
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
      ameba_setup(dev);
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
 * Name: ameba_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int ameba_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  return hal_uart_stubs.hal_uart_getc(&priv->adapter);
}

/****************************************************************************
 * Name: ameba_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void ameba_rxint(struct uart_dev_s *dev, bool enable)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  hal_uart_stubs.hal_uart_set_rts(&priv->adapter, enable);
}

/****************************************************************************
 * Name: ameba_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool ameba_rxavailable(struct uart_dev_s *dev)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  return hal_uart_stubs.hal_uart_readable(&priv->adapter);
}

/****************************************************************************
 * Name: ameba_dma*
 *
 * Description:
 *   Stubbed out DMA-related methods
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool ameba_rxflowcontrol(struct uart_dev_s *dev,
                                unsigned int nbuffered,
                                bool upper)
{
  return false;
}

#endif

/****************************************************************************
 * Name: ameba_dma*
 *
 * Description:
 *   Stub functions used when serial DMA is enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
static void ameba_dmasend(struct uart_dev_s *dev)
{
}

#endif
#ifdef CONFIG_SERIAL_RXDMA
static void ameba_dmareceive(struct uart_dev_s *dev)
{
}

static void ameba_dmarxfree(struct uart_dev_s *dev)
{
}

#endif
#ifdef CONFIG_SERIAL_TXDMA
static void ameba_dmatxavail(struct uart_dev_s *dev)
{
}

#endif

/****************************************************************************
 * Name: ameba_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void ameba_send(struct uart_dev_s *dev, int ch)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  while (hal_uart_stubs.hal_uart_writeable(&priv->adapter) == 0);
  hal_uart_stubs.hal_uart_putc(&priv->adapter, ch);
}

/****************************************************************************
 * Name: ameba_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void ameba_txint(struct uart_dev_s *dev, bool enable)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  priv->adapter.base_addr->ier_b.etbei = enable;
  if (enable)
    {
      uart_xmitchars(dev);
    }
}

/****************************************************************************
 * Name: ameba_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool ameba_txready(struct uart_dev_s *dev)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  return hal_uart_stubs.hal_uart_writeable(&priv->adapter);
}

/****************************************************************************
 * Name: ameba_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool ameba_txempty(struct uart_dev_s *dev)
{
  struct ameba_s *priv = (struct ameba_s *)dev->priv;
  return priv->adapter.base_addr->tflvr_b.tx_fifo_lv > 0 ? 0 : 1;
}

/****************************************************************************
 * Name: ameba_putc
 *
 * Description:
 *   Write one character to the UART (polled)
 *
 ****************************************************************************/

#ifdef HAVE_AMEBA_CONSOLE
static void ameba_putc(struct ameba_s *priv, int ch)
{
  while (hal_uart_stubs.hal_uart_writeable(&priv->adapter) == 0);
  hal_uart_stubs.hal_uart_putc(&priv->adapter, ch);
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
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

void arm_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
#ifndef CONFIG_AMEBA_SUPRESS_INITIAL_CONFIG
  ameba_setup(&CONSOLE_DEV);
#endif
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
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

#ifdef HAVE_AMEBA_CONSOLE
int up_putc(int ch)
{
  struct ameba_s *priv = (struct ameba_s *)CONSOLE_DEV.priv;

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      ameba_putc(priv, '\r');
    }

  ameba_putc(priv, ch);
  return ch;
}

#endif
#endif /* CONFIG_AMEBA_UART */
