/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_serial.c
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
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "gd32vw55x_config.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_dma.h"
#include "gd32vw55x_lowputc.h"
#include "hardware/gd32vw55x_uart.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which UART will be tty0/console and which tty1/tty2? */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart0port
#  define TTYS0_DEV       g_usart0port
#  define USART0_ASSIGNED 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart1port
#  define TTYS0_DEV       g_uart1port
#  define UART1_ASSIGNED  1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart2port
#  define TTYS0_DEV       g_uart2port
#  define UART2_ASSIGNED  1
#elif defined(CONFIG_GD32VW55X_USART0)
#  define TTYS0_DEV       g_usart0port
#  define USART0_ASSIGNED 1
#elif defined(CONFIG_GD32VW55X_UART1)
#  define TTYS0_DEV       g_uart1port
#  define UART1_ASSIGNED  1
#elif defined(CONFIG_GD32VW55X_UART2)
#  define TTYS0_DEV       g_uart2port
#  define UART2_ASSIGNED  1
#endif

/* Assign remaining UARTs to ttyS1/ttyS2 */

#if defined(CONFIG_GD32VW55X_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV       g_usart0port
#  define USART0_ASSIGNED 1
#elif defined(CONFIG_GD32VW55X_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV       g_uart1port
#  define UART1_ASSIGNED  1
#elif defined(CONFIG_GD32VW55X_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV       g_uart2port
#  define UART2_ASSIGNED  1
#endif

#if defined(CONFIG_GD32VW55X_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV       g_uart1port
#  define UART1_ASSIGNED  1
#elif defined(CONFIG_GD32VW55X_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV       g_uart2port
#  define UART2_ASSIGNED  1
#endif

/* Rx DMA: the received characters go to a circular DMA buffer */

#if defined(CONFIG_USART0_RXDMA) || defined(CONFIG_UART1_RXDMA) || \
    defined(CONFIG_UART2_RXDMA)
#  ifndef CONFIG_GD32VW55X_DMA
#    error "UART Rx DMA requires CONFIG_GD32VW55X_DMA"
#  endif
#  define SERIAL_HAVE_RXDMA 1
#  define RXDMA_BUFFER_SIZE CONFIG_GD32VW55X_SERIAL_RXDMA_BUFFER_SIZE
#endif

/* Tx DMA: the characters are sent from the transmit buffer by the DMA */

#if defined(CONFIG_USART0_TXDMA) || defined(CONFIG_UART1_TXDMA) || \
    defined(CONFIG_UART2_TXDMA)
#  ifndef CONFIG_GD32VW55X_DMA
#    error "UART Tx DMA requires CONFIG_GD32VW55X_DMA"
#  endif
#  define SERIAL_HAVE_TXDMA 1
#endif

/* The operations of each UART, depending on the DMA it uses */

#if defined(CONFIG_USART0_RXDMA) && defined(CONFIG_USART0_TXDMA)
#  define USART0_OPS g_uart_rxtxdma_ops
#elif defined(CONFIG_USART0_RXDMA)
#  define USART0_OPS g_uart_rxdma_ops
#elif defined(CONFIG_USART0_TXDMA)
#  define USART0_OPS g_uart_txdma_ops
#else
#  define USART0_OPS g_uart_ops
#endif

#if defined(CONFIG_UART1_RXDMA) && defined(CONFIG_UART1_TXDMA)
#  define UART1_OPS  g_uart_rxtxdma_ops
#elif defined(CONFIG_UART1_RXDMA)
#  define UART1_OPS  g_uart_rxdma_ops
#elif defined(CONFIG_UART1_TXDMA)
#  define UART1_OPS  g_uart_txdma_ops
#else
#  define UART1_OPS  g_uart_ops
#endif

#if defined(CONFIG_UART2_RXDMA) && defined(CONFIG_UART2_TXDMA)
#  define UART2_OPS  g_uart_rxtxdma_ops
#elif defined(CONFIG_UART2_RXDMA)
#  define UART2_OPS  g_uart_rxdma_ops
#elif defined(CONFIG_UART2_TXDMA)
#  define UART2_OPS  g_uart_txdma_ops
#else
#  define UART2_OPS  g_uart_ops
#endif

#if (defined(CONFIG_GD32VW55X_USART0) && \
     !defined(CONFIG_USART0_RXDMA) && !defined(CONFIG_USART0_TXDMA)) || \
    (defined(CONFIG_GD32VW55X_UART1) && \
     !defined(CONFIG_UART1_RXDMA) && !defined(CONFIG_UART1_TXDMA)) || \
    (defined(CONFIG_GD32VW55X_UART2) && \
     !defined(CONFIG_UART2_RXDMA) && !defined(CONFIG_UART2_TXDMA))
#  define SERIAL_HAVE_NODMA 1
#endif

#if (defined(CONFIG_GD32VW55X_USART0) && defined(CONFIG_USART0_RXDMA) && \
     !defined(CONFIG_USART0_TXDMA)) || \
    (defined(CONFIG_GD32VW55X_UART1) && defined(CONFIG_UART1_RXDMA) && \
     !defined(CONFIG_UART1_TXDMA)) || \
    (defined(CONFIG_GD32VW55X_UART2) && defined(CONFIG_UART2_RXDMA) && \
     !defined(CONFIG_UART2_TXDMA))
#  define SERIAL_HAVE_RXDMA_OPS 1
#endif

#if (defined(CONFIG_GD32VW55X_USART0) && !defined(CONFIG_USART0_RXDMA) && \
     defined(CONFIG_USART0_TXDMA)) || \
    (defined(CONFIG_GD32VW55X_UART1) && !defined(CONFIG_UART1_RXDMA) && \
     defined(CONFIG_UART1_TXDMA)) || \
    (defined(CONFIG_GD32VW55X_UART2) && !defined(CONFIG_UART2_RXDMA) && \
     defined(CONFIG_UART2_TXDMA))
#  define SERIAL_HAVE_TXDMA_OPS 1
#endif

#if (defined(CONFIG_GD32VW55X_USART0) && defined(CONFIG_USART0_RXDMA) && \
     defined(CONFIG_USART0_TXDMA)) || \
    (defined(CONFIG_GD32VW55X_UART1) && defined(CONFIG_UART1_RXDMA) && \
     defined(CONFIG_UART1_TXDMA)) || \
    (defined(CONFIG_GD32VW55X_UART2) && defined(CONFIG_UART2_RXDMA) && \
     defined(CONFIG_UART2_TXDMA))
#  define SERIAL_HAVE_RXTXDMA_OPS 1
#endif

/* Reception by interrupt, and transmission by interrupt */

#if defined(SERIAL_HAVE_NODMA) || defined(SERIAL_HAVE_TXDMA_OPS)
#  define SERIAL_HAVE_INTRX 1
#endif

#if defined(SERIAL_HAVE_NODMA) || defined(SERIAL_HAVE_RXDMA_OPS)
#  define SERIAL_HAVE_INTTX 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32vw55x_uart_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t uartclk;   /* Peripheral kernel clock (PCLK1 or PCLK2) */
  uint32_t baud;      /* Configured baud */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint32_t im;        /* Shadow of the CTL0 interrupt enable bits */
#ifdef SERIAL_HAVE_RXDMA
  uint8_t  rxdmareq;  /* DMA request of the receiver, 0 without DMA */
  bool     rxenable;  /* Reception enabled by the serial driver */
  DMA_HANDLE rxdma;   /* DMA channel of the receiver */
  char    *rxfifo;    /* Circular DMA buffer */
  uint32_t rxdmanext; /* Next character to read in rxfifo */
#endif
#ifdef SERIAL_HAVE_TXDMA
  uint8_t  txdmareq;  /* DMA request of the transmitter, 0 without DMA */
  bool     txbusy;    /* A transmit DMA is in progress */
  DMA_HANDLE txdma;   /* DMA channel of the transmitter */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  gd32vw55x_setup(struct uart_dev_s *dev);
static void gd32vw55x_shutdown(struct uart_dev_s *dev);
static int  gd32vw55x_attach(struct uart_dev_s *dev);
static void gd32vw55x_detach(struct uart_dev_s *dev);
static int  gd32vw55x_interrupt(int irq, void *context, void *arg);
static int  gd32vw55x_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifdef SERIAL_HAVE_INTRX
static int  gd32vw55x_receive(struct uart_dev_s *dev, unsigned int *status);
static void gd32vw55x_rxint(struct uart_dev_s *dev, bool enable);
static bool gd32vw55x_rxavailable(struct uart_dev_s *dev);
#endif
static void gd32vw55x_send(struct uart_dev_s *dev, int ch);
#ifdef SERIAL_HAVE_INTTX
static void gd32vw55x_txint(struct uart_dev_s *dev, bool enable);
#endif
static bool gd32vw55x_txready(struct uart_dev_s *dev);
static bool gd32vw55x_txempty(struct uart_dev_s *dev);
#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int  gd32vw55x_dma_setup(struct uart_dev_s *dev);
static void gd32vw55x_dma_shutdown(struct uart_dev_s *dev);
#endif
#ifdef SERIAL_HAVE_RXDMA
static int  gd32vw55x_dma_receive(struct uart_dev_s *dev,
                                 unsigned int *status);
static void gd32vw55x_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool gd32vw55x_dma_rxavailable(struct uart_dev_s *dev);
#endif
#ifdef SERIAL_HAVE_TXDMA
static void gd32vw55x_dma_txcallback(DMA_HANDLE handle, uint16_t status,
                                    void *arg);
static void gd32vw55x_dma_send(struct uart_dev_s *dev);
static void gd32vw55x_dma_txint(struct uart_dev_s *dev, bool enable);
static void gd32vw55x_dma_txavailable(struct uart_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef SERIAL_HAVE_NODMA
static const struct uart_ops_s g_uart_ops =
{
  .setup       = gd32vw55x_setup,
  .shutdown    = gd32vw55x_shutdown,
  .attach      = gd32vw55x_attach,
  .detach      = gd32vw55x_detach,
  .ioctl       = gd32vw55x_ioctl,
  .receive     = gd32vw55x_receive,
  .rxint       = gd32vw55x_rxint,
  .rxavailable = gd32vw55x_rxavailable,
  .send        = gd32vw55x_send,
  .txint       = gd32vw55x_txint,
  .txready     = gd32vw55x_txready,
  .txempty     = gd32vw55x_txempty,
};
#endif

#ifdef SERIAL_HAVE_RXDMA_OPS
static const struct uart_ops_s g_uart_rxdma_ops =
{
  .setup       = gd32vw55x_dma_setup,
  .shutdown    = gd32vw55x_dma_shutdown,
  .attach      = gd32vw55x_attach,
  .detach      = gd32vw55x_detach,
  .ioctl       = gd32vw55x_ioctl,
  .receive     = gd32vw55x_dma_receive,
  .rxint       = gd32vw55x_dma_rxint,
  .rxavailable = gd32vw55x_dma_rxavailable,
  .send        = gd32vw55x_send,
  .txint       = gd32vw55x_txint,
  .txready     = gd32vw55x_txready,
  .txempty     = gd32vw55x_txempty,
};
#endif

#ifdef SERIAL_HAVE_TXDMA_OPS
static const struct uart_ops_s g_uart_txdma_ops =
{
  .setup       = gd32vw55x_dma_setup,
  .shutdown    = gd32vw55x_dma_shutdown,
  .attach      = gd32vw55x_attach,
  .detach      = gd32vw55x_detach,
  .ioctl       = gd32vw55x_ioctl,
  .receive     = gd32vw55x_receive,
  .rxint       = gd32vw55x_rxint,
  .rxavailable = gd32vw55x_rxavailable,
  .send        = gd32vw55x_send,
  .txint       = gd32vw55x_dma_txint,
  .txready     = gd32vw55x_txready,
  .txempty     = gd32vw55x_txempty,
  .dmasend     = gd32vw55x_dma_send,
  .dmatxavail  = gd32vw55x_dma_txavailable,
};
#endif

#ifdef SERIAL_HAVE_RXTXDMA_OPS
static const struct uart_ops_s g_uart_rxtxdma_ops =
{
  .setup       = gd32vw55x_dma_setup,
  .shutdown    = gd32vw55x_dma_shutdown,
  .attach      = gd32vw55x_attach,
  .detach      = gd32vw55x_detach,
  .ioctl       = gd32vw55x_ioctl,
  .receive     = gd32vw55x_dma_receive,
  .rxint       = gd32vw55x_dma_rxint,
  .rxavailable = gd32vw55x_dma_rxavailable,
  .send        = gd32vw55x_send,
  .txint       = gd32vw55x_dma_txint,
  .txready     = gd32vw55x_txready,
  .txempty     = gd32vw55x_txempty,
  .dmasend     = gd32vw55x_dma_send,
  .dmatxavail  = gd32vw55x_dma_txavailable,
};
#endif

#ifdef CONFIG_GD32VW55X_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#ifdef CONFIG_USART0_RXDMA
static char g_usart0rxfifo[RXDMA_BUFFER_SIZE];
#endif

static struct gd32vw55x_uart_s g_usart0priv =
{
  .uartbase = GD32VW55X_USART0_BASE,
  .uartclk  = GD32VW55X_PCLK1_FREQ,
  .baud     = CONFIG_USART0_BAUD,
  .irq      = GD32VW55X_IRQ_USART0,
#ifdef CONFIG_USART0_RXDMA
  .rxdmareq = DMA_REQ_USART0_RX,
  .rxfifo   = g_usart0rxfifo,
#endif
#ifdef CONFIG_USART0_TXDMA
  .txdmareq = DMA_REQ_USART0_TX,
#endif
};

static uart_dev_t g_usart0port =
{
  .recv =
  {
    .size    = CONFIG_USART0_RXBUFSIZE,
    .buffer  = g_usart0rxbuffer,
  },
  .xmit =
  {
    .size    = CONFIG_USART0_TXBUFSIZE,
    .buffer  = g_usart0txbuffer,
  },
  .ops  = &USART0_OPS,
  .priv = &g_usart0priv,
};
#endif

#ifdef CONFIG_GD32VW55X_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#ifdef CONFIG_UART1_RXDMA
static char g_uart1rxfifo[RXDMA_BUFFER_SIZE];
#endif

static struct gd32vw55x_uart_s g_uart1priv =
{
  .uartbase = GD32VW55X_UART1_BASE,
  .uartclk  = GD32VW55X_PCLK1_FREQ,
  .baud     = CONFIG_UART1_BAUD,
  .irq      = GD32VW55X_IRQ_UART1,
#ifdef CONFIG_UART1_RXDMA
  .rxdmareq = DMA_REQ_UART1_RX,
  .rxfifo   = g_uart1rxfifo,
#endif
#ifdef CONFIG_UART1_TXDMA
  .txdmareq = DMA_REQ_UART1_TX,
#endif
};

static uart_dev_t g_uart1port =
{
  .recv =
  {
    .size    = CONFIG_UART1_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit =
  {
    .size    = CONFIG_UART1_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
  },
  .ops  = &UART1_OPS,
  .priv = &g_uart1priv,
};
#endif

#ifdef CONFIG_GD32VW55X_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#ifdef CONFIG_UART2_RXDMA
static char g_uart2rxfifo[RXDMA_BUFFER_SIZE];
#endif

static struct gd32vw55x_uart_s g_uart2priv =
{
  .uartbase = GD32VW55X_UART2_BASE,
  .uartclk  = GD32VW55X_PCLK2_FREQ,
  .baud     = CONFIG_UART2_BAUD,
  .irq      = GD32VW55X_IRQ_UART2,
#ifdef CONFIG_UART2_RXDMA
  .rxdmareq = DMA_REQ_UART2_RX,
  .rxfifo   = g_uart2rxfifo,
#endif
#ifdef CONFIG_UART2_TXDMA
  .txdmareq = DMA_REQ_UART2_TX,
#endif
};

static uart_dev_t g_uart2port =
{
  .recv =
  {
    .size    = CONFIG_UART2_RXBUFSIZE,
    .buffer  = g_uart2rxbuffer,
  },
  .xmit =
  {
    .size    = CONFIG_UART2_TXBUFSIZE,
    .buffer  = g_uart2txbuffer,
  },
  .ops  = &UART2_OPS,
  .priv = &g_uart2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32vw55x_restoreint
 ****************************************************************************/

static void gd32vw55x_restoreint(struct gd32vw55x_uart_s *priv)
{
  modifyreg32(priv->uartbase + GD32VW55X_UART_CTL0_OFFSET,
              UART_CTL0_RBNEIE | UART_CTL0_TBEIE | UART_CTL0_IDLEIE,
              priv->im);
}

/****************************************************************************
 * Name: gd32vw55x_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int gd32vw55x_setup(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  /* Pin muxing and peripheral clock were done in gd32vw55x_lowsetup() */

  gd32vw55x_uart_configure(priv->uartbase, priv->uartclk, priv->baud);
  priv->im = 0;
  return OK;
}

/****************************************************************************
 * Name: gd32vw55x_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void gd32vw55x_shutdown(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  /* Disable interrupts */

  priv->im = 0;
  gd32vw55x_restoreint(priv);
}

/****************************************************************************
 * Name: gd32vw55x_attach
 ****************************************************************************/

static int gd32vw55x_attach(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->irq, gd32vw55x_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: gd32vw55x_detach
 ****************************************************************************/

static void gd32vw55x_detach(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: gd32vw55x_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.
 *
 ****************************************************************************/

static int gd32vw55x_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct gd32vw55x_uart_s *priv;
  uint32_t status;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct gd32vw55x_uart_s *)dev->priv;

  status = getreg32(priv->uartbase + GD32VW55X_UART_STAT_OFFSET);

  /* Clear any error conditions (overrun, framing, noise, parity) */

  if ((status & (UART_STAT_ORERR | UART_STAT_FERR |
                 UART_STAT_NERR | UART_STAT_PERR)) != 0)
    {
      putreg32(UART_INTC_OREC | UART_INTC_FEC |
               UART_INTC_NEC | UART_INTC_PEC,
               priv->uartbase + GD32VW55X_UART_INTC_OFFSET);
    }

  /* Handle received data */

  if ((status & UART_STAT_RBNE) != 0 &&
      (priv->im & UART_CTL0_RBNEIE) != 0)
    {
      uart_recvchars(dev);
    }

#ifdef SERIAL_HAVE_RXDMA
  /* The line went idle: hand what the DMA received to the serial driver */

  if ((status & UART_STAT_IDLEF) != 0 &&
      (priv->im & UART_CTL0_IDLEIE) != 0)
    {
      putreg32(UART_INTC_IDLEC, priv->uartbase + GD32VW55X_UART_INTC_OFFSET);

      if (priv->rxenable && gd32vw55x_dma_rxavailable(dev))
        {
          uart_recvchars(dev);
        }
    }
#endif

  /* Handle outgoing data */

  if ((status & UART_STAT_TBE) != 0 &&
      (priv->im & UART_CTL0_TBEIE) != 0)
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: gd32vw55x_ioctl
 ****************************************************************************/

static int gd32vw55x_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

#ifdef SERIAL_HAVE_INTRX
/****************************************************************************
 * Name: gd32vw55x_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int gd32vw55x_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  *status = getreg32(priv->uartbase + GD32VW55X_UART_STAT_OFFSET);
  return getreg32(priv->uartbase + GD32VW55X_UART_RDATA_OFFSET) & 0xff;
}

/****************************************************************************
 * Name: gd32vw55x_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void gd32vw55x_rxint(struct uart_dev_s *dev, bool enable)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_CTL0_RBNEIE;
#endif
    }
  else
    {
      priv->im &= ~UART_CTL0_RBNEIE;
    }

  gd32vw55x_restoreint(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: gd32vw55x_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool gd32vw55x_rxavailable(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  return (getreg32(priv->uartbase + GD32VW55X_UART_STAT_OFFSET) &
          UART_STAT_RBNE) != 0;
}

#endif /* SERIAL_HAVE_INTRX */

/****************************************************************************
 * Name: gd32vw55x_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void gd32vw55x_send(struct uart_dev_s *dev, int ch)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  putreg32((uint32_t)ch, priv->uartbase + GD32VW55X_UART_TDATA_OFFSET);
}

/****************************************************************************
 * Name: gd32vw55x_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_INTTX
static void gd32vw55x_txint(struct uart_dev_s *dev, bool enable)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_CTL0_TBEIE;
      gd32vw55x_restoreint(priv);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->im &= ~UART_CTL0_TBEIE;
      gd32vw55x_restoreint(priv);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: gd32vw55x_txready
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool gd32vw55x_txready(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  return (getreg32(priv->uartbase + GD32VW55X_UART_STAT_OFFSET) &
          UART_STAT_TBE) != 0;
}

/****************************************************************************
 * Name: gd32vw55x_txempty
 *
 * Description:
 *   Return true if all characters have been sent
 *
 ****************************************************************************/

static bool gd32vw55x_txempty(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  return (getreg32(priv->uartbase + GD32VW55X_UART_STAT_OFFSET) &
          UART_STAT_TC) != 0;
}

#ifdef SERIAL_HAVE_RXDMA
/****************************************************************************
 * Name: gd32vw55x_dma_nextrx
 *
 * Description:
 *   Return the index of the next character that the DMA will write.
 *
 ****************************************************************************/

static uint32_t gd32vw55x_dma_nextrx(struct gd32vw55x_uart_s *priv)
{
  return RXDMA_BUFFER_SIZE - gd32_dma_tansnum_get(priv->rxdma);
}

/****************************************************************************
 * Name: gd32vw55x_dma_rxcallback
 *
 * Description:
 *   The DMA filled half or all of the buffer: hand the characters to the
 *   serial driver before they are overwritten.
 *
 ****************************************************************************/

static void gd32vw55x_dma_rxcallback(DMA_HANDLE handle, uint16_t status,
                                    void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  if (priv->rxenable && gd32vw55x_dma_rxavailable(dev))
    {
      uart_recvchars(dev);
    }
}
#endif /* SERIAL_HAVE_RXDMA */

#ifdef SERIAL_HAVE_TXDMA
/****************************************************************************
 * Name: gd32vw55x_dma_txstart
 *
 * Description:
 *   Start the transmit DMA on a part of the transmit buffer.
 *
 ****************************************************************************/

static void gd32vw55x_dma_txstart(struct uart_dev_s *dev, const char *buffer,
                                 size_t length)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;
  struct gd32_dma_config_s cfg;

  cfg.periph_addr         = priv->uartbase + GD32VW55X_UART_TDATA_OFFSET;
  cfg.memory0_addr        = (uint32_t)buffer;
  cfg.number              = length;
  cfg.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
  cfg.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
  cfg.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
  cfg.circular_mode       = DMA_CIRCULAR_MODE_DISABLE;
  cfg.direction           = DMA_MEMORY_TO_PERIPH;
  cfg.priority            = DMA_PRIO_MEDIUM_SELECT;

  gd32_dma_stop(priv->txdma);
  gd32_dma_setup(priv->txdma, &cfg, GD32_DMA_SINGLE_DATA_MODE);
  gd32_dma_start(priv->txdma, gd32vw55x_dma_txcallback, dev,
                 DMA_CHXCTL_FTFIE | DMA_CHXCTL_TAEIE);
}

/****************************************************************************
 * Name: gd32vw55x_dma_txcallback
 *
 * Description:
 *   The DMA sent its part of the transmit buffer: send the second part of
 *   a buffer that wraps around, then free the space and send what was
 *   written meanwhile.
 *
 ****************************************************************************/

static void gd32vw55x_dma_txcallback(DMA_HANDLE handle, uint16_t status,
                                    void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  if ((status & DMA_INTF_FTFIF) != 0)
    {
      dev->dmatx.nbytes += dev->dmatx.length;

      if (dev->dmatx.nlength != 0)
        {
          dev->dmatx.length  = dev->dmatx.nlength;
          dev->dmatx.nlength = 0;
          gd32vw55x_dma_txstart(dev, dev->dmatx.nbuffer, dev->dmatx.length);
          return;
        }
    }

  priv->txbusy = false;
  uart_xmitchars_done(dev);
  uart_xmitchars_dma(dev);
}
#endif /* SERIAL_HAVE_TXDMA */

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
/****************************************************************************
 * Name: gd32vw55x_dma_setup
 *
 * Description:
 *   Configure the UART, then start the circular reception DMA and get the
 *   transmit DMA channel.
 *
 ****************************************************************************/

static int gd32vw55x_dma_setup(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;
#ifdef SERIAL_HAVE_RXDMA
  struct gd32_dma_config_s cfg;
#endif

  /* The console was set up at boot */

  if (!dev->isconsole)
    {
      gd32vw55x_setup(dev);
    }

#ifdef SERIAL_HAVE_TXDMA
  if (priv->txdmareq != 0 && priv->txdma == NULL)
    {
      priv->txdma = gd32_dma_channel_alloc(priv->txdmareq);
      if (priv->txdma == NULL)
        {
          return -EBUSY;
        }

      priv->txbusy = false;
      modifyreg32(priv->uartbase + GD32VW55X_UART_CTL2_OFFSET, 0,
                  UART_CTL2_DENT);
    }
#endif

#ifdef SERIAL_HAVE_RXDMA
  if (priv->rxdmareq == 0)
    {
      return OK;
    }

  if (priv->rxdma == NULL)
    {
      priv->rxdma = gd32_dma_channel_alloc(priv->rxdmareq);
      if (priv->rxdma == NULL)
        {
          return -EBUSY;
        }
    }

  cfg.periph_addr         = priv->uartbase + GD32VW55X_UART_RDATA_OFFSET;
  cfg.memory0_addr        = (uint32_t)priv->rxfifo;
  cfg.number              = RXDMA_BUFFER_SIZE;
  cfg.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
  cfg.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
  cfg.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
  cfg.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
  cfg.direction           = DMA_PERIPH_TO_MEMORY;
  cfg.priority            = DMA_PRIO_HIGH_SELECT;

  gd32_dma_setup(priv->rxdma, &cfg, GD32_DMA_SINGLE_DATA_MODE);
  priv->rxdmanext = 0;

  modifyreg32(priv->uartbase + GD32VW55X_UART_CTL2_OFFSET, 0,
              UART_CTL2_DENR);

  gd32_dma_start(priv->rxdma, gd32vw55x_dma_rxcallback, dev,
                 DMA_CHXCTL_HTFIE | DMA_CHXCTL_FTFIE);
#endif

  return OK;
}

/****************************************************************************
 * Name: gd32vw55x_dma_shutdown
 ****************************************************************************/

static void gd32vw55x_dma_shutdown(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  gd32vw55x_shutdown(dev);

  modifyreg32(priv->uartbase + GD32VW55X_UART_CTL2_OFFSET,
              UART_CTL2_DENR | UART_CTL2_DENT, 0);

#ifdef SERIAL_HAVE_RXDMA
  if (priv->rxdma != NULL)
    {
      gd32_dma_stop(priv->rxdma);
      gd32_dma_channel_free(priv->rxdma);
      priv->rxdma = NULL;
    }
#endif

#ifdef SERIAL_HAVE_TXDMA
  if (priv->txdma != NULL)
    {
      gd32_dma_stop(priv->txdma);
      gd32_dma_channel_free(priv->txdma);
      priv->txdma  = NULL;
      priv->txbusy = false;
    }
#endif
}
#endif /* SERIAL_HAVE_RXDMA || SERIAL_HAVE_TXDMA */

#ifdef SERIAL_HAVE_RXDMA

/****************************************************************************
 * Name: gd32vw55x_dma_receive
 *
 * Description:
 *   Return the next character of the DMA buffer.
 *
 ****************************************************************************/

static int gd32vw55x_dma_receive(struct uart_dev_s *dev,
                                unsigned int *status)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;
  int c = 0;

  *status = 0;

  if (gd32vw55x_dma_nextrx(priv) != priv->rxdmanext)
    {
      c = priv->rxfifo[priv->rxdmanext];

      if (++priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
          priv->rxdmanext = 0;
        }
    }

  return c;
}

/****************************************************************************
 * Name: gd32vw55x_dma_rxint
 *
 * Description:
 *   Enable or disable the delivery of the received characters.  The DMA
 *   keeps receiving; the characters are delivered on the idle line
 *   interrupt and the DMA half and full transfer interrupts.
 *
 ****************************************************************************/

static void gd32vw55x_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->rxenable = enable;

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_CTL0_IDLEIE;
#endif
    }
  else
    {
      priv->im &= ~UART_CTL0_IDLEIE;
    }

  gd32vw55x_restoreint(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: gd32vw55x_dma_rxavailable
 *
 * Description:
 *   Return true if the DMA buffer holds characters not read yet.
 *
 ****************************************************************************/

static bool gd32vw55x_dma_rxavailable(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  return gd32vw55x_dma_nextrx(priv) != priv->rxdmanext;
}
#endif /* SERIAL_HAVE_RXDMA */

#ifdef SERIAL_HAVE_TXDMA
/****************************************************************************
 * Name: gd32vw55x_dma_send
 *
 * Description:
 *   Start sending the part of the transmit buffer set up by the serial
 *   driver in dev->dmatx.
 *
 ****************************************************************************/

static void gd32vw55x_dma_send(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;

  priv->txbusy      = true;
  dev->dmatx.nbytes = 0;
  gd32vw55x_dma_txstart(dev, dev->dmatx.buffer, dev->dmatx.length);
}

/****************************************************************************
 * Name: gd32vw55x_dma_txint
 *
 * Description:
 *   The transmitter does not use the UART interrupts: the DMA interrupt
 *   ends each transfer.
 *
 ****************************************************************************/

static void gd32vw55x_dma_txint(struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: gd32vw55x_dma_txavailable
 *
 * Description:
 *   The transmit buffer holds new characters: send them unless a transfer
 *   is in progress, which sends them when it ends.
 *
 ****************************************************************************/

static void gd32vw55x_dma_txavailable(struct uart_dev_s *dev)
{
  struct gd32vw55x_uart_s *priv = (struct gd32vw55x_uart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (priv->txdma != NULL && !priv->txbusy)
    {
      uart_xmitchars_dma(dev);
    }

  leave_critical_section(flags);
}
#endif /* SERIAL_HAVE_TXDMA */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in gd32vw55x_lowsetup() and main clock
 *   initialization performed in gd32vw55x_clockconfig().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void riscv_earlyserialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  gd32vw55x_setup(&CONSOLE_DEV);
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
#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
  /* The console is not set up again when it is opened: start its DMA */

  if (CONSOLE_DEV.ops->setup == gd32vw55x_dma_setup)
    {
      gd32vw55x_dma_setup(&CONSOLE_DEV);
    }
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
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  riscv_lowputc(ch);
#endif
}
