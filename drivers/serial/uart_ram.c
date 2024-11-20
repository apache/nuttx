/****************************************************************************
 * drivers/serial/uart_ram.c
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

#include <assert.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/kmalloc.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/uart_ram.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_RAMBUF_SECTION(n) \
        locate_data(CONFIG_RAM_UART_BUFFER_SECTION "." #n)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_ram_s
{
  struct uart_dev_s uart;
  struct wdog_s wdog;
  FAR struct uart_rambuf_s *tx;
  FAR struct uart_rambuf_s *rx;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int uart_ram_setup(FAR struct uart_dev_s *dev);
static void uart_ram_shutdown(FAR struct uart_dev_s *dev);
static int uart_ram_attach(FAR struct uart_dev_s *dev);
static void uart_ram_detach(FAR struct uart_dev_s *dev);
static int uart_ram_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int uart_ram_receive(FAR struct uart_dev_s *dev,
                            FAR uint32_t *status);
static void uart_ram_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_ram_rxavailable(FAR struct uart_dev_s *dev);
static void uart_ram_dmasend(FAR struct uart_dev_s *dev);
static void uart_ram_dmareceive(FAR struct uart_dev_s *dev);
static void uart_ram_dmarxfree(FAR struct uart_dev_s *dev);
static void uart_ram_dmatxavail(FAR struct uart_dev_s *dev);
static void uart_ram_send(FAR struct uart_dev_s *dev, int ch);
static void uart_ram_txint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_ram_txready(FAR struct uart_dev_s *dev);
static bool uart_ram_txempty(FAR struct uart_dev_s *dev);

static void uart_ram_wdog(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ram_ops =
{
  uart_ram_setup,
  uart_ram_shutdown,
  uart_ram_attach,
  uart_ram_detach,
  uart_ram_ioctl,
  uart_ram_receive,
  uart_ram_rxint,
  uart_ram_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  NULL,
#endif
  uart_ram_dmasend,
  uart_ram_dmareceive,
  uart_ram_dmarxfree,
  uart_ram_dmatxavail,
  uart_ram_send,
  uart_ram_txint,
  uart_ram_txready,
  uart_ram_txempty,
};

#ifdef CONFIG_RAM_UART0
static struct uart_rambuf_s UART_RAMBUF_SECTION(0) g_uart_rambuf0[2];

static struct uart_ram_s g_uart_ram0 =
{
  .uart =
  {
    .xmit =
    {
#  ifdef CONFIG_RAM_UART0_SLAVE
      .buffer = g_uart_rambuf0[1].buffer,
#  else
      .buffer = g_uart_rambuf0[0].buffer,
#  endif
      .size = CONFIG_RAM_UART_BUFSIZE,
    },
    .recv =
    {
#  ifdef CONFIG_RAM_UART0_SLAVE
      .buffer = g_uart_rambuf0[0].buffer,
#  else
      .buffer = g_uart_rambuf0[1].buffer,
#  endif
      .size = CONFIG_RAM_UART_BUFSIZE,
    },
    .ops = &g_uart_ram_ops,
    .priv = &g_uart_ram0,
  },
#  ifdef CONFIG_RAM_UART0_SLAVE
  .tx = &g_uart_rambuf0[1],
  .rx = &g_uart_rambuf0[0],
#  else
  .tx = &g_uart_rambuf0[0],
  .rx = &g_uart_rambuf0[1],
#  endif
};
#endif

#ifdef CONFIG_RAM_UART1
static struct uart_rambuf_s UART_RAMBUF_SECTION(1) g_uart_rambuf1[2];

static struct uart_ram_s g_uart_ram1 =
{
  .uart =
  {
    .xmit =
    {
#  ifdef CONFIG_RAM_UART1_SLAVE
      .buffer = g_uart_rambuf1[1].buffer,
#  else
      .buffer = g_uart_rambuf1[0].buffer,
#  endif
      .size = CONFIG_RAM_UART_BUFSIZE,
    },
    .recv =
    {
#  ifdef CONFIG_RAM_UART1_SLAVE
      .buffer = g_uart_rambuf1[0].buffer,
#  else
      .buffer = g_uart_rambuf1[1].buffer,
#  endif
      .size = CONFIG_RAM_UART_BUFSIZE,
    },
    .ops = &g_uart_ram_ops,
    .priv = &g_uart_ram1,
  },
#  ifdef CONFIG_RAM_UART1_SLAVE
  .tx = &g_uart_rambuf1[1],
  .rx = &g_uart_rambuf1[0],
#  else
  .tx = &g_uart_rambuf1[0],
  .rx = &g_uart_rambuf1[1],
#  endif
};
#endif

#ifdef CONFIG_RAM_UART2
static struct uart_rambuf_s UART_RAMBUF_SECTION(2) g_uart_rambuf2[2];

static struct uart_ram_s g_uart_ram2 =
{
  .uart =
  {
    .xmit =
    {
#  ifdef CONFIG_RAM_UART2_SLAVE
      .buffer = g_uart_rambuf2[1].buffer,
#  else
      .buffer = g_uart_rambuf2[0].buffer,
#  endif
      .size = CONFIG_RAM_UART_BUFSIZE,
    },
    .recv =
    {
#  ifdef CONFIG_RAM_UART2_SLAVE
      .buffer = g_uart_rambuf2[0].buffer,
#  else
      .buffer = g_uart_rambuf2[1].buffer,
#  endif
      .size = CONFIG_RAM_UART_BUFSIZE,
    },
    .ops = &g_uart_ram_ops,
    .priv = &g_uart_ram2,
  },
#  ifdef CONFIG_RAM_UART2_SLAVE
  .tx = &g_uart_rambuf2[1],
  .rx = &g_uart_rambuf2[0],
#  else
  .tx = &g_uart_rambuf2[0],
  .rx = &g_uart_rambuf2[1],
#  endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_rambuf_txready
 ****************************************************************************/

static size_t uart_rambuf_txready(FAR struct uart_rambuf_s *buf)
{
  int wroff = atomic_read(&buf->wroff);
  int rdoff = atomic_read(&buf->rdoff);
  return rdoff > wroff ? rdoff - wroff - 1 :
         sizeof(buf->buffer) - wroff + rdoff - 1;
}

/****************************************************************************
 * Name: uart_rambuf_rxavailable
 ****************************************************************************/

static size_t uart_rambuf_rxavailable(FAR struct uart_rambuf_s *buf)
{
  int wroff = atomic_read(&buf->wroff);
  int rdoff = atomic_read(&buf->rdoff);

  return wroff >= rdoff ? wroff - rdoff :
         sizeof(buf->buffer) - rdoff + wroff;
}

/****************************************************************************
 * Name: uart_ram_setup
 ****************************************************************************/

static int uart_ram_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: uart_ram_shutdown
 ****************************************************************************/

static void uart_ram_shutdown(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: uart_ram_attach
 ****************************************************************************/

static int uart_ram_attach(FAR struct uart_dev_s *dev)
{
  FAR struct uart_ram_s *priv = dev->priv;
  wd_start(&priv->wdog, USEC2TICK(CONFIG_RAM_UART_POLLING_INTERVAL),
           uart_ram_wdog, (wdparm_t)dev);
  return OK;
}

/****************************************************************************
 * Name: uart_ram_detach
 ****************************************************************************/

static void uart_ram_detach(FAR struct uart_dev_s *dev)
{
  FAR struct uart_ram_s *priv = dev->priv;
  wd_cancel(&priv->wdog);
}

/****************************************************************************
 * Name: uart_ram_ioctl
 ****************************************************************************/

static int uart_ram_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: uart_ram_receive
 ****************************************************************************/

static int uart_ram_receive(FAR struct uart_dev_s *dev, FAR uint32_t *status)
{
  FAR struct uart_ram_s *priv = dev->priv;
  int rdoff;
  int ch;

  while (!uart_rambuf_rxavailable(priv->rx));

  rdoff = atomic_read(&priv->rx->rdoff);
  ch = priv->rx->buffer[rdoff];
  if (++rdoff >= sizeof(priv->rx->buffer))
    {
      rdoff = 0;
    }

  atomic_set(&priv->rx->rdoff, rdoff);

  *status = 0;
  return ch;
}

/****************************************************************************
 * Name: uart_ram_rxint
 ****************************************************************************/

static void uart_ram_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: uart_ram_rxavailable
 ****************************************************************************/

static bool uart_ram_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct uart_ram_s *priv = dev->priv;
  return uart_rambuf_rxavailable(priv->rx) != 0;
}

/****************************************************************************
 * Name: uart_ram_dmasend
 ****************************************************************************/

static void uart_ram_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct uart_ram_s *priv = dev->priv;

  atomic_set(&priv->tx->wroff, dev->xmit.head);
}

/****************************************************************************
 * Name: uart_ram_dmareceive
 ****************************************************************************/

static void uart_ram_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct uart_ram_s *priv = dev->priv;
  dev->dmarx.nbytes = uart_rambuf_rxavailable(priv->rx);
  uart_recvchars_done(dev);
}

/****************************************************************************
 * Name: uart_ram_dmarxfree
 ****************************************************************************/

static void uart_ram_dmarxfree(FAR struct uart_dev_s *dev)
{
  FAR struct uart_ram_s *priv = dev->priv;

  /* When the dma RX buffer is free, update the read data position */

  atomic_set(&priv->rx->rdoff, dev->recv.tail);
}

/****************************************************************************
 * Name: uart_ram_dmatxavail
 ****************************************************************************/

static void uart_ram_dmatxavail(FAR struct uart_dev_s *dev)
{
  if (dev->dmatx.length == 0)
    {
      uart_xmitchars_dma(dev);
    }
}

/****************************************************************************
 * Name: uart_ram_send
 ****************************************************************************/

static void uart_ram_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct uart_ram_s *priv = dev->priv;
  int wroff;

  while (!uart_rambuf_txready(priv->tx));

  wroff = atomic_read(&priv->tx->wroff);
  priv->tx->buffer[wroff] = ch;
  if (++wroff >= sizeof(priv->tx->buffer))
    {
      wroff = 0;
    }

  atomic_set(&priv->tx->wroff, wroff);
}

/****************************************************************************
 * Name: uart_ram_txint
 ****************************************************************************/

static void uart_ram_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: uart_ram_txready
 ****************************************************************************/

static bool uart_ram_txready(FAR struct uart_dev_s *dev)
{
  FAR struct uart_ram_s *priv = dev->priv;
  return uart_rambuf_txready(priv->tx) != 0;
}

/****************************************************************************
 * Name: uart_ram_txempty
 ****************************************************************************/

static bool uart_ram_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct uart_ram_s *priv = dev->priv;
  return uart_rambuf_rxavailable(priv->tx) == 0;
}

/****************************************************************************
 * Name: uart_ram_wdog
 ****************************************************************************/

static void uart_ram_wdog(wdparm_t arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;
  FAR struct uart_ram_s *priv = dev->priv;

  /* When the read and write pointers of the tx buffer are same,
   * it means that the data transmission is completed
   */

  if (dev->dmatx.length > 0 && !uart_rambuf_rxavailable(priv->tx))
    {
      dev->dmatx.nbytes = dev->dmatx.length + dev->dmatx.nlength;
      uart_xmitchars_done(dev);
    }

  /* When the read and write pointers of the rx buffer are different,
   * it means that the data is received
   */

  if (dev->dmarx.length == 0 && uart_rambuf_rxavailable(priv->rx))
    {
      uart_recvchars_dma(dev);
    }

  wd_start(&priv->wdog, USEC2TICK(CONFIG_RAM_UART_POLLING_INTERVAL),
           uart_ram_wdog, (wdparm_t)dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_ram_register
 ****************************************************************************/

int uart_ram_register(FAR const char *devname,
                      struct uart_rambuf_s rambuf[2],
                      bool slave)
{
  FAR struct uart_ram_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct uart_ram_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  atomic_set(&rambuf[0].wroff, 0);
  atomic_set(&rambuf[0].rdoff, 0);
  atomic_set(&rambuf[1].wroff, 0);
  atomic_set(&rambuf[1].rdoff, 0);

  if (slave)
    {
      priv->tx = rambuf + 1;
      priv->rx = rambuf;
    }
  else
    {
      priv->tx = rambuf;
      priv->rx = rambuf + 1;
    }

  priv->uart.priv = priv;
  priv->uart.ops = &g_uart_ram_ops;
  priv->uart.xmit.buffer = priv->tx->buffer;
  priv->uart.recv.buffer = priv->rx->buffer;
  priv->uart.xmit.size = CONFIG_RAM_UART_BUFSIZE;
  priv->uart.recv.size = CONFIG_RAM_UART_BUFSIZE;

  ret = uart_register(devname, &priv->uart);
  if (ret < 0)
    {
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: ram_serialinit
 ****************************************************************************/

void ram_serialinit(void)
{
#ifdef CONFIG_RAM_UART0
  uart_register("/dev/tty0", &g_uart_ram0.uart);
#endif

#ifdef CONFIG_RAM_UART1
  uart_register("/dev/tty1", &g_uart_ram1.uart);
#endif

#ifdef CONFIG_RAM_UART2
  uart_register("/dev/tty2", &g_uart_ram2.uart);
#endif
}
