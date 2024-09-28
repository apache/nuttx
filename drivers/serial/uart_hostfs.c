/****************************************************************************
 * drivers/serial/uart_hostfs.c
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

#include <nuttx/clock.h>
#include <nuttx/fs/hostfs.h>
#include <nuttx/serial/serial.h>
#include <nuttx/wdog.h>

#include <debug.h>
#include <fcntl.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_WDOG_DELAY  USEC2TICK(CONFIG_UART_HOSTFS_DELAY_USEC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_hostfs_priv_s
{
  int             fd;
  bool            rxint;
  struct wdog_s   wdog;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  uart_hostfs_setup(FAR struct uart_dev_s *dev);
static void uart_hostfs_shutdown(FAR struct uart_dev_s *dev);
static int  uart_hostfs_attach(FAR struct uart_dev_s *dev);
static void uart_hostfs_detach(FAR struct uart_dev_s *dev);
static int  uart_hostfs_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);
static void uart_hostfs_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_hostfs_rxavailable(FAR struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool uart_hostfs_rxflowcontrol(FAR struct uart_dev_s *dev,
                                      unsigned int nbuffered, bool upper);
#endif
static void uart_hostfs_dmatxavail(FAR struct uart_dev_s *dev);
static void uart_hostfs_dmasend(FAR struct uart_dev_s *dev);
static void uart_hostfs_dmarxfree(FAR struct uart_dev_s *dev);
static void uart_hostfs_dmareceive(FAR struct uart_dev_s *dev);
static void uart_hostfs_send(FAR struct uart_dev_s *dev, int ch);
static void uart_hostfs_txint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_hostfs_txready(FAR struct uart_dev_s *dev);
static bool uart_hostfs_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_hostfs_ops =
{
  .setup          = uart_hostfs_setup,
  .shutdown       = uart_hostfs_shutdown,
  .attach         = uart_hostfs_attach,
  .detach         = uart_hostfs_detach,
  .ioctl          = uart_hostfs_ioctl,
  .rxint          = uart_hostfs_rxint,
  .rxavailable    = uart_hostfs_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = uart_hostfs_rxflowcontrol,
#endif
  .dmatxavail     = uart_hostfs_dmatxavail,
  .dmasend        = uart_hostfs_dmasend,
  .dmarxfree      = uart_hostfs_dmarxfree,
  .dmareceive     = uart_hostfs_dmareceive,
  .send           = uart_hostfs_send,
  .txint          = uart_hostfs_txint,
  .txready        = uart_hostfs_txready,
  .txempty        = uart_hostfs_txempty,
};

static char g_uart_hostfs_rxbuf[CONFIG_UART_HOSTFS_RXBUFSIZE];
static char g_uart_hostfs_txbuf[CONFIG_UART_HOSTFS_TXBUFSIZE];

static struct uart_hostfs_priv_s g_uart_hostfs_priv;

static struct uart_dev_s g_uart_hostfs_dev =
{
  .ops            = &g_uart_hostfs_ops,
  .priv           = &g_uart_hostfs_priv,
  .xmit =
  {
    .size         = CONFIG_UART_HOSTFS_TXBUFSIZE,
    .buffer       = g_uart_hostfs_txbuf,
  },
  .recv =
  {
    .size         = CONFIG_UART_HOSTFS_RXBUFSIZE,
    .buffer       = g_uart_hostfs_rxbuf,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int uart_hostfs_setup(FAR struct uart_dev_s *dev)
{
  FAR struct uart_hostfs_priv_s *priv = dev->priv;

  priv->fd = host_open(CONFIG_UART_HOSTFS_DEVPATH, O_RDWR | O_NONBLOCK,
                       0666);
  return priv->fd;
}

static void uart_hostfs_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct uart_hostfs_priv_s *priv = dev->priv;

  if (priv->fd > 0)
    {
      host_close(priv->fd);
      priv->fd = -1;
    }
}

static int  uart_hostfs_attach(FAR struct uart_dev_s *dev)
{
  return OK;
}

static void uart_hostfs_detach(FAR struct uart_dev_s *dev)
{
}

static int  uart_hostfs_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg)
{
  return -ENOTTY;
}

static void uart_hostfs_wdog(wdparm_t arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;
  FAR struct uart_hostfs_priv_s *priv = dev->priv;

  if (priv->rxint)
    {
      uart_dmarxfree(dev);
    }

  wd_start(&priv->wdog, UART_WDOG_DELAY, uart_hostfs_wdog, (wdparm_t)dev);
}

static void uart_hostfs_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct uart_hostfs_priv_s *priv = dev->priv;

  priv->rxint = enable;
  if (enable)
    {
      wd_start(&priv->wdog, 0, uart_hostfs_wdog, (wdparm_t)dev);
    }
  else
    {
      wd_cancel(&priv->wdog);
    }
}

static bool uart_hostfs_rxavailable(FAR struct uart_dev_s *dev)
{
  return true;
}

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool uart_hostfs_rxflowcontrol(FAR struct uart_dev_s *dev,
                                      unsigned int nbuffered, bool upper)
{
  FAR struct uart_buffer_s *rxbuf = &dev->recv;

  return nbuffered == rxbuf->size;
}
#endif

static void uart_hostfs_dmatxavail(FAR struct uart_dev_s *dev)
{
  uart_xmitchars_dma(dev);
}

static void uart_hostfs_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct uart_hostfs_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  int ret;

  xfer->nbytes = 0;
  ret = host_write(priv->fd, xfer->buffer, xfer->length);
  if (ret > 0)
    {
      xfer->nbytes = ret;

      if (ret == xfer->length && xfer->nlength > 0)
        {
          ret = host_write(priv->fd, xfer->nbuffer, xfer->nlength);
          if (ret > 0)
            {
              xfer->nbytes += ret;
            }
        }
    }

  uart_xmitchars_done(dev);
}

static void uart_hostfs_dmarxfree(FAR struct uart_dev_s *dev)
{
  uart_recvchars_dma(dev);
}

static void uart_hostfs_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct uart_hostfs_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  ssize_t ret;

  xfer->nbytes = 0;
  ret = host_read(priv->fd, xfer->buffer, xfer->length);
  if (ret > 0)
    {
      xfer->nbytes = ret;

      if (ret == xfer->length && xfer->nlength > 0)
        {
          ret = host_read(priv->fd, xfer->nbuffer, xfer->nlength);
          if (ret > 0)
            {
              xfer->nbytes += ret;
            }
        }
    }

  uart_recvchars_done(dev);
}

static void uart_hostfs_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct uart_hostfs_priv_s *priv = dev->priv;
  char c = ch;

  host_write(priv->fd, &c, 1);
}

static void uart_hostfs_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

static bool uart_hostfs_txready(FAR struct uart_dev_s *dev)
{
  return true;
}

static bool uart_hostfs_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uart_hostfs_init(void)
{
  uart_register("/dev/console", &g_uart_hostfs_dev);
}

int up_putc(int ch)
{
  FAR struct uart_dev_s *dev = &g_uart_hostfs_dev;

  uart_hostfs_send(dev, ch);
  return ch;
}
