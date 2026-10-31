/****************************************************************************
 * arch/rh850/src/u2c/u2c_serial.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>

#include "rh850_internal.h"
#include "hardware/u2c_rlin3.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_RH850_RLIN311) && defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define HAVE_SERIAL_CONSOLE 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct u2c_uart_s
{
  uintptr_t base;     /* RLIN3 unit base address */
  uint32_t  baud;     /* Baud rate */
  int       txirq;    /* UR0, one request per transmitted byte */
  int       rxirq;    /* UR1, reception complete */
  bool      txbusy;   /* A byte was written, its UR0 not taken yet */
  bool      rxpend;   /* A received byte waits in LURDR */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  u2c_setup(FAR struct uart_dev_s *dev);
static void u2c_shutdown(FAR struct uart_dev_s *dev);
static int  u2c_attach(FAR struct uart_dev_s *dev);
static void u2c_detach(FAR struct uart_dev_s *dev);
static int  u2c_tx_interrupt(int irq, FAR void *context, FAR void *arg);
static int  u2c_rx_interrupt(int irq, FAR void *context, FAR void *arg);
static int  u2c_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  u2c_receive(FAR struct uart_dev_s *dev,
                        FAR unsigned int *status);
static void u2c_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool u2c_rxavailable(FAR struct uart_dev_s *dev);
static void u2c_send(FAR struct uart_dev_s *dev, int ch);
static void u2c_txint(FAR struct uart_dev_s *dev, bool enable);
static bool u2c_txready(FAR struct uart_dev_s *dev);
static bool u2c_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_u2c_uart_ops =
{
  .setup          = u2c_setup,
  .shutdown       = u2c_shutdown,
  .attach         = u2c_attach,
  .detach         = u2c_detach,
  .ioctl          = u2c_ioctl,
  .receive        = u2c_receive,
  .rxint          = u2c_rxint,
  .rxavailable    = u2c_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = u2c_send,
  .txint          = u2c_txint,
  .txready        = u2c_txready,
  .txempty        = u2c_txempty,
};

#ifdef CONFIG_RH850_RLIN311
static char g_rlin311rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_rlin311txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct u2c_uart_s g_rlin311priv =
{
  .base  = U2C_RLIN311_BASE,
  .baud  = CONFIG_UART0_BAUD,
  .txirq = RH850_IRQ_RLIN311_TX,
  .rxirq = RH850_IRQ_RLIN311_RX,
};

static uart_dev_t g_rlin311port =
{
#ifdef HAVE_SERIAL_CONSOLE
  .isconsole = true,
#endif
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_rlin311rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_rlin311txbuffer,
  },
  .ops       = &g_u2c_uart_ops,
  .priv      = &g_rlin311priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int u2c_setup(FAR struct uart_dev_s *dev)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  /* The console is set up by rh850_lowsetup() */

  if (!dev->isconsole)
    {
      u2c_rlin3_setup(priv->base, priv->baud);
    }

  return OK;
}

static void u2c_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  up_disable_irq(priv->txirq);
  up_disable_irq(priv->rxirq);
}

static int u2c_attach(FAR struct uart_dev_s *dev)
{
  FAR struct u2c_uart_s *priv = dev->priv;
  int ret;

  /* up_putc() output before this point left a UR0 request */

  u2c_irq_pending(priv->txirq);
  ret = irq_attach(priv->txirq, u2c_tx_interrupt, dev);
  if (ret == OK)
    {
      ret = irq_attach(priv->rxirq, u2c_rx_interrupt, dev);
    }

  return ret;
}

static void u2c_detach(FAR struct uart_dev_s *dev)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  up_disable_irq(priv->txirq);
  up_disable_irq(priv->rxirq);
  irq_detach(priv->txirq);
  irq_detach(priv->rxirq);
}

/* UR0 for the last byte written: the next byte may go */

static int u2c_tx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = arg;
  FAR struct u2c_uart_s *priv = dev->priv;

  priv->txbusy = false;
  uart_xmitchars(dev);
  return OK;
}

/* Accepting the edge request cleared EIRF: the byte is noted in rxpend */

static int u2c_rx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = arg;
  FAR struct u2c_uart_s *priv = dev->priv;

  priv->rxpend = true;
  uart_recvchars(dev);
  return OK;
}

static int u2c_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

static int u2c_receive(FAR struct uart_dev_s *dev, FAR unsigned int *status)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  priv->rxpend = false;
  *status = 0;
  return getreg16(priv->base + RLIN3_LURDR_OFFSET) & 0xff;
}

static void u2c_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  if (enable)
    {
      up_enable_irq(priv->rxirq);
    }
  else
    {
      up_disable_irq(priv->rxirq);
    }
}

/* A byte that arrived after the request was accepted sets EIRF again */

static bool u2c_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  if (u2c_irq_pending(priv->rxirq))
    {
      priv->rxpend = true;
    }

  return priv->rxpend;
}

static void u2c_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  priv->txbusy = true;
  putreg8((uint8_t)ch, priv->base + RLIN3_LUTDR_OFFSET);
}

/* One byte per UR0 request.  With no byte in flight no request will
 * come, so enabling starts the first byte.
 */

static void u2c_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct u2c_uart_s *priv = dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      up_enable_irq(priv->txirq);
      if (!priv->txbusy)
        {
          uart_xmitchars(dev);
        }
    }
  else
    {
      up_disable_irq(priv->txirq);
    }

  leave_critical_section(flags);
}

static bool u2c_txready(FAR struct uart_dev_s *dev)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  return !priv->txbusy &&
         (getreg8(priv->base + RLIN3_LST_OFFSET) & RLIN3_LST_UTS) == 0;
}

static bool u2c_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct u2c_uart_s *priv = dev->priv;

  return (getreg8(priv->base + RLIN3_LST_OFFSET) & RLIN3_LST_UTS) == 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rh850_serialinit
 ****************************************************************************/

void rh850_serialinit(void)
{
#ifdef CONFIG_RH850_RLIN311
#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &g_rlin311port);
#endif
  uart_register("/dev/ttyS0", &g_rlin311port);
#endif
}

#endif /* USE_SERIALDRIVER */
