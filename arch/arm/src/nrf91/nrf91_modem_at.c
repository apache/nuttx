/****************************************************************************
 * arch/arm/src/nrf91/nrf91_modem_at.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/serial/serial.h>

#include <debug.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string.h>

#include "nrf91_modem_at.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF91_MODEM_AT_RX 255
#define NRF91_MODEM_AT_TX 255

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf91_modem_at_s
{
  /* Norificaiton */

  bool        notif_now;
  const char *notif;
  size_t      notif_len;
  size_t      notif_i;

  /* Response */

  const char *resp;
  bool        resp_now;
  size_t      resp_len;
  size_t      resp_i;

  /* TX */

  char   txbuf[NRF91_MODEM_AT_TX];
  size_t tx_i;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf91_modem_at_setup(struct uart_dev_s *dev);
static void nrf91_modem_at_shutdown(struct uart_dev_s *dev);
static int nrf91_modem_at_attach(struct uart_dev_s *dev);
static void nrf91_modem_at_detach(struct uart_dev_s *dev);
static int nrf91_modem_at_ioctl(struct file *filep, int cmd,
                                unsigned long arg);
static int nrf91_modem_at_receive(struct uart_dev_s *dev,
                                  unsigned int *status);
static void nrf91_modem_at_rxint(struct uart_dev_s *dev, bool enable);
static bool nrf91_modem_at_rxavailable(struct uart_dev_s *dev);
static void nrf91_modem_at_send(struct uart_dev_s *dev, int ch);
static void nrf91_modem_at_txint(struct uart_dev_s *dev, bool enable);
static bool nrf91_modem_at_txready(struct uart_dev_s *dev);
static bool nrf91_modem_at_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_nrf91_modem_at_fops =
{
  .setup          = nrf91_modem_at_setup,
  .shutdown       = nrf91_modem_at_shutdown,
  .attach         = nrf91_modem_at_attach,
  .detach         = nrf91_modem_at_detach,
  .ioctl          = nrf91_modem_at_ioctl,
  .receive        = nrf91_modem_at_receive,
  .rxint          = nrf91_modem_at_rxint,
  .rxavailable    = nrf91_modem_at_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = nrf91_modem_at_send,
  .txint          = nrf91_modem_at_txint,
  .txready        = nrf91_modem_at_txready,
  .txempty        = nrf91_modem_at_txempty,
};

static struct nrf91_modem_at_s g_nrf91_modem_at_priv;

static char g_modem1rxbuffer[NRF91_MODEM_AT_RX];
static char g_modem1txbuffer[NRF91_MODEM_AT_TX];

static uart_dev_t g_nrf91_modem_at =
{
  .isconsole = false,
  .ops       = &g_nrf91_modem_at_fops,
  .priv      = &g_nrf91_modem_at_priv,
  .recv      =
  {
    .size    = NRF91_MODEM_AT_RX,
    .buffer  = g_modem1rxbuffer,
  },
  .xmit      =
  {
    .size    = NRF91_MODEM_AT_TX,
    .buffer  = g_modem1txbuffer,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_modem_at_notify_handler
 ****************************************************************************/

static void nrf91_modem_at_notify_handler(const char *notif)
{
  struct uart_dev_s       *dev  = &g_nrf91_modem_at;
  struct nrf91_modem_at_s *priv = (struct nrf91_modem_at_s *)dev->priv;

  priv->notif_i      = 0;
  priv->notif        = notif;
  priv->notif_len    = strlen(notif);

  uart_recvchars(dev);
}

/****************************************************************************
 * Name: nrf91_modem_at_resp_handler
 ****************************************************************************/

static void nrf91_modem_at_resp_handler(const char *resp)
{
  struct uart_dev_s       *dev  = &g_nrf91_modem_at;
  struct nrf91_modem_at_s *priv = (struct nrf91_modem_at_s *)dev->priv;

  priv->resp_i       = 0;
  priv->resp         = resp;
  priv->resp_len     = strlen(resp);

  uart_recvchars(dev);
}

/****************************************************************************
 * Name: nrf91_modem_at_setup
 ****************************************************************************/

static int nrf91_modem_at_setup(struct uart_dev_s *dev)
{
  struct nrf91_modem_at_s *priv = (struct nrf91_modem_at_s *)dev->priv;

  /* Reset private data */

  memset(priv, 0, sizeof(struct nrf91_modem_at_s));

  /* Initialize AT modem */

  nrf_modem_at_notif_handler_set(nrf91_modem_at_notify_handler);
  nrf_modem_at_cmd_custom_set(NULL, 0);

  return OK;
}

/****************************************************************************
 * Name: nrf91_modem_at_shutdown
 ****************************************************************************/

static void nrf91_modem_at_shutdown(struct uart_dev_s *dev)
{
  nrf_modem_at_notif_handler_set(NULL);
}

/****************************************************************************
 * Name: nrf91_modem_at_attach
 ****************************************************************************/

static int nrf91_modem_at_attach(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: nrf91_modem_at_detach
 ****************************************************************************/

static void nrf91_modem_at_detach(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: nrf91_modem_at_ioct
 ****************************************************************************/

static int nrf91_modem_at_ioctl(struct file *filep, int cmd,
                                unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Name: nrf91_modem_at_receive
 ****************************************************************************/

static int nrf91_modem_at_receive(struct uart_dev_s *dev,
                                  unsigned int *status)
{
  struct nrf91_modem_at_s *priv = (struct nrf91_modem_at_s *)dev->priv;
  char ch = 0;

  *status = 0;

  if (priv->resp != NULL && priv->notif_now == false)
    {
      priv->resp_now = true;
      ch = priv->resp[priv->resp_i++];
      if (priv->resp_i >= priv->resp_len)
        {
          priv->resp     = NULL;
          priv->resp_now = false;
        }
    }
  else if (priv->notif != NULL && priv->resp_now == false)
    {
      priv->notif_now = true;
      ch = priv->notif[priv->notif_i++];
      if (priv->notif_i >= priv->notif_len)
        {
          priv->notif     = NULL;
          priv->notif_now = false;
        }
    }

  return (int)ch;
}

/****************************************************************************
 * Name: nrf91_modem_at_rxint
 ****************************************************************************/

static void nrf91_modem_at_rxint(struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: nrf91_modem_at_rxavailable
 ****************************************************************************/

static bool nrf91_modem_at_rxavailable(struct uart_dev_s *dev)
{
  struct nrf91_modem_at_s *priv = (struct nrf91_modem_at_s *)dev->priv;
  return priv->notif || priv->resp;
}

/****************************************************************************
 * Name: nrf91_modem_at_send
 ****************************************************************************/

static void nrf91_modem_at_send(struct uart_dev_s *dev, int ch)
{
  struct nrf91_modem_at_s *priv = (struct nrf91_modem_at_s *)dev->priv;
  int                      ret  = OK;

  if (priv->tx_i + 1 > NRF91_MODEM_AT_TX)
    {
      _err("no free space in TX buffer\n");
    }

  priv->txbuf[priv->tx_i] = (char)ch;
  priv->tx_i += 1;

  /* Special formating Nordic AT interface (escape charactes) */

  if (ch == '%')
    {
      priv->txbuf[priv->tx_i] = '%';
      priv->tx_i += 1;
    }

  if (priv->txbuf[priv->tx_i - 1] == '\r')
    {
      priv->txbuf[priv->tx_i - 1] = '\0';

      /* Send AT command */

      ret = nrf_modem_at_cmd_async(nrf91_modem_at_resp_handler, priv->txbuf);
      if (ret < 0)
        {
          _err("nrf_modem_at_cmd_async failed %d\n", ret);
        }

      /* Reset buffer */

      memset(priv->txbuf, 0, NRF91_MODEM_AT_TX);
      priv->tx_i = 0;
    }
}

/****************************************************************************
 * Name: nrf91_modem_at_txint
 ****************************************************************************/

static void nrf91_modem_at_txint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      uart_xmitchars(dev);
    }
}

/****************************************************************************
 * Name: nrf91_modem_at_txready
 ****************************************************************************/

static bool nrf91_modem_at_txready(struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: nrf91_modem_at_txempty
 ****************************************************************************/

static bool nrf91_modem_at_txempty(struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_at_register
 ****************************************************************************/

int nrf91_at_register(const char *path)
{
  int ret = OK;

  /* Register driver */

  ret = uart_register(path, &g_nrf91_modem_at);
  if (ret < 0)
    {
      nerr("ERROR: register_driver failed: %d\n", ret);
    }

  return ret;
}
