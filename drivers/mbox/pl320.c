/****************************************************************************
 * drivers/mbox/pl320.c
 * Driver for the PL320 mailbox
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

#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mbox/mbox.h>
#include <nuttx/mbox/pl320.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define getreg32(a)                 (*(volatile uint32_t *)(a))
#define putreg32(v, a)              (*(volatile uint32_t *)(a) = (v))

#define IPCMXSOURCE(m)              (((m) * 0x40))
#define IPCMXDSET(m)                (((m) * 0x40) + 0x004)
#define IPCMXDCLEAR(m)              (((m) * 0x40) + 0x008)
#define IPCMXDSTATUS(m)             (((m) * 0x40) + 0x00C)
#define IPCMXMODE(m)                (((m) * 0x40) + 0x010)
#define IPCMXMSET(m)                (((m) * 0x40) + 0x014)
#define IPCMXMCLEAR(m)              (((m) * 0x40) + 0x018)
#define IPCMXMSTATUS(m)             (((m) * 0x40) + 0x01C)
#define IPCMXSEND(m)                (((m) * 0x40) + 0x020)
#define IPCMXDR(m, dr)              (((m) * 0x40) + ((dr) * 4) + 0x024)

#define IPCMMIS(irq)                (((irq) * 8) + 0x800)
#define IPCMRIS(irq)                (((irq) * 8) + 0x804)

#define MBOX_MASK(n)                (1U << (n))
#define CHAN_MASK(n)                (1U << (n))

#define IPCM_DR_SIZE                (7)

#define SEND_INACTIVE               (0x0)
#define SEND_MSG                    (0x1)
#define SEND_ACK                    (0x2)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct pl320_dev_s
{
  struct mbox_dev_s       base;
  uintptr_t               reg_base;
  int                     ipcmint;
  int                     irq_num;
  struct pl320_chan_s     *chan_map[PL320_MAX_CHAN_NUM];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct mbox_chan_s *pl320_getchan(FAR struct mbox_dev_s *dev,
                                         FAR void *arg);

static int pl320_setup(FAR struct mbox_chan_s *chan, FAR void *arg);

static int pl320_shutdown(FAR struct mbox_chan_s *chan);

static int pl320_send(FAR struct mbox_chan_s *chan,
                      FAR const void *data, size_t size);

static int pl320_recv(FAR struct mbox_chan_s *chan, FAR void *data);

static int pl320_acknowledge(FAR struct mbox_chan_s *chan,
                             FAR const void *data, size_t size);

static int pl320_txfinish(FAR struct mbox_chan_s *chan);

static int pl320_setcallback(FAR struct mbox_chan_s *chan,
                             mbox_callback_t callback, FAR void *arg);

static bool pl320_rxavailable(FAR struct mbox_chan_s *chan);

static bool pl320_txready(FAR struct mbox_chan_s *chan);

static bool pl320_txacked(FAR struct mbox_chan_s *chan);

static int pl320_irq_handler(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct mbox_ops_s g_pl320mbox_ops =
{
  .getchan       = pl320_getchan,
  .setup         = pl320_setup,
  .shutdown      = pl320_shutdown,
  .send          = pl320_send,
  .recv          = pl320_recv,
  .acknowledge   = pl320_acknowledge,
  .txfinish      = pl320_txfinish,
  .setcallback   = pl320_setcallback,
  .rxavailable   = pl320_rxavailable,
  .txready       = pl320_txready,
  .txacked       = pl320_txacked
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pl320_getchan
 *
 * Description:
 *   Get pl320 channel handle by chan id
 *
 ****************************************************************************/

static struct mbox_chan_s *pl320_getchan(FAR struct mbox_dev_s *dev,
                                         FAR void *arg)
{
  uint32_t ch = (uint32_t)(uintptr_t)arg;
  FAR struct pl320_dev_s *priv;

  if (!dev || ch >= PL320_MAX_CHAN_NUM)
    {
      return NULL;
    }

  priv = (FAR struct pl320_dev_s *)dev;
  return (FAR struct mbox_chan_s *)priv->chan_map[ch];
}

/****************************************************************************
 * Name: pl320_setup
 *
 * Description:
 *   Set up a PL320 channel before transmission. Configure IPCMXSOURCE,
 *   IPCMXDEST and IPCMXMSET registers if the ctrl field is set.
 *
 ****************************************************************************/

static int pl320_setup(FAR struct mbox_chan_s *chan, FAR void *arg)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;

  if (!chan)
    {
      return -EINVAL;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  if (priv_chan->ctrl)
    {
      int src  = priv_chan->src;
      int dst  = priv_chan->dst;
      int mbox = priv_chan->mbox;

      if (getreg32(priv_dev->reg_base + IPCMXSOURCE(mbox)) != 0)
        {
          putreg32(0x0, priv_dev->reg_base + IPCMXSOURCE(mbox));
        }

      putreg32(CHAN_MASK(src), priv_dev->reg_base + IPCMXSOURCE(mbox));
      putreg32(CHAN_MASK(dst), priv_dev->reg_base + IPCMXDSET(mbox));
      putreg32(CHAN_MASK(src) | CHAN_MASK(dst),
               priv_dev->reg_base + IPCMXMSET(mbox));
    }

  return 0;
}

/****************************************************************************
 * Name: pl320_shutdown
 *
 * Description:
 *   Release ownership of this channel by clearing IPCMXSOURCE
 *
 ****************************************************************************/

static int pl320_shutdown(FAR struct mbox_chan_s *chan)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;

  if (!chan)
    {
      return -EINVAL;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  putreg32(0x0, priv_dev->reg_base + IPCMXSOURCE(priv_chan->mbox));
  return 0;
}

/****************************************************************************
 * Name: pl320_send
 *
 * Description:
 *   Send a mailbox message over the PL320 mailbox. This function is
 *   attached to the virtual mbox_dev ".send()" pointer. This way every
 *   time a mailbox message is sent it is called.
 *
 * Input Parameters:
 *   chan  - Pointer to the (virtual) mbox_chan_s.
 *   msg   - Message to be send.
 *
 * Returned Value:
 *   0 on success or a negative error.
 *
 ****************************************************************************/

static int pl320_send(FAR struct mbox_chan_s *chan,
                      FAR const void *data, size_t size)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;
  uint32_t buffer[IPCM_DR_SIZE];

  if (!chan || !data)
    {
      return -EINVAL;
    }

  if (size > PL320_MSG_SIZE)
    {
      return -EMSGSIZE;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  memset(buffer, 0, sizeof(buffer));
  memcpy(buffer, data, size);
  for (size_t i = 0; i < IPCM_DR_SIZE; ++i)
    {
      putreg32(buffer[i], priv_dev->reg_base + IPCMXDR(priv_chan->mbox, i));
    }

  putreg32(SEND_MSG, priv_dev->reg_base + IPCMXSEND(priv_chan->mbox));
  return 0;
}

/****************************************************************************
 * Name: pl320_recv
 *
 * Description:
 *   Recv a mailbox message over the PL320 mailbox. This function is
 *   called when rxavailable happened.
 *
 * Input Parameters:
 *   chan  - Pointer to the (virtual) mbox_chan_s.
 *   msg   - Message to hold the data read
 *
 * Returned Value:
 *   0 on success or a negative error.
 *
 ****************************************************************************/

static int pl320_recv(FAR struct mbox_chan_s *chan, FAR void *data)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;
  FAR volatile uint32_t   *buffer;

  if (!chan || !data)
    {
      return -EINVAL;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  buffer = (FAR volatile uint32_t *)data;
  for (size_t i = 0; i < IPCM_DR_SIZE; ++i)
    {
      buffer[i] = getreg32(priv_dev->reg_base + IPCMXDR(priv_chan->mbox, i));
    }

  return IPCM_DR_SIZE * sizeof(uint32_t);
}

/****************************************************************************
 * Name: pl320_acknowledge
 *
 * Description:
 *   Send an acknowledge to remote sender after received data.
 *
 ****************************************************************************/

static int pl320_acknowledge(FAR struct mbox_chan_s *chan,
                             FAR const void *data, size_t size)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;

  if (!chan)
    {
      return -EINVAL;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  putreg32(SEND_ACK, priv_dev->reg_base + IPCMXSEND(priv_chan->mbox));
  return 0;
}

/****************************************************************************
 * Name: pl320_txfinish
 *
 * Description:
 *   Finish the transmission and prepare for the next transfer.
 *
 ****************************************************************************/

static int pl320_txfinish(FAR struct mbox_chan_s *chan)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;

  if (!chan)
    {
      return -EINVAL;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  putreg32(SEND_INACTIVE, priv_dev->reg_base + IPCMXSEND(priv_chan->mbox));
  return 0;
}

/****************************************************************************
 * Name: pl320_setcallback
 *
 * Description:
 *   Register an receiving callback function to designated mailbox channel.
 *   The callback function will be saved in pl320_chans structure of
 *   the corresponding channel.
 ****************************************************************************/

static int pl320_setcallback(FAR struct mbox_chan_s *chan,
                             mbox_callback_t callback, FAR void *arg)
{
  if (!chan)
    {
      return -EINVAL;
    }

  chan->callback = callback;
  chan->priv     = arg;
  return 0;
}

/****************************************************************************
 * Name: pl320_rxavailable
 *
 * Description:
 *   Check if the channel is ready to receive data.
 *
 ****************************************************************************/

static bool pl320_rxavailable(FAR struct mbox_chan_s *chan)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;

  if (!chan)
    {
      return false;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  return getreg32(priv_dev->reg_base + IPCMXSEND(priv_chan->mbox))
      == SEND_MSG;
}

/****************************************************************************
 * Name: pl320_txready
 *
 * Description:
 *   Check if the channel is ready to transmit message.
 *
 ****************************************************************************/

static bool pl320_txready(FAR struct mbox_chan_s *chan)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;

  if (!chan)
    {
      return false;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  return getreg32(priv_dev->reg_base + IPCMXSEND(priv_chan->mbox))
      == SEND_INACTIVE;
}

/****************************************************************************
 * Name: pl320_txacked
 *
 * Description:
 *   Check if the channel is acked.
 *
 ****************************************************************************/

static bool pl320_txacked(FAR struct mbox_chan_s *chan)
{
  FAR struct pl320_chan_s *priv_chan;
  FAR struct pl320_dev_s  *priv_dev;

  if (!chan)
    {
      return false;
    }

  priv_chan = (FAR struct pl320_chan_s *)chan;
  priv_dev  = (FAR struct pl320_dev_s *)chan->dev;

  return getreg32(priv_dev->reg_base + IPCMXSEND(priv_chan->mbox))
      == SEND_ACK;
}

/****************************************************************************
 * Name: pl320_irq_handler
 *
 * Description:
 *   Interrupt handler, it happened when the IPCMXSEND register was set.
 *
 ****************************************************************************/

static int pl320_irq_handler(int irq, void *context, void *arg)
{
  FAR struct pl320_dev_s *dev = (FAR struct pl320_dev_s *)arg;
  DEBUGASSERT(dev != NULL);

  uint32_t irq_stat = getreg32(dev->reg_base + IPCMMIS(dev->ipcmint));

  for (int m = 0; m < PL320_MAX_MBOX_NUM; ++m)
    {
      /* Find the mailbox that triggered this interrupt */

      if (irq_stat & MBOX_MASK(m))
        {
          uint32_t src = getreg32(dev->reg_base + IPCMXSOURCE(m));
          uint32_t dst = getreg32(dev->reg_base + IPCMXDSTATUS(m));

          FAR struct pl320_chan_s *chan =
            (FAR struct pl320_chan_s *)
              pl320_getchan(&dev->base, (FAR void *)(uintptr_t)m);

          if (!chan)
            {
              return -ENXIO;
            }

          if (src & CHAN_MASK(dev->ipcmint))
            {
              /* It's a TX channel, executing in ack irq */

              mbox_chan_tx_done(&chan->base);
            }
          else if (dst & CHAN_MASK(dev->ipcmint))
            {
              /* It's a RX channel */

              mbox_chan_rx_data(&chan->base);
            }
          else
            {
              /* Unknown status */

              return -EIO;
            }
        }
    }

  return 0;
}

/****************************************************************************
 * Name: pl320_initialize
 *
 * Description:
 *   Initialize the PL320 mailbox device.
 *
 * Input Parameters:
 *   chans     - Pointer to PL320 channel configuration array.
 *   num_chans - Number of chans.
 *   reg_base  - Register address of PL320 device.
 *
 * Returned Value:
 *   Common mbox_dev_s instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct mbox_dev_s *pl320_initialize(
    FAR const struct pl320_config_s *config, uintptr_t reg_base)
{
  FAR struct pl320_dev_s *priv;
  size_t i;

  /* Sanity check */

  DEBUGASSERT(config != NULL && config->chans != NULL
    && config->num_chans <= PL320_MAX_CHAN_NUM && reg_base);

  /* Initialize the PL320 mailbox device structure */

  priv = kmm_zalloc(sizeof(struct pl320_dev_s));

  if (priv == NULL)
    {
      return NULL;
    }

  priv->base.ops       = &g_pl320mbox_ops;
  priv->base.chans     = (FAR struct mbox_chan_s *)config->chans;
  priv->base.num_chans = config->num_chans;

  priv->reg_base  = reg_base;
  priv->ipcmint   = config->ipcmint;
  priv->irq_num   = config->irq_num;

  /* Initialize the PL320 mailbox channels */

  for (i = 0; i < config->num_chans; ++i)
    {
      FAR struct pl320_chan_s *chan = &(config->chans[i]);

      if (chan->mbox >= PL320_MAX_MBOX_NUM)
        {
          goto err_chan;
        }

      priv->chan_map[chan->mbox] = chan;
      chan->base.id = chan->mbox;

      enum mbox_direction_e dir = (chan->src == priv->ipcmint) ?
          MBOX_TX : MBOX_RX;

      if (mbox_chan_init(&chan->base, &priv->base,
                         dir, chan->txbuf, chan->bufsize) != 0)
        {
          goto err_chan;
        }

      pl320_setup(&chan->base, NULL);
    }

  /* Initialize the IPCMINT interrupt */

  if (irq_attach(priv->irq_num, pl320_irq_handler, priv))
    {
      /* Could not attach the ISR to the interrupt */

      goto err_irq;
    }

  up_enable_irq(priv->irq_num);

  return &(priv->base);
err_irq:
  i = config->num_chans;
err_chan:
  while (i > 0)
    {
      mbox_chan_deinit(&config->chans[--i].base);
    }

  kmm_free(priv);
  return NULL;
}
