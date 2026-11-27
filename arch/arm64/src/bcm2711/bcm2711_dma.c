/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_dma.c
 *
 * Author: Matteo Golin <linguini@apache.org>
 *
 * SPDX-License-Identifer: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
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

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include "bcm2711_dma.h"

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "hardware/bcm2711_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers for static initialization of DMA channels */

#define DMA_CHAN_SINIT_BASE(chno, chtype, chbase, chirq)                     \
  {                                                                          \
      .chan =                                                                \
          {                                                                  \
              .ops = &g_chanops,                                             \
          },                                                                 \
      .no = (chno),                                                          \
      .base = (chbase),                                                      \
      .irq = (chirq),                                                        \
      .type = (chtype),                                                      \
      .lock = NXMUTEX_INITIALIZER,                                           \
  }

#define DMA_CHAN_SINIT(chno, chtype, chirq)                                  \
  DMA_CHAN_SINIT_BASE(chno, chtype, BCM_DMA(chno), chirq)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMA channel type */

enum dma_type_e
{
  DMA_REG,  /* Regular */
  DMA_DMA4, /* DMA 4 */
  DMA_LITE, /* DMA Lite */
};

/* BCM2711 DMA channel
 *
 * NOTE: by storing the control block memory in this struct (static
 * allocation), we are currently ignoring the linked-list feature of the DMA
 * interfaces where control blocks can point to one another in sequence.
 */

struct bcm2711_dma_chan_s
{
  struct dma_chan_s chan; /* DMA channel struct expected by upper-half */
  mutex_t lock;           /* Mutually exclusive access to channel */
  int no;                 /* Channel number */
  int irq;                /* IRQ number for this channel */
  enum dma_type_e type;   /* Channel type */
  uint32_t base;          /* Channel registers base address */
  union
  {
    struct bcm2711_dma_cb_s reg;
    struct bcm2711_dma4_cb_s dma4;
    struct bcm2711_dmalite_cb_s lite;
  } ctrlblk;         /* Control block for channel */
  dma_callback_t cb; /* Callback for end of transfer */
  void *cb_arg;      /* Callback argument */
};

/* BCM2711 DMA device incorporating global operations */

struct bcm2711_dma_dev_s
{
  struct dma_dev_s dev; /* DMA device struct expected by upper-half */
  bool inited;          /* True if DMA interfaces initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Channel operations */

static int bcm2711_config(struct dma_chan_s *chan,
                          const struct dma_config_s *cfg);
static int bcm2711_start(struct dma_chan_s *chan, dma_callback_t callback,
                         void *arg, uintptr_t dst, uintptr_t src, size_t len);
static int bcm2711_start_cyclic(struct dma_chan_s *chan,
                                dma_callback_t callback, void *arg,
                                uintptr_t dst, uintptr_t src, size_t len,
                                size_t period_len);

#ifdef CONFIG_DMA_LINK
static int bcm2711_start_link(struct dma_chan_s *chan,
                              dma_callback_t callback, void *arg,
                              unsigned int work_mode,
                              struct dma_link_config_s *cfg);
#endif

static int bcm2711_stop(struct dma_chan_s *chan);
static int bcm2711_pause(struct dma_chan_s *chan);
static int bcm2711_resume(struct dma_chan_s *chan);
static size_t bcm2711_residual(struct dma_chan_s *chan);

/* Device operations */

static struct dma_chan_s *bcm2711_get_chan(struct dma_dev_s *dev,
                                           unsigned int ident);
static void bcm2711_put_chan(struct dma_dev_s *dev, struct dma_chan_s *chan);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Global device handling arbitration of channels */

static struct bcm2711_dma_dev_s g_dev = {
    .dev =
        {
            .get_chan = bcm2711_get_chan,
            .put_chan = bcm2711_put_chan,
        },
    .inited = false,
};

/* Operations for each channel to use */

static const struct dma_ops_s g_chanops = {
    .config = bcm2711_config,
    .start = bcm2711_start,
    .start_cyclic = bcm2711_start_cyclic,
#ifdef CONFIG_DMA_LINK
    .start_link = bcm2711_start_link,
#endif
    .stop = bcm2711_stop,
    .pause = bcm2711_pause,
    .resume = bcm2711_resume,
    .residual = bcm2711_residual,
};

/* List of all channels.
 *
 * Channels are initialized such that their channel number corresponds to
 * their index in this list.
 */

static struct bcm2711_dma_chan_s g_channels[BCM_DMA_CHANNUM] = {
    DMA_CHAN_SINIT(0, DMA_REG, BCM_IRQ_VC_DMA0),
    DMA_CHAN_SINIT(1, DMA_REG, BCM_IRQ_VC_DMA1),
    DMA_CHAN_SINIT(2, DMA_REG, BCM_IRQ_VC_DMA2),
    DMA_CHAN_SINIT(3, DMA_REG, BCM_IRQ_VC_DMA3),
    DMA_CHAN_SINIT(4, DMA_REG, BCM_IRQ_VC_DMA4),
    DMA_CHAN_SINIT(5, DMA_REG, BCM_IRQ_VC_DMA5),
    DMA_CHAN_SINIT(6, DMA_REG, BCM_IRQ_VC_DMA6),
    DMA_CHAN_SINIT(7, DMA_LITE, BCM_IRQ_VC_DMA7N8),
    DMA_CHAN_SINIT(8, DMA_LITE, BCM_IRQ_VC_DMA7N8),
    DMA_CHAN_SINIT(9, DMA_LITE, BCM_IRQ_VC_DMA9N10),
    DMA_CHAN_SINIT(10, DMA_LITE, BCM_IRQ_VC_DMA9N10),
    DMA_CHAN_SINIT(11, DMA_DMA4, BCM_IRQ_VC_DMA11),
    DMA_CHAN_SINIT(12, DMA_DMA4, BCM_IRQ_VC_DMA12),
    DMA_CHAN_SINIT(13, DMA_DMA4, BCM_IRQ_VC_DMA13),
    DMA_CHAN_SINIT(14, DMA_DMA4, BCM_IRQ_VC_DMA14),
    DMA_CHAN_SINIT_BASE(15, DMA_REG, BCM_DMA15, BCM_IRQ_VC_DMA15),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_channel_handler
 *
 * Description:
 *   Handle interrupt servicing for a specific DMA channel.
 *
 * Input Parameters:
 *   priv - The channel to service the interrupt for
 *
 * Returned Value:
 *   0 on success, negated errno on failure.
 *
 ****************************************************************************/

static int bcm2711_channel_handler(struct bcm2711_dma_chan_s *priv)
{
  /* Check that this channel actually needs handling */

  if (!(getreg32(BCM_DMA_INT_STATUS) & BCM_DMA_INT_STATUS_INT(priv->no)))
    {
      return 0;
    }

  /* Handle the interrupt TODO */

  /* Clear the interrupt TODO */

  return 0;
}

/****************************************************************************
 * Name: bcm2711_interrupt_handler
 *
 * Description:
 *   The primary DMA interrupt handler. This handler is used for _every_ DMA
 *   channel IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number
 *   context - The interrupt context
 *   arg - The argument passed to the interrupt handler
 *
 * Returned Value:
 *   0 on success, negated errno on failure.
 *
 ****************************************************************************/

static int bcm2711_interrupt_handler(int irq, void *context, void *arg)
{
  /* First, check if this is a shared DMA Lite IRQ. If it is, we might need to
   * service both channels.
   *
   * `bcm2711_channel_handler` checks the interrupt status bit of the channel,
   * so it will not perform unneeded servicing.
   */

  if (irq == BCM_IRQ_VC_DMA7N8)
    {
      bcm2711_channel_handler(&g_channels[7]);
      bcm2711_channel_handler(&g_channels[8]);
      return 0;
    }
  else if (irq == BCM_IRQ_VC_DMA9N10)
    {
      bcm2711_channel_handler(&g_channels[9]);
      bcm2711_channel_handler(&g_channels[10]);
      return 0;
    }

  /* If we are here, this was a not a DMA Lite channel shared IRQ. In this
   * case, we can just focus on servicing the interrupt which corresponds to
   * this channel. When registering the interrupt, the argument we passed was
   * the reference to the channel device struct.
   */

  bcm2711_channel_handler(arg);
  return 0;
}

/****************************************************************************
 * Name: bcm2711_config
 *
 * Description:
 *   Configure DMA before using
 *
 * Input Parameters:
 *   chan - The channel to configure
 *   cfg - The configuration settings to use
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 *
 ****************************************************************************/

static int bcm2711_config(struct dma_chan_s *chan,
                          const struct dma_config_s *cfg)
{
  struct bcm2711_dma_chan_s *priv = (struct bcm2711_dma_chan_s *)chan;

  /* For all types:
   *
   * timeout: might need an external watchdog?
   * option: ignored, no special options right now.
   */

  switch (priv->type)
    {
    case DMA_LITE:

      /* direction: ?
       * priority: maybe can map to CS_PANIC_PRIORITY/CS_PRIORITY?
       *
       * dst_width: TI_DEST_WIDTH
       * src_width: TI_SRC_WIDTH
       *
       * dst_drq: TI_DEST_DREQ? TI_PERMAP?
       * src_drq: TI_SRC_DREQ? TI_PERMAP?
       *
       * dst_step: TI_DEST_INC, only some values are allowed
       * src_step: TI_SRC_INC, only some values are allowed
       */

      /* TODO */

      break;

    case DMA_REG:

      /* direction: ?
       * priority:
       *
       * dst_width:
       * src_width:
       *
       * dst_drq:
       * src_drq:
       *
       * dst_step:
       * src_step:
       */

      /* TODO */

      break;

    case DMA_DMA4:

      /* direction: ?
       * priority: maybe can map to CS_PANIC_QOS/CS_QOS
       *
       * dst_width: DESTI_SIZE
       * src_width: SRCI_SIZE
       *
       * dst_drq: TI_D_DREQ? TI_PERMAP?
       * src_drq: TI_S_DREQ? TI_PERMAP?
       *
       * dst_step can go in DESTI_STRIDE as signed 2's compliment
       * src_step can go in SRCI_STRIDE as a signed 2's compliment
       * If there is a stride set, then set TI_TDMODE bit.
       */

      /* TODO */

      break;
    }

  /* TODO */

  return 0;
}

/****************************************************************************
 * Name: bcm2711_start
 *
 * Description:
 *   Start the DMA transfer. NOTE: The DMA module does *NOT* perform any
 *   cache operations. It is the responsibility of the DMA client to clean
 *   DMA buffers after staring of the DMA TX operations.
 *
 * Assumptions:
 *   DMA_CONFIG was called before this to configure the DMA transfer.
 *
 * Input Parameters:
 *   chan - The channel to start
 *   callback - The callback to be called when the transfer is finished
 *   arg - The argument for the callback
 *   dst - The destination address
 *   src - The source address
 *   len - The length to transfer
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int bcm2711_start(struct dma_chan_s *chan, dma_callback_t callback,
                         void *arg, uintptr_t dst, uintptr_t src, size_t len)
{
  struct bcm2711_dma_chan_s *priv = (struct bcm2711_dma_chan_s *)chan;

  priv->cb = callback;
  priv->cb_arg = arg;

  /* NOTE: We've assumed that `bcm2711_config` has already been called to
   * configure settings for the transfer. Therefore, reserved fields of the
   * control block are already zeroed, and other fields have populated
   * configuration settings. We only need to populate the destination address,
   * source address and transfer length now.
   *
   * We also enable interrupts for control block completion here.
   */

  switch (priv->type)
    {
    case DMA_REG:

      /* Only 32-bit values allowed here, check for no upper bits */

      if ((src & UINT32_MAX) != src || (dst & UINT32_MAX) != dst ||
          (len & UINT32_MAX) != len)
        {
          return -EINVAL;
        }

      priv->ctrlblk.reg.source_ad = src;
      priv->ctrlblk.reg.dest_ad = dst;

      /* TODO: txfr length setting based on 2D mode or not */

      /* Enable control block complete interrupt */

      priv->ctrlblk.dma4.ti |= BCM_DMA4_TI_INTEN;
      break;

    case DMA_LITE:

      /* Only 32-bit src, destination addresses allowed.
       * 16-bit transfer length max imposed.
       */

      if ((src & UINT32_MAX) != src || (dst & UINT32_MAX) != dst ||
          (len & UINT16_MAX) != len)
        {
          return -EINVAL;
        }

      priv->ctrlblk.lite.source_ad = src;
      priv->ctrlblk.lite.dest_ad = dst;
      priv->ctrlblk.lite.txfr_len = len;

      /* Enable control block complete interrupt */

      priv->ctrlblk.lite.ti |= BCM_DMA_TI_INTEN;
      break;

    case DMA_DMA4:

      /* Only up to 32-bit lengths supported. Destination and source addresses
       * can be up to 40 bits long.
       */

      if ((src & 0xffffffffff) != src || (dst & 0xffffffffff) != dst ||
          (len & UINT32_MAX) != len)
        {
          return -EINVAL;
        }

      /* Put lower address bits in control block src/dst */

      priv->ctrlblk.dma4.src = src & UINT32_MAX;
      priv->ctrlblk.dma4.dest = dst & UINT32_MAX;

      /* Put upper address bits in control block srci/desti */

      priv->ctrlblk.dma4.srci =
          (src >> BCM_DMA4_SRCI_ADDR_SHIFT) & BCM_DMA4_SRCI_ADDR;
      priv->ctrlblk.dma4.desti =
          (dst >> BCM_DMA4_DESTI_ADDR_SHIFT) & BCM_DMA4_DESTI_ADDR;

      /* TODO: txfr length setting based on 2D mode or not */

      /* Enable control block complete interrupt */

      priv->ctrlblk.reg.ti |= BCM_DMA_TI_INTEN;
      break;
    }

  /* A DMA transfer is started by writing the address of a control block
   * structure into CONBLK_AD (or CB for DMA4) and then setting the active
   * bit.
   */

  switch (priv->type)
    {
    case DMA_REG:
      /* Intentional fall-through */
    case DMA_LITE:
      putreg32((uintptr_t)&priv->ctrlblk, BCM_DMA_CONBLK_AD(priv->base));
      break;
    case DMA_DMA4:
      putreg32((uintptr_t)&priv->ctrlblk, BCM_DMA4_CB(priv->base));
      break;
    }

  /* Control block is fully configured, interrupts are enabled, control block
   * address is populated; activate the channel to start the DMA operation.
   */

  modreg32(BCM_DMA_CS_ACTIVE, BCM_DMA_CS_ACTIVE, BCM_DMA_CS(priv->base));
  return 0;
}

/****************************************************************************
 * Name: bcm2711_start_cyclic
 *
 * Description:
 *   Start the cyclic DMA transfer.
 *   Note: the callback gets called for each period length data DMA transfer.
 *
 * Input Parameters:
 *   chan - The channel to do the transfer on
 *   callback - The callback to call at each period
 *   arg - The argument to the callback
 *   dst - The destination address
 *   src - The source address
 *   len - The length of the transfer
 *   period_len - The period length in bytes
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 *
 ****************************************************************************/

static int bcm2711_start_cyclic(struct dma_chan_s *chan,
                                dma_callback_t callback, void *arg,
                                uintptr_t dst, uintptr_t src, size_t len,
                                size_t period_len)
{
  struct bcm2711_dma_chan_s *priv = (struct bcm2711_dma_chan_s *)chan;

  priv->cb = callback;
  priv->cb_arg = arg;

  /* TODO */

  return -ENOSYS;
}

#ifdef CONFIG_DMA_LINK

/****************************************************************************
 * Name: bcm2711_start_link
 *
 * Description:
 *   Start the DMA link transfer.
 *   Note: the callback gets called when the DMA link transfer finishes.
 *
 * Input Parameters:
 *   chan - The channel to perform the transfer on
 *   callback - The callback to call when the transfer finishes
 *   arg - The argument to the callback function
 *   work_mode - Not sure? I think work queue TODO
 *   cfg - A linked list of source/destination addresses and sizes
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 *
 ****************************************************************************/

static int bcm2711_start_link(struct dma_chan_s *chan,
                              dma_callback_t callback, void *arg,
                              unsigned int work_mode,
                              struct dma_link_config_s *cfg)
{
  /* TODO */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: bcm2711_stop
 *
 * Description:
 *   Stop the DMA transfer.
 *
 * Input Parameters:
 *   chan - The channel whose transfer to stop
 *
 * Returned Value:
 *   0 on success, negated error code otherwise.
 *
 ****************************************************************************/

static int bcm2711_stop(struct dma_chan_s *chan)
{
  struct bcm2711_dma_chan_s *priv = (struct bcm2711_dma_chan_s *)chan;

  /* Clear the next control block register, then abort the current transfer.
   * This will cause the transfer to stop.
   */

  switch (priv->type)
    {
    case DMA_REG:
      /* Intentional fall-through */
    case DMA_LITE:
      putreg32(0, BCM_DMA_NEXTCONBK(priv->base));
      break;
    case DMA_DMA4:
      putreg32(0, BCM_DMA4_NEXT_CB(priv->base));
      break;
    }

  modreg32(BCM_DMA_CS_ABORT, BCM_DMA_CS_ABORT, BCM_DMA_CS(priv->base));
  return 0;
}

/****************************************************************************
 * Name: bcm2711_pause
 *
 * Description:
 *   Pause the DMA transfer. After DMA_PAUSE() is called, DMA_RESUME() must
 *   be called to restart the transfer again.
 *
 * Input Parameters:
 *   chan - The channel whose transfer to pause
 *
 * Returned Value:
 *   0 on success and negative error code on failure
 *
 ****************************************************************************/

static int bcm2711_pause(struct dma_chan_s *chan)
{
  struct bcm2711_dma_chan_s *priv = (struct bcm2711_dma_chan_s *)chan;

  modreg32(0, BCM_DMA_CS_ACTIVE, BCM_DMA_CS(priv->base));
  return 0;
}

/****************************************************************************
 * Name: bcm2711_resume
 *
 * Description:
 *   Resume the DMA transfer.
 *
 * Input Parameters:
 *   chan - descr
 *
 * Returned Value:
 *   description
 *
 ****************************************************************************/

static int bcm2711_resume(struct dma_chan_s *chan)
{
  struct bcm2711_dma_chan_s *priv = (struct bcm2711_dma_chan_s *)chan;

  modreg32(BCM_DMA_CS_ACTIVE, BCM_DMA_CS_ACTIVE, BCM_DMA_CS(priv->base));
  return 0;
}

/****************************************************************************
 * Name: bcm2711_residual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred.
 *
 * Input Parameters:
 *   chan - The channel to get the residual bytes of
 *
 * Returned Value:
 *   The number of bytes remaining to be transferred.
 *
 ****************************************************************************/

static size_t bcm2711_residual(struct dma_chan_s *chan)
{
  struct bcm2711_dma_chan_s *priv = (struct bcm2711_dma_chan_s *)chan;
  uint32_t regval;
  size_t len = 0;

  switch (priv->type)
    {
    case DMA_LITE:
      regval = getreg32(BCM_DMA_TXFR_LEN(priv->base));
      len = regval & BCM_DMA_TXFR_LEN_XLENGTH;
      break;
    case DMA_REG:
      regval = getreg32(BCM_DMA_TXFR_LEN(priv->base));
      len = regval; /* TODO: handle 2D case */
      break;
    case DMA_DMA4:
      regval = getreg32(BCM_DMA4_LEN(priv->base));
      len = regval; /* TODO: handle 2D case */
      break;
    }

  return len;
}

/****************************************************************************
 * Name: bcm2711_get_chan
 *
 * Description:
 *   Get a DMA channel. This function gives the caller mutually exclusive
 *   access to the DMA channel specified by the 'ident' argument.
 *
 *   If the DMA channel is not available, then DMA_GET_CHAN will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   DMA_PUT_CHAN().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the DMA_GET_CHAN
 *   call for the other will hang forever in this function!
 *
 * Input Parameters:
 *   dev - DMA device struct reference
 *   ident - The number of the channel to get
 *
 * Returned Value:
 *   A reference to the DMA channel device. Returns NULL if the channel number
 *   is invalid.
 *
 ****************************************************************************/

static struct dma_chan_s *bcm2711_get_chan(struct dma_dev_s *dev,
                                           unsigned int ident)
{
  struct bcm2711_dma_chan_s *priv;

  if (ident > 15)
    {
      return NULL;
    }

  /* Lock the DMA channel for use */

  priv = &g_channels[ident];
  nxmutex_lock(&g_channels[ident].lock);

  /* Enable power for the DMA channel (no enable toggle for channel 15) */

  if (ident != 15)
    {
      modreg32(BCM_DMA_ENABLE_EN(priv->no), BCM_DMA_ENABLE_EN(priv->no),
               BCM_DMA_ENABLE);
    }

  /* Reset the DMA channel TODO */

  /* Enable the interrupt handler for this channel, which is already
   * registered. Can be unconditional, it doesn't matter if we enable the
   * shared IRQ twice.
   */

  up_enable_irq(priv->irq);

  return &priv->chan;
}

/****************************************************************************
 * Name: bcm2711_put_chan
 *
 * Description:
 *   Release a DMA channel. If another thread is waiting for this DMA channel
 *   in a call to DMA_GET_CHAN, then this function will re-assign the DMA
 *   channel to that thread and wake it up. NOTE: The 'chan' used in this
 *   argument must NEVER be used again until DMA_GET_CHAN() is called again
 *   to re-gain access to the channel.
 *
 *   WARNING: assumes that the channel is not performing a DMA operation
 *   during release time.
 *
 * Input Parameters:
 *   dev - The DMA device struct reference
 *   chan - The DMA channel struct reference being released
 *
 ****************************************************************************/

static void bcm2711_put_chan(struct dma_dev_s *dev, struct dma_chan_s *chan)
{
  uint32_t regval;
  struct bcm2711_dma_chan_s *priv = (struct bcm2711_dma_chan_s *)chan;

  DEBUGASSERT(chan != NULL);

  /* The channel is no longer being used, we can safely power it down (not an
   * option for DMA15).
   *
   * WARNING: disabling a channel during DMA operation is fatal.
   * TODO: check DMA_BUSY bit.
   */

  if (priv->no != 15)
    {
      modreg32(0, BCM_DMA_ENABLE_EN(priv->no), BCM_DMA_ENABLE);
    }

  /* Disable the interrupt handler for this channel.
   *
   * For DMA Lite channels, we check to see if both of the two
   * channels associated with the IRQ are disabled before un-registering the
   * handler. Otherwise, we might un-register the handler while the other
   * channel sharing it still needs it.
   */

  switch (priv->type)
    {
    case DMA_LITE:
      regval = getreg32(BCM_DMA_ENABLE);

      if (priv->irq == BCM_IRQ_VC_DMA7N8 &&
          !(regval & (BCM_DMA_ENABLE_EN(7) | BCM_DMA_ENABLE_EN(7))))
        {
          up_disable_irq(priv->irq); /* 7 & 8 are both disabled */
        }
      else if (priv->irq == BCM_IRQ_VC_DMA9N10 &&
               !(regval & (BCM_DMA_ENABLE_EN(9) | BCM_DMA_ENABLE_EN(10))))
        {
          up_disable_irq(priv->irq); /* 9 & 10 are both disabled */
        }

      break;
    default: /* All other DMA channel kinds have their own IRQ */
      up_disable_irq(priv->irq);
      break;
    }

  /* Release the lock on the channel */

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct dma_dev_s *bcm2711_dma_initialize(void)
{
  unsigned i;
  int err;
  struct bcm2711_dma_chan_s *chan;

  if (g_dev.inited)
    {
      return &g_dev.dev;
    }

  /* Disable all DMA channels by default when not used */

  putreg32(0, BCM_DMA_ENABLE);

  /* Register interrupt handler functions but do not enable them yet.
   *
   * NOTE: interrupt handlers are enabled when DMA channels are claimed
   * using `get_chan`. This is because each channel has its own interrupt
   * (except DMA Lite channels which share). We conserve power and interrupts
   * by only enabling DMA channels that are requested for use, and by
   * disabling those channels' interrupts & power when they are returned from
   * use with `put_chan`.
   */

  for (i = 0; i < BCM_DMA_CHANNUM; i++)
    {
      chan = &g_channels[i];

      /* TODO: should I return NULL if interrupt handlers fail to attach? */

      switch (chan->type)
        {
        case DMA_LITE:

          /* Check if interrupt handler is already registered; don't register
           * twice. TODO: is this possible to check?
           * NOTE: does it matter to register twice here?
           */

          err = irq_attach(chan->irq, bcm2711_interrupt_handler, NULL);
          if (err < 0)
            {
              dmaerr("Could not attach interrupt handler for DMA Lite %d",
                     chan->no);
              return NULL;
            }

          up_prioritize_irq(chan->irq, 0);
          up_set_irq_type(chan->irq, IRQ_RISING_EDGE);
          break;

        default: /* All other kinds of DMA channels */
          err = irq_attach(chan->irq, bcm2711_interrupt_handler, chan);
          if (err < 0)
            {
              dmaerr("Could not attach interrupt handler for DMA channel %d",
                     chan->no);
              return NULL;
            }

          up_prioritize_irq(chan->irq, 0);
          up_set_irq_type(chan->irq, IRQ_RISING_EDGE);
          break;
        }
    }

  g_dev.inited = true;
  return &g_dev.dev;
}
