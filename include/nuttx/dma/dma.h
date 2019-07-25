/****************************************************************************
 * include/nuttx/dma/dma.h
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

#ifndef __INCLUDE_NUTTX_DMA_DMA_H
#define __INCLUDE_NUTTX_DMA_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA transfer direction indicator */

#define DMA_MEM_TO_MEM          1
#define DMA_MEM_TO_DEV          2
#define DMA_DEV_TO_MEM          3
#define DMA_DEV_TO_DEV          4

#ifdef CONFIG_DMA_LINK
#  define DMA_BLOCK_MODE        0
#  define DMA_SRC_LINK_MODE     1
#  define DMA_DST_LINK_MODE     2
#  define DMA_DUAL_LINK_MODE    3
#endif

/****************************************************************************
 * Name: DMA_GET_CHAN
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
 *   ident - Identifies the channel resource
 *
 * Returned Value:
 *   Provided that 'ident' is valid, this function ALWAYS returns a non-NULL,
 *   dma_chan_s instance.  (If 'ident' is invalid, the function will assert
 *   if debug is enabled or do something ignorant otherwise).
 *
 ****************************************************************************/

#define DMA_GET_CHAN(dev, ident) (dev)->get_chan(dev, ident)

/****************************************************************************
 * Name: DMA_PUT_CHAN
 *
 * Description:
 *   Release a DMA channel. If another thread is waiting for this DMA channel
 *   in a call to DMA_GET_CHAN, then this function will re-assign the DMA
 *   channel to that thread and wake it up. NOTE: The 'chan' used in this
 *   argument must NEVER be used again until DMA_GET_CHAN() is called again
 *   to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define DMA_PUT_CHAN(dev, chan) (dev)->put_chan(dev, chan)

/****************************************************************************
 * Name: DMA_CONFIG
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

#define DMA_CONFIG(chan, cfg) (chan)->ops->config(chan, cfg)

/****************************************************************************
 * Name: DMA_START
 *
 * Description:
 *   Start the DMA transfer. NOTE: The DMA module does *NOT* perform any
 *   cache operations. It is the responsibility of the DMA client to clean
 *   DMA buffers after staring of the DMA TX operations.
 *
 * Input Parameters:
 *   chan     - The channel to start
 *   callback - The callback when the transfer finish
 *   arg      - The argument will pass to callback
 *   dst      - The destination address
 *   src      - The source address
 *   len      - The length to transfer
 *
 * Returned Value:
 *   The error code
 *
 ****************************************************************************/

#define DMA_START(chan, callback, arg, dst, src, len) \
    (chan)->ops->start(chan, callback, arg, dst, src, len)

/****************************************************************************
 * Name: DMA_START_CYCLIC
 *
 * Description:
 *   Start the cyclic DMA transfer.
 *
 * Note: callback get called for each period length data DMA transfer.
 *
 ****************************************************************************/

#define DMA_START_CYCLIC(chan, callback, arg, dst, src, len, period_len) \
    (chan)->ops->start_cyclic(chan, callback, arg, dst, src, len, period_len)

/****************************************************************************
 * Name: DMA_START_CYCLIC
 *
 * Description:
 *   Start the cyclic DMA transfer.
 *
 * Note: callback get called for each period length data DMA transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_DMA_LINK
#  define DMA_START_LINK(chan, callback, arg, mode, link_cfg) \
    (chan)->ops->start_link(chan, callback, arg, mode, link_cfg)
#endif

/****************************************************************************
 * Name: DMA_PAUSE
 *
 * Description:
 *   Pause the DMA transfer. After DMA_PAUSE() is called, DMA_RESUME() must
 *   be called to restart the transfer again.
 *
 ****************************************************************************/

#define DMA_PAUSE(chan) (chan)->ops->pause(chan)

/****************************************************************************
 * Name: DMA_RESUME
 *
 * Description:
 *   Resume the DMA transfer.
 *
 ****************************************************************************/

#define DMA_RESUME(chan) (chan)->ops->resume(chan)

/****************************************************************************
 * Name: DMA_STOP
 *
 * Description:
 *   Stop the DMA transfer.
 *
 ****************************************************************************/

#define DMA_STOP(chan) (chan)->ops->stop(chan)

/****************************************************************************
 * Name: DMA_RESIDUAL
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred.
 *
 ****************************************************************************/

#define DMA_RESIDUAL(chan) (chan)->ops->residual(chan)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * This is the type of the callback that is used to inform the user of the
 * completion of the DMA. NOTE: The DMA module does *NOT* perform any cache
 * operations. It is the responsibility of the DMA client to invalidate DMA
 * buffers after completion of the DMA RX operations.
 *
 * Input Parameters:
 *   chan - Refers to the DMA channel.
 *   arg  - A user-provided value that was provided when start() was called.
 *   len  - Positive value represent the transfer length, negative is error
 *          code.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

struct dma_chan_s;
typedef CODE void (*dma_callback_t)(FAR struct dma_chan_s *chan,
                                    FAR void *arg, ssize_t len);

/* This struct is passed in as configuration data to a DMA engine
 * in order to set up a certain channel for DMA transport at runtime.
 *
 * The rationale for adding configuration information to this struct is as
 * follows: if it is likely that more than one DMA controllers in the world
 * will support the configuration option, then make it generic. If not: if
 * it is fixed so that it be sent in static from the platform data, then
 * prefer to do that.
 *
 * direction - whether the data shall go in or out on this channel.
 * priority  - the bus priority compare with other active channel.
 * timeout   - abort if the source can't prepare data within this value.
 * dst_width - this is the width in bytes of destination target(TX) register
 *             where DMA data shall be read. If the source is memory this
 *             may be ignored. Legal values: 1, 2, 4, 8.
 * src_width - same as dst_width but for the source(RX) mutatis mutandis.
 *
 * Note: the special value zero means to keep the current value without
 *       change.
 *
 ****************************************************************************/

struct dma_config_s
{
  unsigned int direction;
  unsigned int priority;
  unsigned int timeout;
  unsigned int dst_width;
  unsigned int src_width;
};

#ifdef CONFIG_DMA_LINK
struct dma_link_s
{
  uintptr_t addr;
  union
  {
    unsigned int link_num;
    unsigned int link_size;
  };
};

struct dma_link_config_s
{
  unsigned int dst_link_num;
  unsigned int src_link_num;
  FAR struct dma_link_s *dst_link;
  FAR struct dma_link_s *src_link;
};
#endif

/* The DMA vtable */

struct dma_ops_s
{
  CODE int (*config)(FAR struct dma_chan_s *chan,
                     FAR const struct dma_config_s *cfg);
  CODE int (*start)(FAR struct dma_chan_s *chan,
                    dma_callback_t callback, FAR void *arg,
                    uintptr_t dst, uintptr_t src, size_t len);
  CODE int (*start_cyclic)(FAR struct dma_chan_s *chan,
                           dma_callback_t callback, FAR void *arg,
                           uintptr_t dst, uintptr_t src,
                           size_t len, size_t period_len);
#ifdef CONFIG_DMA_LINK
  CODE int (*start_link)(FAR struct dma_chan_s *chan,
                         dma_callback_t callback, FAR void *arg,
                         unsigned int work_mode,
                         FAR struct dma_link_config_s *cfg);
#endif
  CODE int (*stop)(FAR struct dma_chan_s *chan);
  CODE int (*pause)(FAR struct dma_chan_s *chan);
  CODE int (*resume)(FAR struct dma_chan_s *chan);
  CODE size_t (*residual)(FAR struct dma_chan_s *chan);
};

/* This structure only defines the initial fields of the structure
 * visible to the DMA client. The specific implementation may add
 * additional device specific fields after the vtable.
 */

struct dma_chan_s
{
  FAR const struct dma_ops_s *ops;
};

struct dma_dev_s
{
  CODE FAR struct dma_chan_s *(*get_chan)(FAR struct dma_dev_s *dev,
                                          unsigned int ident);
  CODE void (*put_chan)(FAR struct dma_dev_s *dev,
                        FAR struct dma_chan_s *chan);
};

#endif /* __INCLUDE_NUTTX_DMA_DMA_H */
