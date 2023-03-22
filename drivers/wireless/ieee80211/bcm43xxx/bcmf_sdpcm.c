/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_sdpcm.c
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
#include <nuttx/compiler.h>

#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>

#include <stddef.h>
#include <string.h>

#include "bcmf_core.h"
#include "bcmf_sdpcm.h"
#include "bcmf_cdc.h"
#include "bcmf_bdc.h"
#include "bcmf_interface.h"
#include "bcmf_utils.h"

 #include "bcmf_netdev.h"

#include "bcmf_sdio_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SDPCM_CONTROL_CHANNEL 0  /* Control frame id */
#define SDPCM_EVENT_CHANNEL   1  /* Asynchronous event frame id */
#define SDPCM_DATA_CHANNEL    2  /* Data frame id */

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct bcmf_sdpcm_header
{
  uint16_t size;
  uint16_t checksum;
  uint8_t  sequence;
  uint8_t  channel;
  uint8_t  next_length;
  uint8_t  data_offset;
  uint8_t  flow_control;
  uint8_t  credit;
  uint16_t padding;
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcmf_sdpcm_rxfail(FAR bcmf_interface_dev_t *ibus, bool retry);

static int bcmf_sdpcm_process_header(FAR bcmf_interface_dev_t *ibus,
                              struct bcmf_sdpcm_header *header);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_sdpcm_rxfail(FAR bcmf_interface_dev_t *ibus, bool retry)
{
  /* issue abort command for F2 through F0 */

  bcmf_bus_io_abort(ibus);

  bcmf_write_reg(ibus, 1, SBSDIO_FUNC1_FRAMECTRL, SFC_RF_TERM);

  /* TODO Wait until the packet has been flushed (device/FIFO stable) */

  if (retry)
    {
      /* Send NAK to retry to read frame */

      bcmf_write_sbregb(ibus,
                  CORE_BUS_REG(ibus->chip->core_base[SDIOD_CORE_ID],
                  tosbmailbox), SMB_NAK);
    }

  return 0;
}

int bcmf_sdpcm_process_header(FAR bcmf_interface_dev_t *ibus,
                              struct bcmf_sdpcm_header *header)
{
  if (header->data_offset < sizeof(struct bcmf_sdpcm_header) ||
      header->data_offset > header->size)
    {
      wlerr("Invalid data offset\n");
      bcmf_sdpcm_rxfail(ibus, false);
      return -ENXIO;
    }

  /* Update tx credits */

  ibus->max_seq = header->credit;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint16_t len;
  uint16_t checksum;
  struct bcmf_sdpcm_header *header;
  struct bcmf_sdpcm_header tmp_hdr;
  bcmf_interface_frame_t *iframe;
  FAR bcmf_interface_dev_t *ibus = (FAR bcmf_interface_dev_t *)priv->bus;

  /* Read the first 4 bytes of sdpcm header
   * to get the length of the following data to be read
   */

  ret = bcmf_transfer_bytes(ibus, false, 2, 0,
                            (uint8_t *)&tmp_hdr,
                            FIRST_WORD_SIZE);
  if (ret != OK)
    {
      wlinfo("Failed to read size\n");
      bcmf_sdpcm_rxfail(ibus, false);
      return -EIO;
    }

  len = tmp_hdr.size;
  checksum = tmp_hdr.checksum;

  /* All zero means no more to read */

  if (len == 0)
    {
      wlinfo("No data\n");

      return -ENODATA;
    }

  wlinfo("len: %d Header checksum: 0x%04x\n", len, checksum);

  if (((~len & 0xffff) ^ checksum) || len < sizeof(struct bcmf_sdpcm_header))
    {
      wlerr("Invalid header checksum or len %d 0x%04x\n", len, checksum);
      bcmf_sdpcm_rxfail(ibus, false);
      return -EINVAL;
    }

  if (len == FC_UPDATE_PKT_LENGTH)
    {
      /* Flow control update packet with no data */

      wlinfo("Flow control\n");

      ret = bcmf_transfer_bytes(ibus, false, 2, 0,
                                (uint8_t *)&tmp_hdr + FIRST_WORD_SIZE,
                                FC_UPDATE_PKT_LENGTH - FIRST_WORD_SIZE);
      if (ret != OK)
        {
          wlinfo("Failed to read the rest 8 bytes\n");
          bcmf_sdpcm_rxfail(ibus, false);
          return -EIO;
        }

      ret = bcmf_sdpcm_process_header(ibus, &tmp_hdr);

      if (ret != OK)
        {
          wlerr("Error while processing header %d\n", ret);
          return -EINVAL;
        }

      return OK;
    }

  /* Request free frame buffer */

  iframe = bcmf_interface_allocate_frame(priv, false, false);

  if (iframe == NULL)
    {
      wlinfo("fail alloc\n");

      /* Read out the rest of the header to get the bus credit information */

      ret = bcmf_transfer_bytes(ibus, false, 2, 0,
                                (uint8_t *)&tmp_hdr + FIRST_WORD_SIZE,
                                FC_UPDATE_PKT_LENGTH - FIRST_WORD_SIZE);
      if (ret != OK)
        {
          wlinfo("Failed to read the rest 8 bytes\n");
          bcmf_sdpcm_rxfail(ibus, false);
          return -EIO;
        }

      bcmf_sdpcm_rxfail(ibus, false);

      ret = bcmf_sdpcm_process_header(ibus, &tmp_hdr);

      if (ret != OK)
        {
          wlerr("Error while processing header %d\n", ret);
          return -EINVAL;
        }

      return -EAGAIN;
    }

  header = (struct bcmf_sdpcm_header *)iframe->data;

  /* Read the remaining frame data (the buffer is DMA aligned here) */

  if (len <= FIRST_WORD_SIZE)
    {
      ret = OK;
      goto exit_free_frame;
    }

  ret = bcmf_transfer_bytes(ibus, false, 2, 0,
                            (uint8_t *)header + FIRST_WORD_SIZE,
                            len - FIRST_WORD_SIZE);
  if (ret != OK)
    {
      wlinfo("Failed to read remaining frame data\n");
      ret = -EIO;
      goto exit_abort;
    }

  memcpy(header, &tmp_hdr, FIRST_WORD_SIZE);

  if (len > iframe->header.len)
    {
      wlerr("Frame is too large, cancel %d %d\n", len, iframe->header.len);
      ret = -ENOMEM;
      goto exit_abort;
    }

#if 1
  wlinfo("Receive frame %p %d\n", iframe, len);

  wlinfo("size:%d  seq: %d, channel: %d  next len: %d\n",
         header->size,
         header->sequence,
         header->channel,
         header->next_length);

  wlinfo("data offset:0x%02X  flow: %d, credit: %d\n",
          header->data_offset,
          header->flow_control,
          header->credit);

  bcmf_hexdump((uint8_t *)header, header->size, (unsigned int)header);
#endif

  /* Process and validate header */

  ret = bcmf_sdpcm_process_header(ibus, header);
  if (ret != OK)
    {
      wlerr("Error while processing header %d\n", ret);
      ret = -EINVAL;
      goto exit_free_frame;
    }

  /* Update frame structure */

  iframe->header.len = header->size;
  iframe->header.data += header->data_offset;

  /* Process received frame content */

  switch (header->channel & 0x0f)
    {
      case SDPCM_CONTROL_CHANNEL:
        ret = bcmf_cdc_process_control_frame(priv, &iframe->header);
        goto exit_free_frame;

      case SDPCM_EVENT_CHANNEL:
        if (header->data_offset == header->size)
          {
            /* Empty event, ignore */

            ret = OK;
          }
        else
          {
            ret = bcmf_bdc_process_event_frame(priv, &iframe->header);
          }

        goto exit_free_frame;

      case SDPCM_DATA_CHANNEL:

        /* Queue frame and notify network layer frame is available */

        if (nxmutex_lock(&ibus->queue_lock) < 0)
          {
            DEBUGPANIC();
          }

        list_add_tail(&ibus->rx_queue, &iframe->list_entry);
        nxmutex_unlock(&ibus->queue_lock);

        bcmf_netdev_notify_rx(priv);

        /* Upper layer have to free all received frames */

        ret = OK;
        break;

      default:
        wlerr("Got unexpected message type %d\n", header->channel);
        ret = -EINVAL;
        goto exit_free_frame;
    }

  return ret;

exit_abort:
  bcmf_sdpcm_rxfail(ibus, false);
exit_free_frame:
  bcmf_interface_free_frame(priv, iframe);
  return ret;
}

int bcmf_sdpcm_sendframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  bool is_txframe;
  bcmf_interface_frame_t *iframe;
  struct bcmf_sdpcm_header *header;
  FAR bcmf_interface_dev_t *ibus = (FAR bcmf_interface_dev_t *)priv->bus;

  if (list_is_empty(&ibus->tx_queue))
    {
      /* No more frames to send */

      return -ENODATA;
    }

  if (ibus->tx_seq == ibus->max_seq)
    {
      /* TODO handle this case */

      wlwarn("No credit to send frame\n");
      return -EAGAIN;
    }

  if (nxmutex_lock(&ibus->queue_lock) < 0)
    {
      DEBUGPANIC();
    }

  iframe = list_remove_head_type(&ibus->tx_queue, bcmf_interface_frame_t,
                                 list_entry);
  nxmutex_unlock(&ibus->queue_lock);

  header = (struct bcmf_sdpcm_header *)iframe->header.base;

  /* Set frame sequence id */

  header->sequence = ibus->tx_seq++;

#if 1
  wlinfo("Send frame %p\n", iframe);

  bcmf_hexdump(iframe->header.base, iframe->header.len,
               (unsigned long)iframe->header.base);
#endif

  /* Write the frame data (the buffer is DMA aligned here) */

  ret = bcmf_transfer_bytes(ibus, true, 2, 0,
                            iframe->header.base,
                            iframe->header.len);
  is_txframe = iframe->tx;

  /* Free frame buffer */

  bcmf_interface_free_frame(priv, iframe);

  if (ret == OK && is_txframe)
    {
      /* Notify upper layer at least one TX buffer is available */

      bcmf_netdev_notify_tx(priv);
    }

  wlinfo("return %d\n", ret);

  return ret;
}

int bcmf_sdpcm_queue_frame(FAR struct bcmf_dev_s *priv,
                           struct bcmf_frame_s *frame, bool control)
{
  FAR bcmf_interface_dev_t *ibus = (FAR bcmf_interface_dev_t *)priv->bus;
  bcmf_interface_frame_t *iframe = (bcmf_interface_frame_t *)frame;
  struct bcmf_sdpcm_header *header =
    (struct bcmf_sdpcm_header *)iframe->data;
  int semcount;

  /* Prepare sw header */

  memset(header, 0, sizeof(struct bcmf_sdpcm_header));
  header->size = frame->len;
  header->checksum = ~header->size;
  header->data_offset = (uint8_t)(frame->data - frame->base);

  if (control)
    {
      header->channel = SDPCM_CONTROL_CHANNEL;
    }
  else
    {
      header->channel = SDPCM_DATA_CHANNEL;
    }

  /* Add frame in tx queue */

  if (nxmutex_lock(&ibus->queue_lock) < 0)
    {
      DEBUGPANIC();
    }

  list_add_tail(&ibus->tx_queue, &iframe->list_entry);

  nxmutex_unlock(&ibus->queue_lock);

  /* Notify bcmf thread tx frame is ready */

  nxsem_get_value(&ibus->thread_signal, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&ibus->thread_signal);
    }

  return OK;
}

struct bcmf_frame_s *bcmf_sdpcm_alloc_frame(FAR struct bcmf_dev_s *priv,
                                            unsigned int len, bool block,
                                            bool control)
{
  bcmf_interface_frame_t *iframe;
  unsigned int header_len = sizeof(struct bcmf_sdpcm_header);

  if (!control)
    {
      header_len += 2; /* Data frames need alignment padding */
    }

  if (len + header_len > MAX_NETDEV_PKTSIZE + HEADER_SIZE)
    {
      wlerr("Invalid size %d\n", len);
      return NULL;
    }

  /* Allocate a frame for RX in case of control frame */

  iframe = bcmf_interface_allocate_frame(priv, block, !control);

  if (iframe == NULL)
    {
      return NULL;
    }

  iframe->header.len   = header_len + len;
  iframe->header.data += header_len;
  return &iframe->header;
}

void bcmf_sdpcm_free_frame(FAR struct bcmf_dev_s *priv,
                           struct bcmf_frame_s *frame)
{
  bcmf_interface_free_frame(priv, (bcmf_interface_frame_t *)frame);
}

struct bcmf_frame_s *bcmf_sdpcm_get_rx_frame(FAR struct bcmf_dev_s *priv)
{
  bcmf_interface_frame_t *iframe;
  FAR bcmf_interface_dev_t *ibus = (FAR bcmf_interface_dev_t *)priv->bus;

  if (nxmutex_lock(&ibus->queue_lock) < 0)
    {
      DEBUGPANIC();
    }

  iframe = list_remove_head_type(&ibus->rx_queue,
                                 bcmf_interface_frame_t,
                                 list_entry);

  nxmutex_unlock(&ibus->queue_lock);

  if (iframe == NULL)
    {
      return NULL;
    }

  return &iframe->header;
}
