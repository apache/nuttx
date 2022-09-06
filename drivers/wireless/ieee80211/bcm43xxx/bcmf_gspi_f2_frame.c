/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_gspi_f2_frame.c
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
#include "bcmf_utils.h"

 #include "bcmf_netdev.h"

#include "bcmf_sdio_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define F2_FRAME_CONTROL_CHANNEL 0  /* Control frame id */
#define F2_FRAME_EVENT_CHANNEL   1  /* Asynchronous event frame id */
#define F2_FRAME_DATA_CHANNEL    2  /* Data frame id */

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct f2_frame_header_s
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

typedef struct f2_frame_header_s f2_frame_header_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: f2_frame_rx_fail
 ****************************************************************************/

static int f2_frame_rx_fail(FAR bcmf_gspi_dev_t *gbus, bool retry)
{
  /* issue abort command for F2 through F0 */

  bcmf_bus_io_abort(gbus);

  bcmf_write_reg(gbus, 1, SBSDIO_FUNC1_FRAMECTRL, SFC_RF_TERM);

  /* TODO Wait until the packet has been flushed (device/FIFO stable) */

  if (retry)
    {
      /* Send NAK to retry to read frame */

      bcmf_write_sbregb(gbus,
                        CORE_BUS_REG(gbus->chip->core_base[SDIOD_CORE_ID],
                                     tosbmailbox),
                        SMB_NAK);
    }

  return OK;
}

/****************************************************************************
 * Name: process_f2_frame_header
 ****************************************************************************/

int process_f2_frame_header(FAR bcmf_interface_dev_t *gbus,
                            f2_frame_header_t        *header)
{
  if (header->data_offset < sizeof(f2_frame_header_t)
      || header->data_offset > header->size)
    {
      wlerr("Invalid data offset\n");
      f2_frame_rx_fail(gbus, false);
      return -ENXIO;
    }

  /* Update tx credits */

  gbus->max_seq = header->credit;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_gspi_read_f2_frame
 *
 * Description:
 *    Read and process an F2 frame.
 *
 * Parameters:
 *    priv         - the device structure
 *    frame_length - Length of frame we are to read.  (From chip status)
 *
 * Returns:
 *    OK on success, negated error code on failure.
 ****************************************************************************/

int bcmf_gspi_read_f2_frame(FAR struct bcmf_dev_s *priv,
                            int                    frame_length)
{
  FAR bcmf_gspi_dev_t *gbus = (FAR bcmf_gspi_dev_t *)priv->bus;
  FAR gspi_dev_t      *gspi = gbus->gspi;

  bcmf_interface_frame_t   *iframe;
  f2_frame_header_t        *header;
  int                       ret;
  uint16_t                  checksum;

  /* Request free frame buffer */

  if (frame_length <= sizeof(f2_frame_header_t))
    {
      return -EINVAL;
    }

  iframe = bcmf_interface_allocate_frame(priv, false, false);

  if (iframe == NULL)
    {
      wlinfo("fail alloc\n");
      return -EAGAIN;
    }

  if (frame_length > iframe->header.len)
    {
      wlerr("Frame is too large, cancel %d > %d\n",
            frame_length,
            iframe->header.len);

      ret = -ENOMEM;
      goto exit_abort;
    }

  /* Read the frame data (the buffer is DMA aligned here) */

  ret = gspi->read(gspi,
                   true,
                   gspi_f2_dma,
                   0,
                   frame_length,
                   (uint32_t *) iframe->data);

  if (ret != OK)
    {
      wlinfo("Failed to read frame data\n");
      ret = -EIO;
      goto exit_abort;
    }

  header = (f2_frame_header_t *)iframe->data;

  if (header->size == 0)
    {
      ret = OK;
      goto exit_free_frame;
    }

  checksum = header->size | header->checksum;

  if (checksum != 0xffff || header->size < sizeof(f2_frame_header_t))
    {
      wlerr("Checksum failed: size:.0x%04X checksum: 0x%04X\n",
            header->size,
            header->checksum);

      bcmf_hexdump((uint8_t *)header, MIN(header->size, 64), 0);

      f2_frame_rx_fail(gbus, false);
      return -EINVAL;
    }

  /* Process and validate header */

  ret = process_f2_frame_header(gbus, header);
  if (ret != OK)
    {
      wlerr("Error while processing header %d\n", ret);
      ret = -EINVAL;
      goto exit_free_frame;
    }

  if (header->size == FC_UPDATE_PKT_LENGTH)
    {
      /* Flow control update packet with no data */

      return OK;
    }

  /* Update frame structure */

  iframe->header.len   = header->size;
  iframe->header.data += header->data_offset;

  /* Process received frame content */

  switch (header->channel & 0x0f)
    {
      case F2_FRAME_CONTROL_CHANNEL:
        ret = bcmf_cdc_process_control_frame(priv, &iframe->header);
        goto exit_free_frame;

      case F2_FRAME_EVENT_CHANNEL:
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

      case F2_FRAME_DATA_CHANNEL:

        /* Queue frame and notify network layer frame is available */

        if (nxmutex_lock(&gbus->queue_lock) < 0)
          {
            DEBUGPANIC();
          }

        list_add_tail(&gbus->rx_queue, &iframe->list_entry);
        nxmutex_unlock(&gbus->queue_lock);

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
  f2_frame_rx_fail(gbus, false);
exit_free_frame:
  bcmf_interface_free_frame(priv, iframe);
  return ret;
}

/****************************************************************************
 * Name: bcmf_gspi_send_f2_frame
 *
 * Description:
 *    De-queue and send an F2 frame.
 *
 * Parameters:
 *    priv     - the device structure
 *
 * Returns:
 *    OK on success, negated error code on failure.
 ****************************************************************************/

int bcmf_gspi_send_f2_frame(FAR struct bcmf_dev_s *priv)
{
  FAR bcmf_gspi_dev_t      *gbus = (FAR bcmf_gspi_dev_t *)priv->bus;
  FAR gspi_dev_t           *gspi = gbus->gspi;
  f2_frame_header_t       *header;
  bcmf_interface_frame_t  *iframe;
  int                      ret;
  bool                     is_txframe;

  if (list_is_empty(&gbus->tx_queue))
    {
      /* No more frames to send */

      return -ENODATA;
    }

  if (gbus->tx_seq == gbus->max_seq)
    {
      /* TODO handle this case */

      wlerr("No credit to send frame\n");
      return -EAGAIN;
    }

  if (nxmutex_lock(&gbus->queue_lock) < 0)
    {
      DEBUGPANIC();
    }

  iframe = list_remove_head_type(&gbus->tx_queue,
                                 bcmf_interface_frame_t,
                                 list_entry);

  nxmutex_unlock(&gbus->queue_lock);

  is_txframe = iframe->tx;

  header = (f2_frame_header_t *)iframe->header.base;

  /* Set frame sequence id */

  header->sequence = gbus->tx_seq++;

#if 0
  wlinfo("\n>>>---> Send frame %p %d\n", iframe, iframe->header.len);

  wlinfo("size:%d  seq: %d, channel: %d  next len: %d\n",
         header->size,
         header->sequence,
         header->channel,
         header->next_length);

  wlinfo("data offset:0x%02X  flow: %d, credit: %d\n",
          header->data_offset,
          header->flow_control,
          header->credit);
#endif

#if 0
  bcmf_hexdump(iframe->header.base, iframe->header.len,
               (unsigned long)iframe->header.base);
#endif

  /* Write the frame data (the buffer is DMA aligned here) */

  ret = gspi->write(gspi,
                    true,
                    gspi_f2_dma,
                    0,
                    iframe->header.len,
                    (FAR uint32_t *) iframe->header.base);

  /* Free frame buffer */

  bcmf_interface_free_frame(priv, iframe);

  if (ret == OK && is_txframe)
    {
      /* Notify upper layer at least one TX buffer is available */

      bcmf_netdev_notify_tx(priv);
    }

  return ret;
}
