/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_sdpcm.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <stddef.h>
#include <string.h>
#include <queue.h>
#include <semaphore.h>

#include "bcmf_sdio.h"
#include "bcmf_core.h"
#include "bcmf_sdpcm.h"
#include "bcmf_ioctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDA_FRAMECTRL */
#define SFC_RF_TERM  (1 << 0)  /* Read Frame Terminate */
#define SFC_WF_TERM  (1 << 1)  /* Write Frame Terminate */
#define SFC_CRC4WOOS (1 << 2)  /* CRC error for write out of sync */
#define SFC_ABORTALL (1 << 3)  /* Abort all in-progress frames */

/* tosbmailbox bits corresponding to intstatus bits */
#define SMB_NAK     (1 << 0)  /* Frame NAK */
#define SMB_INT_ACK (1 << 1)  /* Host Interrupt ACK */
#define SMB_USE_OOB (1 << 2)  /* Use OOB Wakeup */
#define SMB_DEV_INT (1 << 3)  /* Miscellaneous Interrupt */

/* CDC flag definitions */
#define CDC_DCMD_ERROR    0x01       /* 1=cmd failed */
#define CDC_DCMD_SET      0x02       /* 0=get, 1=set cmd */
#define CDC_DCMD_IF_MASK  0xF000     /* I/F index */
#define CDC_DCMD_IF_SHIFT 12
#define CDC_DCMD_ID_MASK  0xFFFF0000 /* id an cmd pairing */
#define CDC_DCMD_ID_SHIFT 16         /* ID Mask shift bits */
#define CDC_DCMD_ID(flags)  \
  (((flags) & CDC_DCMD_ID_MASK) >> CDC_DCMD_ID_SHIFT)

#define SDPCM_CONTROL_CHANNEL 0  /* Control */
#define SDPCM_EVENT_CHANNEL   1  /* Asyc Event Indication */
#define SDPCM_DATA_CHANNEL    2  /* Data Xmit/Recv */
#define SDPCM_GLOM_CHANNEL    3  /* Coalesced packets */
#define SDPCM_TEST_CHANNEL    15 /* Test/debug packets */

#define SDPCM_CONTROL_TIMEOUT_MS 1000

// TODO remove
void bcmf_hexdump(uint8_t *data, unsigned int len, unsigned long offset);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcmf_sdpcm_header {
    uint16_t size;
    uint16_t checksum;
    uint8_t  sequence;
    uint8_t  channel;
    uint8_t  next_length;
    uint8_t  data_offset;
    uint8_t  flow_control;
    uint8_t  credit;
    uint16_t padding;
};

struct bcmf_sdpcm_cdc_header {
    uint32_t cmd;    /* dongle command value */
    uint32_t len;    /* lower 16: output buflen;
                      * upper 16: input buflen (excludes header) */
    uint32_t flags;  /* flag defns given below */
    uint32_t status; /* status code returned from the device */
};

struct bcmf_sdpcm_cdc_dcmd {
    struct bcmf_sdpcm_header     header;
    struct bcmf_sdpcm_cdc_header cdc_header;
    uint8_t                      data[0];
};

struct bcmf_sdpcm_frame {
    dq_entry_t                   list_entry;
    struct bcmf_sdpcm_header     header;
    uint8_t                      data[0];
};

struct bcmf_sdpcm_cdc_frame {
    dq_entry_t                   list_entry;
    struct bcmf_sdpcm_header     header;
    struct bcmf_sdpcm_cdc_header cdc_header;
    uint8_t                      data[0];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcmf_sdpcm_rxfail(FAR struct bcmf_dev_s *priv, bool retry);

static int bcmf_sdpcm_process_header(FAR struct bcmf_dev_s *priv,
                              struct bcmf_sdpcm_header *header);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_sdpcm_rxfail(FAR struct bcmf_dev_s *priv, bool retry)
{
  /* issue abort command for F2 through F0 */

  bcmf_write_reg(priv, 0, SDIO_CCCR_IOABORT, 2);

  bcmf_write_reg(priv, 1, SBSDIO_FUNC1_FRAMECTRL, SFC_RF_TERM);

  /* TODO Wait until the packet has been flushed (device/FIFO stable) */

  /* Send NAK to retry to read frame */
  if (retry)
    {
      bcmf_write_sbregb(priv,
                  CORE_BUS_REG(priv->get_core_base_address(SDIOD_CORE_ID),
                  tosbmailbox), SMB_NAK);
    }

  return 0;
}

int bcmf_sdpcm_process_header(FAR struct bcmf_dev_s *priv,
                              struct bcmf_sdpcm_header *header)
{
  if (header->data_offset < sizeof(struct bcmf_sdpcm_header) ||
      header->data_offset > header->size)
    {
      _err("Invalid data offset\n");
      bcmf_sdpcm_rxfail(priv, false);
      return -ENXIO;
    }

  /* Update tx credits */

  _info("update credit %x %x %x\n", header->credit,
                                    priv->tx_seq, priv->max_seq);

  if (header->credit - priv->tx_seq > 0x40)
    {
      _err("seq %d: max tx seq number error\n", priv->tx_seq);
      priv->max_seq = priv->tx_seq + 2;
    }
  else
    {
      priv->max_seq = header->credit;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

// FIXME remove
uint8_t tmp_buffer[512];
int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint16_t len, checksum;
  struct bcmf_sdpcm_header *header = (struct bcmf_sdpcm_header*)tmp_buffer;

  /* Read header */

  ret = bcmf_transfer_bytes(priv, false, 2, 0, (uint8_t*)header, 4);
  if (ret != OK)
    {
      _info("failread size\n");
      ret = -EIO;
      goto exit_abort;
    }

  len = header->size;
  checksum = header->checksum;

  /* All zero means no more to read */

  if (!(len | checksum))
    {
      return -ENODATA;
    }

  if (((~len & 0xffff) ^ checksum) || len < sizeof(struct bcmf_sdpcm_header))
    {
      _err("Invalid header checksum or len %x %x\n", len, checksum);
      ret = -EINVAL;
      goto exit_abort;
    }

  // FIXME define for size
  if (len > 512)
    {
      _err("Frame is too large, cancel %d\n", len);
      ret = -ENOMEM;
      goto exit_abort;
    }

  /* Read remaining frame data */
  // TODO allocate buffer

  ret = bcmf_transfer_bytes(priv, false, 2, 0, (uint8_t*)header+4, len - 4);
  if (ret != OK)
    {
      ret = -EIO;
      goto exit_free_abort;
    }

  _info("Receive frame\n");
  bcmf_hexdump((uint8_t*)header, len, (unsigned int)header);

  /* Process and validate header */

  ret = bcmf_sdpcm_process_header(priv, header);
  if (ret != OK)
    {
      _err("Error while processing header %d\n", ret);
      ret = -EINVAL;
      goto exit_free_frame;
    }

  /* Process received frame content */

  switch (header->channel & 0x0f)
    {
      case SDPCM_CONTROL_CHANNEL:
        _info("Control message\n");

        /* Check frame */

        if (header->size < sizeof(struct bcmf_sdpcm_header) + 
                           sizeof(struct bcmf_sdpcm_cdc_header))
          {
            _err("Control frame too small\n");
            ret = -EINVAL;
            goto exit_free_frame;

          }

        struct bcmf_sdpcm_cdc_header *cdc_header =
                (struct bcmf_sdpcm_cdc_header*)&header[1];

        if (header->size < sizeof(struct bcmf_sdpcm_header) + 
                           sizeof(struct bcmf_sdpcm_cdc_header) +
                           cdc_header->len ||
            cdc_header->len > 512 -
                              sizeof(struct bcmf_sdpcm_header) -
                              sizeof(struct bcmf_sdpcm_cdc_header))
          {
            _err("Invalid control frame size\n");
            ret = -EINVAL;
            goto exit_free_frame;
          }

        // TODO check interface ?

        if (cdc_header->flags >> CDC_DCMD_ID_SHIFT == priv->control_reqid)
          {
            /* Expected frame received, send it back to user */

            priv->control_rxframe = (uint8_t*)header;

            sem_post(&priv->control_timeout);
            return OK;
          }
        else
          {
            _info("Got unexpected control frame\n");
            ret = -EINVAL;
            goto exit_free_frame;
          }
        break;

      case SDPCM_EVENT_CHANNEL:
        _info("Event message\n");
        ret = OK;
        break;

      case SDPCM_DATA_CHANNEL:
        _info("Data message\n");
        ret = OK;
        break;

      default:
        _err("Got unexpected message type %d\n", header->channel);
        ret = OK;
    }

exit_free_frame:
  // TODO free frame buffer
  return ret;

exit_free_abort:
  // TODO free frame buffer
exit_abort:
  bcmf_sdpcm_rxfail(priv, false);
  return ret;
}

int bcmf_sdpcm_sendframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  struct bcmf_sdpcm_frame *frame;

  if (priv->tx_queue.tail == NULL)
    {
      /* No more frames to send */

      return -ENODATA;
    }

  if (priv->tx_seq == priv->max_seq)
    {
      // TODO handle this case
      _err("No credit to send frame\n");
      return -EAGAIN;
    }


  if ((ret = sem_wait(&priv->tx_queue_mutex)) != OK)
    {
      return ret;
    }

  frame = (struct bcmf_sdpcm_frame*)priv->tx_queue.tail;

  /* Set frame sequence id */

  frame->header.sequence = priv->tx_seq++;

  _info("Send frame\n");
  bcmf_hexdump((uint8_t*)&frame->header, frame->header.size,
               (unsigned int)&frame->header);

  ret = bcmf_transfer_bytes(priv, true, 2, 0, (uint8_t*)&frame->header,
                            frame->header.size);
  if (ret != OK)
    {
      _info("fail send frame %d\n", ret);
      ret = -EIO;
      goto exit_abort;
      // TODO handle retry count and remove frame from queue + abort TX
    }

  /* Frame sent, remove it from queue */

  if (priv->tx_queue.head == &frame->list_entry)
    {
      /* List is empty */

      priv->tx_queue.head = NULL;
      priv->tx_queue.tail = NULL;
    }
  else
    {
      priv->tx_queue.tail = frame->list_entry.blink;
      frame->list_entry.blink->flink = priv->tx_queue.head;
    }

  /* TODO free frame buffer */

  goto exit_post_sem;

exit_abort:
  // bcmf_sdpcm_txfail(priv, false);
exit_post_sem:
  sem_post(&priv->tx_queue_mutex);
  return ret;
}

// FIXME remove
uint8_t tmp_buffer2[512];
uint8_t* bcmf_sdpcm_allocate_iovar(FAR struct bcmf_dev_s *priv, char *name,
                              char *data, uint32_t *len)
{
  uint32_t data_len;
  uint16_t name_len = strlen(name) + 1;

  if (!data)
    {
      data_len = 0;
    }
  else
   {
      data_len = *len;
   }

  // FIXME allocate buffer and use max_size instead of 512
  if (data_len > 512-sizeof(struct bcmf_sdpcm_cdc_frame) ||
      (data_len + name_len) > 512-sizeof(struct bcmf_sdpcm_cdc_frame))
    {
      *len = 0;
      return NULL;
    }

  // TODO allocate buffer

  /* Copy name string and data */

  memcpy(tmp_buffer2+sizeof(struct bcmf_sdpcm_cdc_frame), name, name_len);
  memcpy(tmp_buffer2+sizeof(struct bcmf_sdpcm_cdc_frame)+name_len,
         data, data_len);

  *len = sizeof(struct bcmf_sdpcm_cdc_frame)+name_len+data_len;
  return tmp_buffer2;
}

int bcmf_sdpcm_queue_frame(FAR struct bcmf_dev_s *priv, uint8_t channel,
                           uint8_t *data, uint32_t len)
{
  int ret;
  struct bcmf_sdpcm_frame *frame = (struct bcmf_sdpcm_frame*)data;
  uint16_t frame_size = len - sizeof(frame->list_entry);

  /* Prepare sw header */

  memset(&frame->header, 0, sizeof(struct bcmf_sdpcm_header));
  frame->header.size = frame_size;
  frame->header.checksum = ~frame_size;
  frame->header.channel = channel;
  frame->header.data_offset = sizeof(struct bcmf_sdpcm_header);

  /* Add frame in tx queue */

  if ((ret = sem_wait(&priv->tx_queue_mutex)) != OK)
    {
      return ret;
    }

  if (priv->tx_queue.head == NULL)
    {
      /* List is empty */

      priv->tx_queue.head = &frame->list_entry;
      priv->tx_queue.tail = &frame->list_entry;

      frame->list_entry.flink = &frame->list_entry;
      frame->list_entry.blink = &frame->list_entry;
    }
  else
    {
      /* Insert entry at list head */

      frame->list_entry.flink = priv->tx_queue.head;
      frame->list_entry.blink = priv->tx_queue.tail;

      priv->tx_queue.head->blink = &frame->list_entry;
      priv->tx_queue.head = &frame->list_entry;
    }

  sem_post(&priv->tx_queue_mutex);

  /* Notify bcmf thread tx frame is ready */

  sem_post(&priv->thread_signal);

  return OK;
}

int bcmf_sdpcm_send_cdc_frame(FAR struct bcmf_dev_s *priv, uint32_t cmd,
                         int ifidx, bool set, uint8_t *data, uint32_t len)
{
  struct bcmf_sdpcm_cdc_frame *msg = (struct bcmf_sdpcm_cdc_frame*)data;

  /* Setup cdc_dcmd header */

  msg->cdc_header.cmd = cmd;
  msg->cdc_header.len = len-sizeof(struct bcmf_sdpcm_cdc_frame);
  msg->cdc_header.status = 0;
  msg->cdc_header.flags = ++priv->control_reqid << CDC_DCMD_ID_SHIFT;
  msg->cdc_header.flags |= ifidx << CDC_DCMD_IF_SHIFT;

  if (set)
  {
    msg->cdc_header.flags |= CDC_DCMD_SET;
  }

  /* Queue frame */

  return bcmf_sdpcm_queue_frame(priv, SDPCM_CONTROL_CHANNEL, data, len);
}

int bcmf_sdpcm_iovar_request(FAR struct bcmf_dev_s *priv,
                             uint32_t ifidx, bool set, char *name,
                             char *data, uint32_t *len)
{
  int ret;
  uint8_t *iovar_buf;
  uint32_t iovar_buf_len = *len;

  *len = 0;

  /* Take device control mutex */

  if ((ret = sem_wait(&priv->control_mutex)) !=OK)
   {
      return ret;
   }

  /* Prepare control frame */

  iovar_buf = bcmf_sdpcm_allocate_iovar(priv, name, data, &iovar_buf_len);
  if (!iovar_buf)
    {
      _err("Cannot allocate iovar buf\n");
      ret = -ENOMEM;
      goto exit_sem_post;
    }

  /* Send control frame. Frame buffer is freed when sent */

  ret = bcmf_sdpcm_send_cdc_frame(priv, set ? WLC_SET_VAR : WLC_GET_VAR,
                             ifidx, set, iovar_buf, iovar_buf_len);
  if (ret != OK)
    {
       goto exit_free_iovar;
    }

  /* Wait for response */

  priv->control_rxframe = NULL;

  ret = bcmf_sem_wait(&priv->control_timeout, SDPCM_CONTROL_TIMEOUT_MS);
  if (ret != OK)
    {
      _err("Error while waiting for control response %d\n", ret);
      goto exit_sem_post;
    }

  if (!set)
    {
      /* Request sent, copy received data back */

      struct bcmf_sdpcm_cdc_dcmd *rxframe =
              (struct bcmf_sdpcm_cdc_dcmd*)priv->control_rxframe;

      memcpy(data, rxframe->data, rxframe->cdc_header.len);

      *len = rxframe->cdc_header.len;
    }

  // TODO free rxframe buffer */

  goto exit_sem_post;

exit_free_iovar:
  // TODO free allocated buffer here
exit_sem_post:
  sem_post(&priv->control_mutex);
  return ret;
}