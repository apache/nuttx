/****************************************************************************
 * drivers/wireless/bcm43xxx/ieee80211/bcmf_bdc.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
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
#include <string.h>

#include <net/ethernet.h>

#include "bcmf_driver.h"
#include "bcmf_ioctl.h"
#include "bcmf_cdc.h"
#include "bcmf_bdc.h"
#include "bcmf_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCMF_EVENT_ETHER_TYPE 0x6C88 /* Ether type of event frames */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct __attribute__((packed)) bcmf_bdc_header
{
  uint8_t flags;       /* bdc frame flags */
  uint8_t priority;    /* bdc frame priority */
  uint8_t flags2;      /* bdc frame additional flags */
  uint8_t data_offset; /* Offset from end of header to payload data, in 4-bytes count */
};

struct __attribute__((packed)) bcmf_eth_header
{
  uint16_t type;     /* Vendor specific type */
  uint16_t len;      /* Event data length */
  uint8_t  version;  /* Protocol version */
  uint8_t  oui[3];   /* Organizationally unique identifier */
  uint16_t usr_type; /* User specific type */
};

struct __attribute__((packed)) bcmf_event_msg
{
  struct ether_header    eth;
  struct bcmf_eth_header bcm_eth;
  struct bcmf_event_s    event;
  uint8_t                data[0];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t bcmf_broadcom_oui[] =
{
  0x00, 0x10, 0x18
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct bcmf_frame_s *bcmf_bdc_allocate_frame(FAR struct bcmf_dev_s *priv,
                                             uint32_t len, bool block)
{
  struct bcmf_frame_s *frame;

  /* Allocate data frame */

  /* TODO check for integer overflow */

  frame = priv->bus->allocate_frame(priv,
                sizeof(struct bcmf_bdc_header) + len, block, false);

  if (!frame)
    {
      return NULL;
    }

  frame->data += sizeof(struct bcmf_bdc_header);

  return frame;
}

int bcmf_bdc_process_event_frame(FAR struct bcmf_dev_s *priv,
                                 struct bcmf_frame_s *frame)
{
  int data_size;
  struct bcmf_bdc_header *header;
  struct bcmf_event_msg *event_msg;
  uint32_t event_id;
  event_handler_t handler;

  /* Check frame header */

  data_size = frame->len - (int)(frame->data - frame->base);

  if (data_size < sizeof(struct bcmf_bdc_header))
    {
      goto exit_invalid_frame;
    }

  header = (struct bcmf_bdc_header *)frame->data;

  data_size -= sizeof(struct bcmf_bdc_header) + header->data_offset * 4;

  if (data_size < sizeof(struct bcmf_event_msg))
    {
      goto exit_invalid_frame;
    }

  data_size -= sizeof(struct ether_header) + sizeof(struct bcmf_eth_header);

  /* Check ethernet header */

  event_msg = (struct bcmf_event_msg *)(frame->data +
                                        sizeof(struct bcmf_bdc_header) +
                                        header->data_offset * 4);

  if (event_msg->eth.ether_type != BCMF_EVENT_ETHER_TYPE ||
      memcmp(event_msg->bcm_eth.oui, bcmf_broadcom_oui, 3))
    {
      goto exit_invalid_frame;
    }

  event_id = bcmf_getle32(&event_msg->event.type);

  if (event_id >= BCMF_EVENT_COUNT)
    {
      wlinfo("Invalid event id %d\n", event_id);
      return -EINVAL;
    }

  /* Dispatch event to registered handler */

  handler = priv->event_handlers[event_id];
  if (handler != NULL)
    {
      handler(priv, &event_msg->event, data_size);
    }

  return OK;

exit_invalid_frame:
  wlerr("Invalid event frame\n");
  bcmf_hexdump(frame->base, frame->len, (unsigned long)frame->base);
  return -EINVAL;
}

int bcmf_event_register(FAR struct bcmf_dev_s *priv, event_handler_t handler,
                        unsigned int event_id)
{
  if (event_id >= BCMF_EVENT_COUNT)
    {
      /* Invalid event id */

      return -EINVAL;
    }

  priv->event_handlers[event_id] = handler;
  return OK;
}

int bcmf_event_unregister(FAR struct bcmf_dev_s *priv,
                          unsigned int event_id)
{
  return bcmf_event_register(priv, NULL, event_id);
}

int bcmf_event_push_config(FAR struct bcmf_dev_s *priv)
{
  int i;
  uint32_t out_len;
  uint8_t event_mask[(BCMF_EVENT_COUNT + 7) >> 3];

  memset(event_mask, 0, sizeof(event_mask));

  for (i = 0; i < BCMF_EVENT_COUNT; i++)
    {
      if (priv->event_handlers[i] != NULL)
        {
          event_mask[i >> 3] |= 1 << (i & 0x7);
        }
    }

  /* Send event mask to chip */

  out_len = sizeof(event_mask);
  if (bcmf_cdc_iovar_request(priv, CHIP_STA_INTERFACE, true,
                                 IOVAR_STR_EVENT_MSGS, event_mask,
                                 &out_len))
    {
      return -EIO;
    }

  return OK;
}

int bcmf_bdc_transmit_frame(FAR struct bcmf_dev_s *priv,
                            struct bcmf_frame_s *frame)
{
  struct bcmf_bdc_header *header;

  /* Set frame data for lower layer */

  frame->data -= sizeof(struct bcmf_bdc_header);
  header = (struct bcmf_bdc_header *)frame->data;

  /* Setup data frame header */

  header->flags       = 0x20; /* Set bdc protocol version */
  header->priority    = 0;    /* TODO handle priority */
  header->flags2      = CHIP_STA_INTERFACE;
  header->data_offset = 0;

  /* Send frame */

  return priv->bus->txframe(priv, frame, false);
}

struct bcmf_frame_s *bcmf_bdc_rx_frame(FAR struct bcmf_dev_s *priv)
{
  unsigned int frame_len;
  struct bcmf_frame_s *frame = priv->bus->rxframe(priv);
  struct bcmf_bdc_header *header;

  /* Very that there is an Rx frame */

  if (frame == NULL)
    {
      return NULL;
    }

  /* Process bdc header */

  frame_len = frame->len - (unsigned int)(frame->data - frame->base);

  if (frame_len < sizeof(struct bcmf_bdc_header))
    {
      wlerr("Data frame too small\n");
      priv->bus->free_frame(priv, frame);
      return NULL;
    }

  /* Transmit frame to upper layer */

  header       = (struct bcmf_bdc_header *)frame->data;
  frame->data += sizeof(struct bcmf_bdc_header) + header->data_offset * 4;
  return frame;
}
