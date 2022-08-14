/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_interface.c
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

#include <nuttx/signal.h>

#include "bcmf_interface.h"
#include "debug.h"
#include "assert.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bcmf_interface_frame_t
g_pktframes[CONFIG_IEEE80211_BROADCOM_FRAME_POOL_SIZE];

static struct list_node free_interface_frames;     /* Queue of available frames */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_initialize_interface_frames
 ****************************************************************************/

void bcmf_initialize_interface_frames(void)
{
  int i;

  list_initialize(&free_interface_frames);

  for (i = 0; i < CONFIG_IEEE80211_BROADCOM_FRAME_POOL_SIZE; ++i)
    {
      list_add_tail(&free_interface_frames, &g_pktframes[i].list_entry);
    }
}

/****************************************************************************
 * Name: bcmf_interface_free_frame
 ****************************************************************************/

void bcmf_interface_free_frame(FAR struct bcmf_dev_s  *priv,
                               bcmf_interface_frame_t *iframe)
{
  FAR bcmf_interface_dev_t *ibus = (FAR bcmf_interface_dev_t *) priv->bus;

  if (nxsem_wait_uninterruptible(&ibus->queue_mutex) < 0)
    {
      DEBUGPANIC();
    }

  list_add_head(&free_interface_frames, &iframe->list_entry);

  if (iframe->tx)
    {
      ibus->tx_queue_count--;
    }

  nxsem_post(&ibus->queue_mutex);
}

/****************************************************************************
 * Name: bcmf_interface_allocate_frame
 ****************************************************************************/

bcmf_interface_frame_t
*bcmf_interface_allocate_frame(FAR struct bcmf_dev_s *priv,
                                        bool                   block,
                                        bool                   tx)
{
  FAR bcmf_interface_dev_t *ibus = (FAR bcmf_interface_dev_t *) priv->bus;
  bcmf_interface_frame_t   *iframe;

  while (1)
    {
      if (nxsem_wait_uninterruptible(&ibus->queue_mutex) < 0)
        {
          DEBUGPANIC();
        }

      if (!tx ||
          ibus->tx_queue_count <
            CONFIG_IEEE80211_BROADCOM_FRAME_POOL_SIZE / 2)
        {
          if ((iframe = list_remove_head_type(&free_interface_frames,
                                              bcmf_interface_frame_t,
                                              list_entry)) != NULL)
            {
              if (tx)
                {
                  ibus->tx_queue_count++;
                }

              nxsem_post(&ibus->queue_mutex);
              break;
            }
        }

      nxsem_post(&ibus->queue_mutex);

      if (!block)
        {
          wlinfo("No avail buffer\n");
          return NULL;
        }

      nxsig_usleep(10 * 1000);
    }

#if defined(CONFIG_IEEE80211_BROADCOM_FULLMAC_GSPI)
  iframe->header.len  = CONFIG_IEEE80211_BROADCOM_FULLMAC_GSPI_MAX_FRAME;
#else
  iframe->header.len  = HEADER_SIZE + MAX_NETDEV_PKTSIZE +
                        CONFIG_NET_GUARDSIZE;
#endif
  iframe->header.base = iframe->data;
  iframe->header.data = iframe->data;
  iframe->tx          = tx;
  return iframe;
}
