/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_interface.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_INTERFACE_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_INTERFACE_H

/* ==== This file contains dispatch between the SDIO & gSPI interface. ==== */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_IEEE80211_BROADCOM_FULLMAC_SDIO)
#include "bcmf_sdio.h"
#elif defined(CONFIG_IEEE80211_BROADCOM_FULLMAC_GSPI)
#include "bcmf_gspi.h"
#else
#error Must define IEEE80211_BROADCOM_FULLMAC_SDIO or IEEE80211_BROADCOM_FULLMAC_GSPI
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bcmf_dev_s;

#if defined(CONFIG_IEEE80211_BROADCOM_FULLMAC_SDIO)
typedef struct bcmf_sdio_dev_s bcmf_interface_dev_t;
#else
typedef bcmf_gspi_dev_t bcmf_interface_dev_t;
#endif

/* Note:
 * The structure referred to above must as its first item:
 *
 *  struct bcmf_bus_dev_s  bus;            --- Default bcmf bus structure
 *
 * The structure must also contain the following items:
 *
 *  int                    cur_chip_id;    --- Chip ID read from the card
 *  struct bcmf_chip_data *chip;           --- Chip specific configuration
 *
 *  sem_t                  thread_signal;  --- Thread event semaphore
 *
 *  uint32_t       backplane_current_addr; --- Current F1 backplane base addr
 *
 *  uint8_t                max_seq;        --- Maximum TX sequence allowed
 *  uint8_t                tx_seq;         --- TX sequence number (next)
 *
 *  sem_t                  queue_mutex;    --- Lock for TX/RX/free queues
 *  struct list_node       tx_queue;       --- Queue of frames to transmit
 *  struct list_node       rx_queue;       --- Queue of frames for receiving
 *  volatile int           tx_queue_count; --- Count of items in TX queue
 */

/* Structure used to manage interface frames */

typedef struct bcmf_interface_frame_s
{
  struct bcmf_frame_s header;
  bool                tx;
  struct list_node    list_entry;
  uint8_t             pad[CONFIG_IEEE80211_BROADCOM_DMABUF_ALIGNMENT -
                          FIRST_WORD_SIZE]
  aligned_data(CONFIG_IEEE80211_BROADCOM_DMABUF_ALIGNMENT);

  /* pad[] array is used and aligned in order to make the following data[]
   * buffer aligned beginning from the offset of 4 bytes to the address
   * boundary for SDIO DMA transfers.
   * The first 4 bytes of data[] buffer are not directly used in DMA
   * transfers. Instead, they are used as the initial phase just to get
   * the length of the remaining long data to be read. Thus only
   * the remaining part of data[] buffer beginning from the offset of 4 bytes
   * is required to be aligned to the address boundary set by
   * CONFIG_IEEE80211_BROADCOM_SDIO_DMA_BUF_ALIGNMENT parameter.
   */

  uint8_t             data[HEADER_SIZE + MAX_NETDEV_PKTSIZE +
                           CONFIG_NET_GUARDSIZE];
} bcmf_interface_frame_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_bus_interface_active
 ****************************************************************************/

static inline int bcmf_bus_interface_active(FAR struct bcmf_dev_s *priv,
                                            bool                   active)
{
#if defined(CONFIG_IEEE80211_BROADCOM_FULLMAC_SDIO)
  return bcmf_bus_sdio_active(priv, active);
#else
  return bcmf_bus_gspi_active(priv, active);
#endif
}

/****************************************************************************
 * Name: bcmf_bus_io_abort
 ****************************************************************************/

static inline void bcmf_bus_io_abort(FAR bcmf_interface_dev_t *ibus)
{
#if defined(CONFIG_IEEE80211_BROADCOM_FULLMAC_SDIO)
  bcmf_write_reg(ibus, 0, SDIO_CCCR_IOABORT, 2);
#else
  return;
#endif
}

/****************************************************************************
 * Name: bcmf_initialize_interface_frames
 ****************************************************************************/

void bcmf_initialize_interface_frames(void);

/****************************************************************************
 * Name: bcmf_interface_free_frame
 ****************************************************************************/

void bcmf_interface_free_frame(FAR struct bcmf_dev_s  *priv,
                               bcmf_interface_frame_t *iframe);

/****************************************************************************
 * Name: bcmf_interface_allocate_frame
 ****************************************************************************/

bcmf_interface_frame_t
*bcmf_interface_allocate_frame(FAR struct bcmf_dev_s *priv,
                              bool                   block,
                              bool                   tx);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_INTERFACE_H */
