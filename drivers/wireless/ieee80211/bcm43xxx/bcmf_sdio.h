/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_sdio.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDIO_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <queue.h>

#include <nuttx/sdio.h>
#include <nuttx/semaphore.h>

#include "bcmf_driver.h"
#include "bcmf_sdio_core.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HEADER_SIZE          0x12 /* Default sdpcm + bdc header size */
#define FIRST_WORD_SIZE      4
#define FC_UPDATE_PKT_LENGTH 12

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SDIO chip configuration structure */

struct bcmf_sdio_chip
{
  uint32_t ram_base;
  uint32_t ram_size;
  uint32_t core_base[MAX_CORE_ID];

  /* In-memory file images */

  FAR uint8_t *nvram_image;
  FAR unsigned int *nvram_image_size;

#ifndef CONFIG_IEEE80211_BROADCOM_FWFILES
  FAR uint8_t *firmware_image;
  FAR unsigned int *firmware_image_size;

#ifdef CONFIG_IEEE80211_BROADCOM_HAVE_CLM
  FAR uint8_t *clm_blob_image;
  FAR unsigned int *clm_blob_image_size;
#endif
#endif
};

/* SDIO bus structure extension */

struct bcmf_sdio_dev_s
{
  struct bcmf_bus_dev_s bus;       /* Default bcmf bus structure */
  FAR struct sdio_dev_s *sdio_dev; /* The SDIO device bound to this instance */
  int minor;                       /* Device minor number */

  int  cur_chip_id;                /* Chip ID read from the card */
  struct bcmf_sdio_chip *chip;     /* Chip specific configuration */

  volatile bool ready;             /* Current device status */
  bool sleeping;                   /* Current sleep status */

  pid_t thread_id;                 /* Processing thread id */
  sem_t thread_signal;             /* Semaphore for processing thread event */
  struct wdog_s waitdog;           /* Processing thread waitdog */

  uint32_t backplane_current_addr; /* Current function 1 backplane base addr */

  volatile bool irq_pending;       /* True if interrupt is pending */
  uint32_t intstatus;              /* Copy of device current interrupt status */

  uint8_t max_seq;                 /* Maximum transmit sequence allowed */
  uint8_t tx_seq;                  /* Transmit sequence number (next) */
  uint8_t rx_seq;                  /* Receive sequence number (expected) */
  bool    flow_ctrl;               /* Current flow control status */

  sem_t queue_mutex;               /* Lock for TX/RX/free queues */
  dq_queue_t free_queue;           /* Queue of available frames */
  dq_queue_t tx_queue;             /* Queue of frames to transmit */
  dq_queue_t rx_queue;             /* Queue of frames used to receive */
  volatile int tx_queue_count;     /* Count of items in TX queue */
};

/* Structure used to manage SDIO frames */

struct bcmf_sdio_frame
{
  struct bcmf_frame_s header;
  bool                tx;
  dq_entry_t          list_entry;
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
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int bcmf_bus_sdio_initialize(FAR struct bcmf_dev_s *priv,
          int minor, FAR struct sdio_dev_s *dev);

int bcmf_bus_sdio_active(FAR struct bcmf_dev_s *priv, bool active);

/* FIXME: Low level bus data transfer function
 * To avoid bus error, len will be aligned to:
 * - upper power of 2 iflen is lesser than 64
 * - upper 64 bytes block if len is greater than 64
 */

int bcmf_transfer_bytes(FAR struct bcmf_sdio_dev_s *sbus, bool write,
                        uint8_t function, uint32_t address,
                        uint8_t *buf, unsigned int len);

int bcmf_read_reg(FAR struct bcmf_sdio_dev_s *sbus, uint8_t function,
                  uint32_t address, uint8_t *reg);

int bcmf_write_reg(FAR struct bcmf_sdio_dev_s *sbus, uint8_t function,
                   uint32_t address, uint8_t reg);

struct bcmf_sdio_frame *bcmf_sdio_allocate_frame(FAR struct bcmf_dev_s *priv,
                                                 bool block, bool tx);

void bcmf_sdio_free_frame(FAR struct bcmf_dev_s *priv,
                          struct bcmf_sdio_frame *sframe);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDIO_H */
