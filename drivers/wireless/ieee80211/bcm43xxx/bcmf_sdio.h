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

#include <nuttx/list.h>
#include <nuttx/sdio.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include "bcmf_chip_data.h"
#include "bcmf_driver.h"
#include "bcmf_sdio_core.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HEADER_SIZE          0x12 /* Default sdpcm + bdc header size */
#define FIRST_WORD_SIZE      4
#define FC_UPDATE_PKT_LENGTH 12

#define BCMF_UPLOAD_TRANSFER_SIZE (64 * 256)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SDIO bus structure extension */

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
 *  mutex_t                queue_lock;    --- Lock for TX/RX/free queues
 *  struct list_node       tx_queue;       --- Queue of frames to transmit
 *  struct list_node       rx_queue;       --- Queue of frames for receiving
 *  volatile int           tx_queue_count; --- Count of items in TX queue
 */

struct bcmf_sdio_dev_s
{
  struct bcmf_bus_dev_s bus;       /* Default bcmf bus structure */
  FAR struct sdio_dev_s *sdio_dev; /* The SDIO device bound to this instance */
  int minor;                       /* Device minor number */

  int  cur_chip_id;                /* Chip ID read from the card */
  struct bcmf_chip_data *chip;     /* Chip specific configuration */

  volatile bool ready;             /* Current device status */
  bool sleeping;                   /* Current sleep status */
  bool kso_enable;                 /* Current Keep sdio on status */
  bool support_sr;                 /* Firmware support save restore */

  pid_t thread_id;                 /* Processing thread id */
  sem_t thread_signal;             /* Semaphore for processing thread event */

  uint32_t backplane_current_addr; /* Current function 1 backplane base addr */

  volatile bool irq_pending;       /* True if interrupt is pending */
  uint32_t intstatus;              /* Copy of device current interrupt status */

  uint8_t max_seq;                 /* Maximum transmit sequence allowed */
  uint8_t tx_seq;                  /* Transmit sequence number (next) */
  uint8_t rx_seq;                  /* Receive sequence number (expected) */
  bool    flow_ctrl;               /* Current flow control status */

  mutex_t queue_lock;              /* Lock for TX/RX/free queues */
  struct list_node tx_queue;       /* Queue of frames to transmit */
  struct list_node rx_queue;       /* Queue of frames used to receive */
  volatile int tx_queue_count;     /* Count of items in TX queue */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int bcmf_sdio_initialize(int minor, FAR struct sdio_dev_s *dev);

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

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_SDIO_H */
