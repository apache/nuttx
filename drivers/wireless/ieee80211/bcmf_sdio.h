/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_sdio.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H

#include "bcmf_driver.h"
#include <stdint.h>
#include <stdbool.h>
#include <queue.h>
#include <semaphore.h>
#include <nuttx/sdio.h>

#include "bcmf_sdio_core.h"

#define HEADER_SIZE        0x12 /* Default sdpcm + bdc header size */
// TODO move to Kconfig
#define BCMF_PKT_POOL_SIZE 4    /* Frame pool size */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* sdio chip configuration structure */

struct bcmf_sdio_chip
{
  uint32_t ram_size;
  uint32_t core_base[MAX_CORE_ID];

  uint8_t      *firmware_image;
  unsigned int *firmware_image_size;

  uint8_t      *nvram_image;
  unsigned int *nvram_image_size;

  uint8_t      *clm_blob_image;
  unsigned int *clm_blob_image_size;
};

/* sdio bus structure extension */

struct bcmf_sdio_dev_s
{
  struct bcmf_bus_dev_s bus;       /* Default bcmf bus structure */
  FAR struct sdio_dev_s *sdio_dev; /* The SDIO device bound to this instance */
  int minor;                       /* Device minor number */

  int  cur_chip_id;                /* Chip ID read from the card */
  struct bcmf_sdio_chip *chip;     /* Chip specific configuration */

  volatile bool ready;             /* Current device status */
  bool sleeping;                   /* Current sleep status */

  int thread_id;                   /* Processing thread id */
  sem_t thread_signal;             /* Semaphore for processing thread event */
  struct wdog_s *waitdog;          /* Processing thread waitdog */

  uint32_t backplane_current_addr; /* Current function 1 backplane base addr */

  volatile bool irq_pending;       /* True if interrupt is pending */
  uint32_t intstatus;              /* Copy of device current interrupt status */

  uint8_t max_seq;                 /* Maximum transmit sequence allowed */
  uint8_t tx_seq;                  /* Transmit sequence number (next) */
  uint8_t rx_seq;                  /* Receive sequence number (expected) */

  sem_t queue_mutex;               /* Lock for TX/RX/free queues */
  dq_queue_t free_queue;           /* Queue of available frames */
  dq_queue_t tx_queue;             /* Queue of frames to tramsmit */
  dq_queue_t rx_queue;             /* Queue of frames used to receive */
  volatile int tx_queue_count;     /* Count of items in TX queue */
};

/* Structure used to manage SDIO frames */

struct bcmf_sdio_frame
{
  struct bcmf_frame_s header;
  bool                tx;
  dq_entry_t          list_entry;
  uint8_t             data[HEADER_SIZE + MAX_NETDEV_PKTSIZE +
                           CONFIG_NET_GUARDSIZE];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int bcmf_bus_sdio_initialize(FAR struct bcmf_dev_s *priv,
          int minor, FAR struct sdio_dev_s *dev);

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

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H */
