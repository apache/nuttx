/****************************************************************************
 * boards/arm/rzv2h/rzv2h-evk/src/rzv2h_rsctable.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <arch/board/board.h>

#include <openamp/remoteproc.h>
#include <openamp/rpmsg_virtio.h>
#include <openamp/virtio.h>
#include <openamp/virtqueue.h>

#include "rzv2h_rsctable.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_RSC_VERSION          1
#define RZV2H_RSC_ENTRIES          2
#define RZV2H_RSC_RPROC_MEM        4
#define RZV2H_RPMSG_VRINGS         2
#define RZV2H_RPMSG_FEATURES       ((1U << VIRTIO_RPMSG_F_NS) | \
                                    VIRTIO_RING_F_MUST_NOTIFY)
#define RZV2H_RPMSG_CHANNELS       2

/* The Multi-OS v3.3 Linux OpenAMP master reads dfeatures directly and does
 * not write gfeatures.  Current OpenAMP on the NuttX device side uses the
 * intersection of both fields.  Initialize both to the common features so
 * the legacy master and current device agree on name-service support.  The
 * legacy master also leaves VRING_AVAIL_F_NO_INTERRUPT set while relying on
 * every remote kick, so VIRTIO_RING_F_MUST_NOTIFY preserves that behavior.
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* RSC_RPROC_MEM is a Renesas extension used by the Multi-OS v3.3 host.
 * Current upstream OpenAMP has no named type for it, but safely ignores
 * unknown non-vendor entries while still parsing the following RSC_VDEV.
 */

struct rzv2h_rsc_rproc_mem_s
{
  uint32_t type;
  uint32_t da;
  uint32_t pa;
  uint32_t len;
  uint32_t reserved;
};

struct rzv2h_resource_table_s
{
  uint32_t version;
  uint32_t num;
  uint32_t reserved[2];
  uint32_t offset[RZV2H_RSC_ENTRIES];
  struct rzv2h_rsc_rproc_mem_s rproc_mem;
  struct fw_rsc_vdev rpmsg_vdev;
  struct fw_rsc_vdev_vring rpmsg_vring0;
  struct fw_rsc_vdev_vring rpmsg_vring1;
};

static_assert(offsetof(struct rzv2h_resource_table_s, rproc_mem) ==
              BOARD_RPTUN_RESOURCE_RPROC_MEM_OFFSET,
              "Renesas resource-table memory-entry offset changed");
static_assert(offsetof(struct rzv2h_resource_table_s, rpmsg_vdev) ==
              BOARD_RPTUN_RESOURCE_VDEV_OFFSET,
              "Renesas resource-table vdev offset changed");
static_assert(sizeof(struct rzv2h_resource_table_s) ==
              BOARD_RPTUN_RESOURCE_ENTRY_SIZE,
              "Renesas resource-table ABI changed");

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rzv2h_resource_table_s
  g_rzv2h_resources[RZV2H_RPMSG_CHANNELS]
  locate_data(".resource_table") used_data;

static const struct rzv2h_resource_table_s
  g_rzv2h_resources_init[RZV2H_RPMSG_CHANNELS] =
{
    {
      .version = RZV2H_RSC_VERSION,
      .num     = RZV2H_RSC_ENTRIES,
      .offset  =
        {
          offsetof(struct rzv2h_resource_table_s, rproc_mem),
          offsetof(struct rzv2h_resource_table_s, rpmsg_vdev),
        },
      .rproc_mem =
        {
          .type = RZV2H_RSC_RPROC_MEM,
          .da   = BOARD_RPTUN_RPMSG_ADDR,
          .pa   = BOARD_RPTUN_RPMSG_ADDR,
          .len  = BOARD_RPTUN_RPMSG_SIZE,
        },
      .rpmsg_vdev =
        {
          .type          = RSC_VDEV,
          .id            = VIRTIO_ID_RPMSG,
          .notifyid      = BOARD_RPTUN_VDEV_NOTIFYID,
          .dfeatures     = RZV2H_RPMSG_FEATURES,
          .gfeatures     = RZV2H_RPMSG_FEATURES,
          .num_of_vrings = RZV2H_RPMSG_VRINGS,
        },
      .rpmsg_vring0 =
        {
          .da       = BOARD_RPTUN_CHANNEL0_VRING0_ADDR,
          .align    = BOARD_RPTUN_VRING_ALIGN,
          .num      = BOARD_RPTUN_VRING_NUM,
          .notifyid = BOARD_RPTUN_CHANNEL0_NOTIFYID,
        },
      .rpmsg_vring1 =
        {
          .da       = BOARD_RPTUN_CHANNEL0_VRING1_ADDR,
          .align    = BOARD_RPTUN_VRING_ALIGN,
          .num      = BOARD_RPTUN_VRING_NUM,
          .notifyid = BOARD_RPTUN_CHANNEL0_NOTIFYID,
        },
    },
    {
      .version = RZV2H_RSC_VERSION,
      .num     = RZV2H_RSC_ENTRIES,
      .offset  =
        {
          offsetof(struct rzv2h_resource_table_s, rproc_mem),
          offsetof(struct rzv2h_resource_table_s, rpmsg_vdev),
        },
      .rproc_mem =
        {
          .type = RZV2H_RSC_RPROC_MEM,
          .da   = BOARD_RPTUN_RPMSG_ADDR,
          .pa   = BOARD_RPTUN_RPMSG_ADDR,
          .len  = BOARD_RPTUN_RPMSG_SIZE,
        },
      .rpmsg_vdev =
        {
          .type          = RSC_VDEV,
          .id            = VIRTIO_ID_RPMSG,
          .notifyid      = BOARD_RPTUN_VDEV_NOTIFYID,
          .dfeatures     = RZV2H_RPMSG_FEATURES,
          .gfeatures     = RZV2H_RPMSG_FEATURES,
          .num_of_vrings = RZV2H_RPMSG_VRINGS,
        },
      .rpmsg_vring0 =
        {
          .da       = BOARD_RPTUN_CHANNEL1_VRING0_ADDR,
          .align    = BOARD_RPTUN_VRING_ALIGN,
          .num      = BOARD_RPTUN_VRING_NUM,
          .notifyid = BOARD_RPTUN_CHANNEL1_NOTIFYID,
        },
      .rpmsg_vring1 =
        {
          .da       = BOARD_RPTUN_CHANNEL1_VRING1_ADDR,
          .align    = BOARD_RPTUN_VRING_ALIGN,
          .num      = BOARD_RPTUN_VRING_NUM,
          .notifyid = BOARD_RPTUN_CHANNEL1_NOTIFYID,
        },
    },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_rptun_resource_initialize
 ****************************************************************************/

FAR struct resource_table *rzv2h_rptun_resource_initialize(void)
{
  memcpy(g_rzv2h_resources, g_rzv2h_resources_init,
         sizeof(g_rzv2h_resources));

  /* The live table is in normal, non-cacheable shared memory.  Complete the
   * initialization before Linux is allowed to observe VirtIO state there.
   */

  UP_DSB();
  return (FAR struct resource_table *)
         &g_rzv2h_resources[BOARD_RPTUN_CHANNEL];
}

/****************************************************************************
 * Name: rzv2h_rptun_resource_reset
 ****************************************************************************/

void rzv2h_rptun_resource_reset(void)
{
  memcpy(&g_rzv2h_resources[BOARD_RPTUN_CHANNEL],
         &g_rzv2h_resources_init[BOARD_RPTUN_CHANNEL],
         sizeof(g_rzv2h_resources[BOARD_RPTUN_CHANNEL]));

  /* Linux owns the virtio DRIVER_OK bit.  Clear the completed session and
   * publish a fresh table so the NuttX RPTUN start thread waits for the next
   * Linux rpmsg_sample_client instance before recreating its virtqueues.
   */

  UP_DSB();
}
