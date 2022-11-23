/****************************************************************************
 * include/nuttx/virtio/virtio-mmio.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_MMIO_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_MMIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <arch/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* See virtio-v1.1-csprd01.pdf */

#define VIRTIO_MAGIC    0x74726976
#define VIRTIO_VERSION  0x2    /* NOTE: Legacy devices used 0x1 */

#define VIRTIO_DEV_NET  0x1

#define VIRTIO_STATUS_ACKNOWLEDGE   (1)
#define VIRTIO_STATUS_DRIVER        (2)
#define VIRTIO_STATUS_DRIVER_OK     (4)
#define VIRTIO_STATUS_FEATURES_OK   (8)

#define VIRTQ_DESC_F_NEXT  1   /* marks a buffer as continuing */
#define VIRTQ_DESC_F_WRITE 2   /* marks a buffer as device write-only */

#define virtio_mb() SP_DMB()

#define virtio_getreg32(a)    (FAR *(volatile FAR uint32_t *)(a))
#define virtio_putreg32(v,a)  (FAR *(volatile FAR uint32_t *)(a) = (v))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Table 4.1 MMIO Device Register Layout */

struct virtio_mmio_regs
{
  uint32_t magic_value;           /* offset:0x000 */
  uint32_t version;               /* offset:0x004 */
  uint32_t device_id;             /* offset:0x008 */
  uint32_t vendor_id;             /* offset:0x00c */
  uint32_t device_features;       /* offset:0x010 */
  uint32_t device_features_sel;   /* offset:0x014 */
  uint32_t _reserved0[2];
  uint32_t driver_features;       /* offset:0x020 */
  uint32_t driver_features_sel;   /* offset:0x024 */
  uint32_t _reserved1[2];
  uint32_t queue_sel;             /* offset:0x030 */
  uint32_t queue_num_max;         /* offset:0x034 */
  uint32_t queue_num;             /* offset:0x038 */
  uint32_t _reserved2[2];
  uint32_t queue_ready;           /* offset:0x044 */
  uint32_t _reserved3[2];
  uint32_t queue_notify;          /* offset:0x050 */
  uint32_t _reserved4[3];
  uint32_t interrupt_status;      /* offset:0x060 */
  uint32_t interrupt_ack;         /* offset:0x064 */
  uint32_t _reserved5[2];
  uint32_t status;                /* offset:0x070 */
  uint32_t _reserved6[3];
  uint32_t queue_desc_low;        /* offset:0x080 */
  uint32_t queue_desc_high;       /* offset:0x084 */
  uint32_t _reserved7[2];
  uint32_t queue_avail_low;       /* offset:0x090 */
  uint32_t queue_avail_high;      /* offset:0x094 */
  uint32_t _reserved8[2];
  uint32_t queue_used_low;        /* offset:0x0a0 */
  uint32_t queue_used_high;       /* offset:0x0a4 */
  uint32_t _reserved9[21];
  uint32_t config_generation;     /* offset:0x0fc */
  uint32_t config[0];
};

struct virtqueue_desc
{
  uint64_t addr;
  uint32_t len;
  uint16_t flags;
  uint16_t next;
};

struct virtqueue_avail
{
  uint16_t flags;
  uint16_t idx;
  uint16_t ring[0];
};

struct virtqueue_used_elem
{
  uint32_t id;
  uint32_t len;
};

struct virtqueue_used
{
  uint16_t flags;
  uint16_t idx;
  struct virtqueue_used_elem ring[0];
};

struct virtqueue
{
  uint32_t len;
  uint16_t last_used_idx;

  FAR struct virtqueue_desc *desc;
  FAR struct virtqueue_avail *avail;
  FAR struct virtqueue_used *used;
  FAR uint16_t *avail_event;
  FAR void **desc_virt;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

FAR struct virtqueue *virtq_create(uint32_t len);

uint32_t virtq_alloc_desc(FAR struct virtqueue *virtq,
                          uint32_t id,
                          FAR void *addr);

void virtq_add_to_mmio_device(FAR struct virtio_mmio_regs *regs,
                              FAR struct virtqueue *virtq,
                              uint32_t queue_sel);

void virtio_mmio_init(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_MMIO_H */
