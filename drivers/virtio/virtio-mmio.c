/****************************************************************************
 * drivers/virtio/virtio-mmio.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/virtio/virtio-mmio.h>

#ifdef CONFIG_DRIVERS_VIRTIO_NET
#  include "virtio-mmio-net.h"
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_BLK
#  include "virtio-mmio-blk.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ptr_to_uint64(x)  ((uint64_t)(uintptr_t)(x))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtq_create
 ****************************************************************************/

FAR struct virtqueue *virtq_create(uint32_t len)
{
  FAR struct virtqueue *virtq = (FAR struct virtqueue *)
    kmm_zalloc(sizeof(struct virtqueue));
  ASSERT(virtq);

  virtq->len = len;

  /* See: 2.6 Split Virtqueues */

  virtq->desc  = (FAR struct virtqueue_desc *)
    kmm_memalign(16, 16 * len);
  ASSERT(virtq->desc);

  virtq->avail = (FAR struct virtqueue_avail *)
    kmm_memalign(2, 6 + 2 * len);
  ASSERT(virtq->avail);

  virtq->used  = (FAR struct virtqueue_used *)
    kmm_memalign(4, 6 + 8 * len);
  ASSERT(virtq->used);

  virtq->desc_virt = (FAR void **)
    kmm_memalign(16, sizeof(FAR void *) * len);
  ASSERT(virtq->desc_virt);

  vrtinfo("virtq=%p (len=%" PRId32 ")\n", virtq, len);
  vrtinfo("virtq->desc=%p \n", virtq->desc);
  vrtinfo("virtq->avail=%p \n", virtq->avail);
  vrtinfo("virtq->used=%p \n", virtq->used);

  virtq->avail->idx = 0;
  virtq->used->idx = 0;
  virtq->last_used_idx = 0;

  return virtq;
}

/****************************************************************************
 * Name: virtq_dev_init
 ****************************************************************************/

static int virtio_dev_init(uintptr_t virt, uint32_t irq)
{
  FAR struct virtio_mmio_regs *regs = (FAR struct virtio_mmio_regs *)virt;
  int ret = -ENODEV;
  uint32_t val;

  vrtinfo("examine virtio at 0x%" PRIxPTR "\n", virt);

  val = virtio_getreg32(&regs->magic_value);

  if (VIRTIO_MAGIC != val)
    {
      vrterr("error: virtio at 0x%" PRIxPTR
             " had wrong magic value 0x%" PRIx32 "\n", virt, val);
      return ret;
    }

  val = virtio_getreg32(&regs->version);

  if (VIRTIO_VERSION != val)
    {
      vrterr("error: virtio at 0x%" PRIxPTR
             " had wrong version 0x%" PRIx32 "\n", virt, val);
      return ret;
    }

  /* Reset */

  virtio_putreg32(0, &regs->status);
  virtio_mb();

  /* Set ack */

  val = virtio_getreg32(&regs->status) | VIRTIO_STATUS_ACKNOWLEDGE;
  virtio_putreg32(val, &regs->status);
  virtio_mb();

  /* Set driver */

  val = virtio_getreg32(&regs->status) | VIRTIO_STATUS_DRIVER;
  virtio_putreg32(val, &regs->status);
  virtio_mb();

  /* Check the device_id */

  val = virtio_getreg32(&regs->device_id);

  switch (val)
    {
#ifdef CONFIG_DRIVERS_VIRTIO_NET
    case VIRTIO_DEV_NET:
      ret = virtio_mmio_net_init(regs, irq);
      break;
#endif
#ifdef CONFIG_DRIVERS_VIRTIO_BLK
    case VIRTIO_DEV_BLK:
      ret = virtio_mmio_blk_init(regs, irq);
      break;
#endif
    default:
      vrtwarn("unsupported device_id 0x%" PRIx32 "\n", val);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtq_alloc_desc
 ****************************************************************************/

uint32_t virtq_alloc_desc(FAR struct virtqueue *virtq,
                          uint32_t idx, FAR void *addr)
{
  uint32_t id = idx % virtq->len;

  vrtinfo("virtq=%p, idx=%" PRId32 "\n", virtq, idx);

  virtq->desc[id].addr = (uintptr_t)addr;
  virtq->desc_virt[id] = addr;
  return id;
}

/****************************************************************************
 * Name: virtq_add_to_mmio_device
 ****************************************************************************/

void virtq_add_to_mmio_device(FAR struct virtio_mmio_regs *regs,
                              FAR struct virtqueue *virtq,
                              uint32_t queue_sel)
{
  vrtinfo("==== queue_sel=%" PRId32 ", virtq->len=%" PRId32 "\n",
          queue_sel, virtq->len);

  virtio_putreg32(queue_sel, &regs->queue_sel);
  virtio_mb();

  virtio_putreg32(virtq->len, &regs->queue_num);

  virtio_putreg32((uint32_t)(ptr_to_uint64(virtq->desc)),
           &regs->queue_desc_low);
  virtio_putreg32((uint32_t)(ptr_to_uint64(virtq->desc) >> 32),
           &regs->queue_desc_high);

  virtio_putreg32((uint32_t)(ptr_to_uint64(virtq->avail)),
           &regs->queue_avail_low);
  virtio_putreg32((uint32_t)(ptr_to_uint64(virtq->avail) >> 32),
           &regs->queue_avail_high);

  virtio_putreg32((uint32_t)(ptr_to_uint64(virtq->used)),
           &regs->queue_used_low);
  virtio_putreg32((uint32_t)(ptr_to_uint64(virtq->used) >> 32),
           &regs->queue_used_high);

  virtio_mb();
  virtio_putreg32(1, &regs->queue_ready);
}

/****************************************************************************
 * Name: virtio_mmio_init
 ****************************************************************************/

void virtio_mmio_init(void)
{
  uintptr_t virtio = (uintptr_t)CONFIG_DRIVERS_VIRTIO_MMIO_BASE;
  uint32_t  irq = CONFIG_DRIVERS_VIRTIO_MMIO_IRQ;
  uint32_t  size = CONFIG_DRIVERS_VIRTIO_MMIO_REGSIZE;
  uint32_t  i;

  for (i = 0; i < CONFIG_DRIVERS_VIRTIO_MMIO_NUM; i++)
    {
      virtio_dev_init(virtio + size * i, irq + i);
    }
}
