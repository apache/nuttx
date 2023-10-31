/****************************************************************************
 * include/nuttx/virtio/virtio.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/compiler.h>
#include <nuttx/list.h>
#include <nuttx/spinlock.h>

#ifdef CONFIG_DRIVERS_VIRTIO

#include <openamp/open_amp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Virtio common feature bits */

#define VIRTIO_F_ANY_LAYOUT   27

/* Virtio helper functions */

#define virtio_has_feature(vdev, fbit) \
      (((vdev)->features & (1ULL << (fbit))) != 0)

#define virtio_read_config_member(vdev, structname, member, ptr) \
      virtio_read_config((vdev), offsetof(structname, member), \
                         (ptr), sizeof(*(ptr)));

#define virtio_write_config_member(vdev, structname, member, ptr) \
      virtio_write_config((vdev), offsetof(structname, member), \
                          (ptr), sizeof(*(ptr)));

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct virtio_driver
{
  struct list_node   node;
  uint32_t           device;   /* device id */
  CODE int         (*probe)(FAR struct virtio_device *vdev);
  CODE void        (*remove)(FAR struct virtio_device *vdev);
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtqueue_add_buffer_lock
 ****************************************************************************/

static inline_function int
virtqueue_add_buffer_lock(FAR struct virtqueue *vq,
                          FAR struct virtqueue_buf *buf_list,
                          int readable, int writable,
                          FAR void *cookie,
                          FAR spinlock_t *lock)
{
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(lock);
  ret = virtqueue_add_buffer(vq, buf_list, readable, writable, cookie);
  spin_unlock_irqrestore(lock, flags);

  return ret;
}

/****************************************************************************
 * Name: virtqueue_get_buffer_lock
 ****************************************************************************/

static inline_function FAR void *
virtqueue_get_buffer_lock(FAR struct virtqueue *vq, FAR uint32_t *len,
                          FAR uint16_t *idx, FAR spinlock_t *lock)
{
  irqstate_t flags;
  FAR void *ret;

  flags = spin_lock_irqsave(lock);
  ret = virtqueue_get_buffer(vq, len, idx);
  spin_unlock_irqrestore(lock, flags);

  return ret;
}

/****************************************************************************
 * Name: virtqueue_get_available_buffer_lock
 ****************************************************************************/

static inline_function FAR void *
virtqueue_get_available_buffer_lock(FAR struct virtqueue *vq,
                                    FAR uint16_t *avail_idx,
                                    FAR uint32_t *len, FAR spinlock_t *lock)
{
  irqstate_t flags;
  FAR void *ret;

  flags = spin_lock_irqsave(lock);
  ret = virtqueue_get_available_buffer(vq, avail_idx, len);
  spin_unlock_irqrestore(lock, flags);

  return ret;
}

/****************************************************************************
 * Name: virtqueue_add_consumed_buffer_lock
 ****************************************************************************/

static inline_function int
virtqueue_add_consumed_buffer_lock(FAR struct virtqueue *vq,
                                   uint16_t head_idx, uint32_t len,
                                   FAR spinlock_t *lock)
{
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(lock);
  ret = virtqueue_add_consumed_buffer(vq, head_idx, len);
  spin_unlock_irqrestore(lock, flags);

  return ret;
}

/****************************************************************************
 * Name: virtqueue_disable_cb_lock
 ****************************************************************************/

static inline_function void
virtqueue_disable_cb_lock(FAR struct virtqueue *vq, FAR spinlock_t *lock)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(lock);
  virtqueue_disable_cb(vq);
  spin_unlock_irqrestore(lock, flags);
}

/****************************************************************************
 * Name: virtqueue_enable_cb_lock
 ****************************************************************************/

static inline_function int virtqueue_enable_cb_lock(FAR struct virtqueue *vq,
                                                    FAR spinlock_t *lock)
{
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(lock);
  ret = virtqueue_enable_cb(vq);
  spin_unlock_irqrestore(lock, flags);

  return ret;
}

/****************************************************************************
 * Name: virtqueue_kick_lock
 ****************************************************************************/

static inline_function void virtqueue_kick_lock(FAR struct virtqueue *vq,
                                                FAR spinlock_t *lock)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(lock);
  virtqueue_kick(vq);
  spin_unlock_irqrestore(lock, flags);
}

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

static inline_function FAR void *
virtio_zalloc_buf(FAR struct virtio_device *vdev, size_t size, size_t align)
{
  FAR void *buf = virtio_alloc_buf(vdev, size, align);
  if (buf != NULL)
    {
      memset(buf, 0, size);
    }

  return buf;
}

/* Driver and device register/unregister function */

int virtio_register_driver(FAR struct virtio_driver *driver);
int virtio_register_device(FAR struct virtio_device *device);

int virtio_unregister_driver(FAR struct virtio_driver *driver);
int virtio_unregister_device(FAR struct virtio_device *device);

/* Virtio driver initailied function, called in NuttX driver_intialize() */

void virtio_register_drivers(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO */

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_H */
