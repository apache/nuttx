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

#include <nuttx/fs/ioctl.h>
#include <nuttx/queue.h>
#include <nuttx/spinlock.h>
#include <nuttx/virtio/virtio-config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VirtIO ioctl command definitions */

#define VIRTIOIOC_DUMP        _VIRTIOIOC(1)

/* Run a ping measurement.  arg is a struct virtio_ping_s, which names the
 * target device, since /dev/virtio is a single global node.
 */

#define VIRTIOIOC_PING        _VIRTIOIOC(2)

/* Instance name length, NUL included.  The name selects one ping device
 * out of several and is shown by "virtio dump" / "vhost dump".
 */

#define VIRTIO_PING_NAMELEN   32

/* Argument to VIRTIOIOC_PING and VHOSTIOC_PING.  Kept beside the commands
 * it serves, so including this header is enough to issue one.
 */

struct virtio_ping_s
{
  char name[VIRTIO_PING_NAMELEN];  /* Instance name, or "all" */
  int  times;                      /* Number of ping operations */
  int  len;                        /* Request length, header included */
  int  sleep;                      /* Interval between operations, ms */
};

#ifdef CONFIG_OPENAMP

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_VIRTIO
/* Called for each device visited by virtio_foreach_device().  Returning a
 * negative errno stops the walk and is handed back to the caller.
 */

typedef CODE int (*virtio_foreach_t)(FAR struct virtio_device *vdev,
                                     FAR void *arg);

struct virtio_driver
{
  uint32_t           device;   /* device id */
  CODE int         (*probe)(FAR struct virtio_device *vdev);
  CODE void        (*remove)(FAR struct virtio_device *vdev);
  CODE void        (*dump)(FAR struct virtio_device *vdev);
  struct dq_entry_s  node;
};
#endif

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

  flags = spin_lock_irqsave_notrace(lock);
  ret = virtqueue_add_buffer(vq, buf_list, readable, writable, cookie);
  spin_unlock_irqrestore_notrace(lock, flags);

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

  flags = spin_lock_irqsave_notrace(lock);
  ret = virtqueue_get_buffer(vq, len, idx);
  spin_unlock_irqrestore_notrace(lock, flags);

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

  flags = spin_lock_irqsave_notrace(lock);
  ret = virtqueue_get_first_avail_buffer(vq, avail_idx, len);
  spin_unlock_irqrestore_notrace(lock, flags);

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

  flags = spin_lock_irqsave_notrace(lock);
  ret = virtqueue_add_consumed_buffer(vq, head_idx, len);
  spin_unlock_irqrestore_notrace(lock, flags);

  return ret;
}

/****************************************************************************
 * Name: virtqueue_disable_cb_lock
 ****************************************************************************/

static inline_function void
virtqueue_disable_cb_lock(FAR struct virtqueue *vq, FAR spinlock_t *lock)
{
  irqstate_t flags;

  flags = spin_lock_irqsave_notrace(lock);
  virtqueue_disable_cb(vq);
  spin_unlock_irqrestore_notrace(lock, flags);
}

/****************************************************************************
 * Name: virtqueue_enable_cb_lock
 ****************************************************************************/

static inline_function int virtqueue_enable_cb_lock(FAR struct virtqueue *vq,
                                                    FAR spinlock_t *lock)
{
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave_notrace(lock);
  ret = virtqueue_enable_cb(vq);
  spin_unlock_irqrestore_notrace(lock, flags);

  return ret;
}

/****************************************************************************
 * Name: virtqueue_kick_lock
 ****************************************************************************/

static inline_function void virtqueue_kick_lock(FAR struct virtqueue *vq,
                                                FAR spinlock_t *lock)
{
  irqstate_t flags;

  flags = spin_lock_irqsave_notrace(lock);
  virtqueue_kick(vq);
  spin_unlock_irqrestore_notrace(lock, flags);
}

static inline_function FAR void *
virtio_malloc_buf(FAR struct virtio_device *vdev, size_t size, size_t align)
{
  FAR void *buf;

  if (virtio_alloc_buf(vdev, &buf, size, align) < 0)
    {
      return NULL;
    }

  return buf;
}

static inline_function FAR void *
virtio_zalloc_buf(FAR struct virtio_device *vdev, size_t size, size_t align)
{
  FAR void *buf;

  if (virtio_alloc_buf(vdev, &buf, size, align) < 0)
    {
      return NULL;
    }

  memset(buf, 0, size);
  return buf;
}

#endif /* CONFIG_OPENAMP */

#ifdef CONFIG_DRIVERS_VIRTIO

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Driver and device register/unregister function */

int virtio_register_driver(FAR struct virtio_driver *driver);
int virtio_register_device(FAR struct virtio_device *device);

int virtio_unregister_driver(FAR struct virtio_driver *driver);
int virtio_unregister_device(FAR struct virtio_device *device);

/* VirtIO driver initailied function, called in NuttX driver_intialize() */

void virtio_register_drivers(void);

/****************************************************************************
 * Name: virtio_foreach_device
 *
 * Description:
 *   Walk every device on the virtio bus whose device id is 'id' and which a
 *   driver has claimed, so vdev->priv is that driver's context.  The bus
 *   lock is held across the walk, which keeps a device from being removed
 *   underneath the handler.
 *
 * Input Parameters:
 *   id      - Device id to match
 *   handler - Called once per matching device
 *   arg     - Opaque argument passed to handler
 *
 * Returned Value:
 *   Zero on success, or the first negative errno a handler returned.
 *
 ****************************************************************************/

int virtio_foreach_device(uint32_t id, virtio_foreach_t handler,
                          FAR void *arg);

/* Buffer map ops for MMIO/PCI with per-process address environments */

#ifdef CONFIG_ARCH_ADDRENV
size_t virtio_addrenv_free_largest(FAR struct virtio_device *vdev);
FAR void *virtio_addrenv_map(FAR struct virtio_device *vdev, FAR void *buf,
                             size_t len, enum virtio_dma_map dir);
void virtio_addrenv_unmap(FAR struct virtio_device *vdev, FAR void *buf,
                          size_t len, enum virtio_dma_map dir,
                          FAR void *mapped);
#endif

FAR void *virtio_get_next_buffer(FAR struct virtqueue *vq, uint16_t idx,
                                 FAR uint16_t *next_idx,
                                 FAR uint32_t *next_len);
ssize_t virtio_get_buffers(FAR struct virtqueue *vq,
                           FAR struct virtqueue_buf *vb, size_t vbcnt);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO */

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_H */
