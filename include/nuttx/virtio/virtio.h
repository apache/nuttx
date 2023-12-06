/****************************************************************************
 * include/nuttx/virtio/virtio.h
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

#ifdef CONFIG_DRIVERS_VIRTIO

#include <openamp/open_amp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define virtio_has_feature(vdev, fbit) \
      (((vdev)->features & (1UL << (fbit))) != 0)

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
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Driver and device register/unregister function */

int virtio_register_driver(FAR struct virtio_driver *driver);
int virtio_register_device(FAR struct virtio_device *device);

int virtio_unregister_driver(FAR struct virtio_driver *driver);
int virtio_unregister_device(FAR struct virtio_device *device);

/* Virtio alloc/free buffer API
 * NOTE:
 * For now, these three apis are implemented in NuttX, and direclty mapping
 * to kmm_memalgin/kmm_free, so it's only compatible with virtio mmio
 * transport for now. After the virtio remoteproc transport layer completed,
 * these three apis should be moved to OpenAmp, and different transport layer
 * provide different implementation.
 */

FAR void *virtio_alloc_buf(FAR struct virtio_device *vdev,
                           size_t size, size_t align);
FAR void *virtio_zalloc_buf(FAR struct virtio_device *vdev,
                            size_t size, size_t align);
void virtio_free_buf(FAR struct virtio_device *vdev, FAR void *buf);

/* Virtio driver initailied function, called in NuttX driver_intialize() */

void virtio_register_drivers(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO */

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_H */
