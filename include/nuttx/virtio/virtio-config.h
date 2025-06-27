/****************************************************************************
 * include/nuttx/virtio/virtio-config.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_CONFIG_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_OPENAMP

#include <nuttx/compiler.h>
#include <openamp/open_amp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Virtio common feature bits */

#define VIRTIO_F_ANY_LAYOUT   27

/* Virtio helper functions */

#define virtio_read_config_member(vdev, structname, member, ptr) \
      virtio_read_config((vdev), offsetof(structname, member), \
                         (ptr), sizeof(*(ptr)));

#define virtio_write_config_member(vdev, structname, member, ptr) \
      virtio_write_config((vdev), offsetof(structname, member), \
                          (ptr), sizeof(*(ptr)));

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

static inline void virtio_read_config_bytes(FAR struct virtio_device *vdev,
                                            uint32_t offset, FAR void *dst,
                                            int len)
{
  int i;

  for (i = 0; i < len; i++)
    {
      virtio_read_config(vdev, offset + i, dst + i, 1);
    }
}

#endif /* CONFIG_OPENAMP */

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_CONFIG_H */
