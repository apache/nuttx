/****************************************************************************
 * drivers/virtio/virtio-mmio-blk.h
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

#ifndef __DRIVERS_VIRTIO_VIRTIO_MMIO_BLK_H
#define __DRIVERS_VIRTIO_VIRTIO_MMIO_BLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_DRIVERS_VIRTIO_BLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_mmio_blk_init
 *
 * Description:
 *   Called from virtio-mmio.c to initialize virtblk
 *
 ****************************************************************************/

int virtio_mmio_blk_init(FAR struct virtio_mmio_regs *regs, uint32_t intid);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO_BLK */
#endif /* __DRIVERS_VIRTIO_VIRTIO_MMIO_BLK_H */
