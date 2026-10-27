/****************************************************************************
 * drivers/ivshmem/ivshmem_flat.c
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

#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/kmalloc.h>
#include <nuttx/ivshmem.h>
#include <nuttx/ivshmem_flat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef getreg32
#define getreg32(a)      (*(volatile uint32_t *)(a))
#endif

#ifndef putreg32
#define putreg32(v,a)    (*(volatile uint32_t *)(a) = (v))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ivshmem_flat_kick_peer
 ****************************************************************************/

static int ivshmem_flat_kick_peer(FAR struct ivshmem_device_s *dev,
                                  uint32_t peer_id)
{
  /* Vector 0 in bits 0..15, peer id in bits 16..31 */

  putreg32(peer_id << 16, (uintptr_t)dev->reg + IVSHMEM_REG_DOORBELL);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ivshmem_flat_register
 ****************************************************************************/

int ivshmem_flat_register(FAR const struct ivshmem_flat_config_s *config)
{
  FAR struct ivshmem_device_s *dev;
  int ret;

  DEBUGASSERT(config != NULL && config->reg != 0 && config->shmem != 0);

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->id         = config->id;
  dev->reg        = (FAR void *)config->reg;
  dev->shmem      = (FAR void *)config->shmem;
  dev->shmem_size = config->shmem_size;
  dev->irq        = config->irq;
  dev->vmid       = IVSHMEM_INVALID_VMID;

  /* Like ivshmem-plain without an MSI-X BAR: no irq means no doorbell, so
   * the drivers on top poll.
   */

  if (config->irq >= 0)
    {
      dev->vmid = getreg32((uintptr_t)dev->reg + IVSHMEM_REG_IVPOSITION);
      if (dev->vmid != 0 && dev->vmid != 1)
        {
          _warn("vmid=%" PRIu32 " not 0/1, use polling mode\n", dev->vmid);
          dev->vmid = IVSHMEM_INVALID_VMID;
        }
    }

  _info("reg=%p shmem=%p size=%zu irq=%d vmid=%" PRIu32 "\n",
        dev->reg, dev->shmem, dev->shmem_size, dev->irq, dev->vmid);

  ret = ivshmem_register_device(dev, ivshmem_flat_kick_peer);
  if (ret < 0)
    {
      kmm_free(dev);
    }

  return ret;
}
