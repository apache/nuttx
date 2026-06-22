/****************************************************************************
 * drivers/virtio/virtio.c
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

#include <nuttx/debug.h>

#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/nuttx.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-blk.h"
#include "virtio-gpu.h"
#include "virtio-input.h"
#include "virtio-net.h"
#include "virtio-rng.h"
#include "virtio-rpmb.h"
#include "virtio-rpmsg.h"
#include "virtio-serial.h"
#include "virtio-snd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Largest buffer virtio_addrenv_map() is asked to bounce at once */

#define VIRTIO_ADDRENV_MAP_MAX (64 * 1024)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_bus_s
{
  mutex_t lock;                /* Lock for the list */
  struct list_node device;     /* Wait match virtio device list */
  struct list_node driver;     /* Virtio driver list */
};

struct virtio_device_item_s
{
  struct list_node      node;    /* list node */
  struct virtio_device *device;  /* Pointer to the virtio device */
  struct virtio_driver *driver;  /* Pointer to the virtio driver that
                                  * matched with current virtio device
                                  */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_bus_s g_virtio_bus =
{
  NXMUTEX_INITIALIZER,
  LIST_INITIAL_VALUE(g_virtio_bus.device),
  LIST_INITIAL_VALUE(g_virtio_bus.driver),
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_drivers
 ****************************************************************************/

void virtio_register_drivers(void)
{
  struct metal_init_params params = METAL_INIT_DEFAULTS;
  int ret = OK;

  ret = metal_init(&params);
  if (ret < 0)
    {
      vrterr("metal_init failed, ret=%d\n", ret);
    }

#ifdef CONFIG_DRIVERS_VIRTIO_BLK
  ret = virtio_register_blk_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_blk_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_GPU
  ret = virtio_register_gpu_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_gpu_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_INPUT
  ret = virtio_register_input_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_input_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_NET
  ret = virtio_register_net_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_net_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_RNG
  ret = virtio_register_rng_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_rng_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_RPMSG
  ret = virtio_register_rpmsg_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_rpmsg_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_SERIAL
  ret = virtio_register_serial_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_serial_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_SOUND
  ret = virtio_register_snd_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_snd_driver failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_RPMB
  ret = virtio_register_rpmb_driver();
  if (ret < 0)
    {
      vrterr("virtio_register_rpmb_driver failed, ret=%d\n", ret);
    }
#endif

  UNUSED(ret);
}

/****************************************************************************
 * Name: virtio_register_driver
 ****************************************************************************/

int virtio_register_driver(FAR struct virtio_driver *driver)
{
  FAR struct list_node *node;
  int ret;

  DEBUGASSERT(driver != NULL && driver->probe != NULL &&
              driver->remove != NULL);

  ret = nxmutex_lock(&g_virtio_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add the driver to the virtio_bus driver list */

  list_add_tail(&g_virtio_bus.driver, &driver->node);

  /* Match all the devices has registered in the virtio_bus */

  list_for_every(&g_virtio_bus.device, node)
    {
      FAR struct virtio_device_item_s *item =
        container_of(node, struct virtio_device_item_s, node);
      FAR struct virtio_device *device = item->device;

      if (item->driver == NULL && driver->device == device->id.device)
        {
          /* If found the device in the device list, call driver probe,
           * if probe success, assign item->driver to indicate the device
           * matched.
           */

          device->priv = driver;
          if (driver->probe(device) >= 0)
            {
              item->driver = driver;
            }
        }
    }

  nxmutex_unlock(&g_virtio_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: virtio_unregister_driver
 ****************************************************************************/

int virtio_unregister_driver(FAR struct virtio_driver *driver)
{
  FAR struct list_node *node;
  int ret;

  DEBUGASSERT(driver != NULL);

  ret = nxmutex_lock(&g_virtio_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find all the devices matched with driver in device list */

  list_for_every(&g_virtio_bus.device, node)
    {
      FAR struct virtio_device_item_s *item =
        container_of(node, struct virtio_device_item_s, node);

      if (item->driver == driver)
        {
          /* 1. Call driver remove function;
           * 2. Mark item->driver NULL to indicate the device unmatched;
           */

          driver->remove(item->device);
          item->driver = NULL;
        }
    }

  /* Remove the driver from the driver list */

  list_delete(&driver->node);

  nxmutex_unlock(&g_virtio_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: virtio_register_device
 ****************************************************************************/

int virtio_register_device(FAR struct virtio_device *device)
{
  FAR struct virtio_device_item_s *item;
  FAR struct list_node *node;
  int ret;

  item = kmm_zalloc(sizeof(*item));
  if (item == NULL)
    {
      return -ENOMEM;
    }

  item->device = device;

  ret = nxmutex_lock(&g_virtio_bus.lock);
  if (ret < 0)
    {
      kmm_free(item);
      return ret;
    }

  /* Add the device to the virtio_bus device list */

  list_add_tail(&g_virtio_bus.device, &item->node);

  /* Match the driver has registered in the virtio_bus */

  list_for_every(&g_virtio_bus.driver, node)
    {
      FAR struct virtio_driver *driver =
        container_of(node, struct virtio_driver, node);

      if (driver->device == device->id.device)
        {
          /* If found the driver in the driver list, call driver probe,
           * if probe success, assign item->driver to indicate the device
           * matched.
           */

          device->priv = driver;
          if (driver->probe(device) >= 0)
            {
              item->driver = driver;
              break;
            }
        }
    }

  nxmutex_unlock(&g_virtio_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: virtio_unregister_device
 ****************************************************************************/

int virtio_unregister_device(FAR struct virtio_device *device)
{
  FAR struct list_node *node;
  int ret;

  ret = nxmutex_lock(&g_virtio_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the device in device list */

  list_for_every(&g_virtio_bus.device, node)
    {
      FAR struct virtio_device_item_s *item =
        container_of(node, struct virtio_device_item_s, node);

      if (item->device == device)
        {
          /* Call driver remove */

          if (item->driver)
            {
              item->driver->remove(device);
            }

          /* Remove the device from the device list and free memory */

          list_delete(&item->node);
          kmm_free(item);
          break;
        }
    }

  nxmutex_unlock(&g_virtio_bus.lock);
  return ret;
}

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Name: virtio_addrenv_free_largest
 *
 * Description:
 *   Return the largest buffer to pass to virtio_addrenv_map() at once, for
 *   a transport whose device reaches guest physical memory directly.
 *
 * Input Parameters:
 *   vdev - The virtio device
 *
 * Returned Value:
 *   The largest mappable length in bytes.
 *
 ****************************************************************************/

size_t virtio_addrenv_free_largest(FAR struct virtio_device *vdev)
{
  /* A fixed limit: walking the heap here would take the heap lock on
   * every request, which fails in interrupt and panic context.
   */

  return VIRTIO_ADDRENV_MAP_MAX;
}

/****************************************************************************
 * Name: virtio_addrenv_map
 *
 * Description:
 *   Map a buffer for a transport whose device reaches guest physical memory
 *   directly. A user address is only virtually contiguous, while the device
 *   accesses len bytes from the physical address of its first byte, so it
 *   is bounced through the kernel heap. A kernel address is linearly
 *   mapped and used as is.
 *
 * Input Parameters:
 *   vdev - The virtio device
 *   buf  - The buffer to map
 *   len  - The buffer length in bytes
 *   dir  - The transfer direction
 *
 * Returned Value:
 *   The address to give the device, or NULL when out of memory.
 *
 ****************************************************************************/

FAR void *virtio_addrenv_map(FAR struct virtio_device *vdev, FAR void *buf,
                             size_t len, enum virtio_dma_map dir)
{
  FAR void *bounce;

  if (!up_addrenv_user_vaddr((uintptr_t)buf))
    {
      return buf;
    }

  bounce = kmm_malloc(len);
  if (bounce != NULL && dir != VIRTIO_DMA_FROM_DEVICE)
    {
      memcpy(bounce, buf, len);
    }

  return bounce;
}

/****************************************************************************
 * Name: virtio_addrenv_unmap
 *
 * Description:
 *   Release a buffer mapped by virtio_addrenv_map(), copying the data the
 *   device wrote back to the user buffer. Must run in the address
 *   environment the buffer was mapped in.
 *
 * Input Parameters:
 *   vdev   - The virtio device
 *   buf    - The buffer passed to virtio_addrenv_map()
 *   len    - The buffer length in bytes
 *   dir    - The transfer direction passed to virtio_addrenv_map()
 *   mapped - The address virtio_addrenv_map() returned
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void virtio_addrenv_unmap(FAR struct virtio_device *vdev, FAR void *buf,
                          size_t len, enum virtio_dma_map dir,
                          FAR void *mapped)
{
  if (mapped == buf)
    {
      return;
    }

  if (dir != VIRTIO_DMA_TO_DEVICE)
    {
      memcpy(buf, mapped, len);
    }

  kmm_free(mapped);
}

#endif /* CONFIG_ARCH_ADDRENV */
