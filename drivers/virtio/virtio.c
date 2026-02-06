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
#include <strings.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
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
  mutex_t lock;                  /* Lock for the list */
  struct dq_queue_s device;      /* Wait match virtio device list */
  struct dq_queue_s driver;      /* VirtIO driver list */
};

struct virtio_device_item_s
{
  struct dq_entry_s     node;    /* list node */
  struct virtio_device *device;  /* Pointer to the virtio device */
  struct virtio_driver *driver;  /* Pointer to the virtio driver that
                                  * matched with current virtio device
                                  */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_dev_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_virtio_dev_ops =
{
  NULL,              /* open */
  NULL,              /* close */
  NULL,              /* read */
  NULL,              /* write */
  NULL,              /* seek */
  virtio_dev_ioctl,  /* ioctl */
};

static struct virtio_bus_s g_virtio_bus =
{
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_dump
 ****************************************************************************/

static void virtio_dump(FAR const char *name)
{
  FAR struct dq_entry_s *node;
  int count = 0;
  int ret;

  syslog(LOG_EMERG, "VirtIO devices:\n");

  name = name == NULL ? "all" : name;
  ret = nxmutex_lock(&g_virtio_bus.lock);
  if (ret < 0)
    {
      syslog(LOG_EMERG, "  Failed to acquire lock\n");
      return;
    }

  dq_for_every(&g_virtio_bus.device, node)
    {
      FAR struct virtio_device_item_s *item =
        container_of(node, struct virtio_device_item_s, node);
      FAR struct virtio_device *vdev = item->device;
      FAR const char *dev_name;
      unsigned int i;

      dev_name = virtio_dev_name(vdev->id.device);
      dev_name = dev_name == NULL ? "unknown" : dev_name;

      /* Apply type filter if specified, "all" means no filter */

      if (strcasecmp(name, "all") != 0 && strcasecmp(dev_name, name) != 0)
        {
          continue;
        }

      count++;

      syslog(LOG_EMERG, "  [%s] id=%" PRIu32 " vdev=%p driver=%p priv=%p\n",
             dev_name, vdev->id.device, vdev, item->driver, vdev->priv);

      /* Call driver-specific dump if available */

      if (item->driver && item->driver->dump)
        {
          item->driver->dump(vdev);
        }

      /* Dump virtqueue information */

      for (i = 0; i < vdev->vrings_num; i++)
        {
          FAR struct virtio_vring_info *vinfo = &vdev->vrings_info[i];

          virtqueue_dump(vinfo->vq);
        }
    }

  if (count == 0)
    {
      syslog(LOG_EMERG, "  No devices of type '%s' found\n", name);
    }

  nxmutex_unlock(&g_virtio_bus.lock);
}

/****************************************************************************
 * Name: virtio_dev_ioctl
 ****************************************************************************/

static int virtio_dev_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  int ret = -ENOTTY;

  switch (cmd)
    {
      case VIRTIOIOC_DUMP:
        virtio_dump((FAR const char *)(uintptr_t)arg);
        ret = OK;
        break;

      default:
        break;
    }

  return ret;
}

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

  /* Register virtio character device */

  ret = register_driver("/dev/virtio", &g_virtio_dev_ops, 0200, NULL);
  if (ret < 0)
    {
      vrterr("register_driver /dev/virtio failed, ret=%d\n", ret);
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
  FAR struct dq_entry_s *node;
  int ret;

  DEBUGASSERT(driver != NULL && driver->probe != NULL &&
              driver->remove != NULL);

  ret = nxmutex_lock(&g_virtio_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add the driver to the virtio_bus driver list */

  dq_addlast(&driver->node, &g_virtio_bus.driver);

  /* Match all the devices has registered in the virtio_bus */

  dq_for_every(&g_virtio_bus.device, node)
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
  FAR struct dq_entry_s *node;
  int ret;

  DEBUGASSERT(driver != NULL);

  ret = nxmutex_lock(&g_virtio_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find all the devices matched with driver in device list */

  dq_for_every(&g_virtio_bus.device, node)
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

  dq_rem(&driver->node, &g_virtio_bus.driver);

  nxmutex_unlock(&g_virtio_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: virtio_register_device
 ****************************************************************************/

int virtio_register_device(FAR struct virtio_device *device)
{
  FAR struct virtio_device_item_s *item;
  FAR struct dq_entry_s *node;
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

  dq_addlast(&item->node, &g_virtio_bus.device);

  /* Match the driver has registered in the virtio_bus */

  dq_for_every(&g_virtio_bus.driver, node)
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
  FAR struct dq_entry_s *node;
  int ret;

  ret = nxmutex_lock(&g_virtio_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the device in device list */

  dq_for_every(&g_virtio_bus.device, node)
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

          dq_rem(&item->node, &g_virtio_bus.device);
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
