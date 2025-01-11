/****************************************************************************
 * drivers/vhost/vhost.c
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

#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/vhost/vhost.h>

#include "vhost-rng.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VHOST_DEFERED_PROBE_PERIOD 100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_bus_s
{
  mutex_t          lock;           /* Lock for the list */
  struct list_node device;         /* Wait match vhost device list */
  struct list_node defered_device; /* Defered vhost device list */
  struct list_node driver;         /* Vhost driver list */
  struct work_s    defered_work;   /* Defered probe work */
};

struct vhost_device_item_s
{
  struct list_node     node;    /* List node */
  struct vhost_device *device;  /* Pointer to the vhost device */
  struct vhost_driver *driver;  /* Pointer to the vhost driver that
                                 * matched with current vhost device
                                 */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_bus_s g_vhost_bus =
{
  NXMUTEX_INITIALIZER,
  LIST_INITIAL_VALUE(g_vhost_bus.device),
  LIST_INITIAL_VALUE(g_vhost_bus.defered_device),
  LIST_INITIAL_VALUE(g_vhost_bus.driver),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_status_driver_ok
 ****************************************************************************/

static bool vhost_status_driver_ok(FAR struct vhost_device *hdev)
{
  bool driver_ok = false;
  uint8_t status;
  int ret;

  ret = vhost_get_status(hdev, &status);
  if (ret)
    {
      return driver_ok;
    }

  /* Busy wait until the remote is ready */

  if (status & VIRTIO_CONFIG_STATUS_NEEDS_RESET)
    {
      vhost_set_status(hdev, 0);
    }
  else if (status & VIRTIO_CONFIG_STATUS_DRIVER_OK)
    {
      driver_ok = true;
    }

  return driver_ok;
}

/****************************************************************************
 * Name: vhost_defered_probe_work
 ****************************************************************************/

static void vhost_defered_probe_work(FAR void *arg)
{
  FAR struct vhost_device_item_s *item;
  FAR struct vhost_device_item_s *tmp;
  FAR struct vhost_driver *driver;

  nxmutex_lock(&g_vhost_bus.lock);

  list_for_every_entry_safe(&g_vhost_bus.defered_device, item, tmp,
                            struct vhost_device_item_s, node)
    {
      if (!vhost_status_driver_ok(item->device))
        {
          vhosterr("device is not ok device=%p\n", item->device);
          continue;
        }

      /* Vhost device (virtio driver) status has been OK, move it to the
       * normal device list
       */

      list_delete(&item->node);
      list_add_tail(&g_vhost_bus.device, &item->node);
      list_for_every_entry(&g_vhost_bus.driver, driver,
                           struct vhost_driver, node)
        {
          if (item->device->id.device == driver->device)
            {
              item->device->priv = driver;
              if (driver->probe(item->device) >= 0)
                {
                  vhosterr("device probe success device=%p\n", item->device);
                  item->driver = driver;
                  break;
                }
            }
        }
    }

  if (!list_is_empty(&g_vhost_bus.defered_device))
    {
      work_queue(LPWORK, &g_vhost_bus.defered_work, vhost_defered_probe_work,
                 NULL, VHOST_DEFERED_PROBE_PERIOD);
    }

  nxmutex_unlock(&g_vhost_bus.lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_driver
 ****************************************************************************/

int vhost_register_driver(FAR struct vhost_driver *driver)
{
  FAR struct vhost_device_item_s *item;
  int ret;

  DEBUGASSERT(driver != NULL && driver->probe != NULL &&
              driver->remove != NULL);

  ret = nxmutex_lock(&g_vhost_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add the driver to the vhost_bus driver list */

  list_add_tail(&g_vhost_bus.driver, &driver->node);

  /* Match all the devices has registered in the vhost_bus */

  list_for_every_entry(&g_vhost_bus.device, item, struct vhost_device_item_s,
                       node)
    {
      if (item->driver == NULL && driver->device == item->device->id.device)
        {
          /* If found the device in the device list, call driver probe,
           * if probe success, assign item->driver to indicate the device
           * matched.
           */

          item->device->priv = driver;
          if (driver->probe(item->device) >= 0)
            {
              item->driver = driver;
            }
        }
    }

  nxmutex_unlock(&g_vhost_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: vhost_unregister_driver
 ****************************************************************************/

int vhost_unregister_driver(FAR struct vhost_driver *driver)
{
  FAR struct vhost_device_item_s *item;
  int ret;

  DEBUGASSERT(driver != NULL);

  ret = nxmutex_lock(&g_vhost_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find all the devices matched with driver in device list */

  list_for_every_entry(&g_vhost_bus.device, item, struct vhost_device_item_s,
                       node)
    {
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

  nxmutex_unlock(&g_vhost_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: vhost_register_device
 ****************************************************************************/

int vhost_register_device(FAR struct vhost_device *device)
{
  FAR struct vhost_device_item_s *item;
  FAR struct vhost_driver *driver;
  int ret;

  item = kmm_zalloc(sizeof(*item));
  if (item == NULL)
    {
      return -ENOMEM;
    }

  item->device = device;

  ret = nxmutex_lock(&g_vhost_bus.lock);
  if (ret < 0)
    {
      kmm_free(item);
      return ret;
    }

  /* 1. Add device to defered device list if virtio driver not OK;
   * 2. Add device to the normal device list and try to probe the driver
   *    if virtio driver has been OK.
   */

  if (!vhost_status_driver_ok(device))
    {
      list_add_tail(&g_vhost_bus.defered_device, &item->node);
      if (list_is_singular(&g_vhost_bus.defered_device))
        {
          work_queue(LPWORK, &g_vhost_bus.defered_work,
                      vhost_defered_probe_work, NULL,
                      VHOST_DEFERED_PROBE_PERIOD);
        }
    }
  else
    {
      list_add_tail(&g_vhost_bus.device, &item->node);

      /* Match the driver has registered in the vhost_bus */

      list_for_every_entry(&g_vhost_bus.driver, driver, struct vhost_driver,
                           node)
        {
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
                }

              break;
            }
        }
    }

  nxmutex_unlock(&g_vhost_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: vhost_unregister_device
 ****************************************************************************/

int vhost_unregister_device(FAR struct vhost_device *device)
{
  FAR struct vhost_device_item_s *item;
  int ret;

  ret = nxmutex_lock(&g_vhost_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the device in device list */

  list_for_every_entry(&g_vhost_bus.device, item,
                       struct vhost_device_item_s, node)
    {
      if (item->device == device)
        {
          /* Call driver remove */

          if (item->driver)
            {
              item->driver->remove(item->device);
            }

          /* Remove the device from the device list and free memory */

          list_delete(&item->node);
          kmm_free(item);
          goto out;
        }
    }

  list_for_every_entry(&g_vhost_bus.defered_device, item,
                       struct vhost_device_item_s, node)
    {
      if (item->device == device)
        {
          list_delete(&item->node);
          kmm_free(item);
          goto out;
        }
    }

out:
  nxmutex_unlock(&g_vhost_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: vhost_register_drivers
 ****************************************************************************/

void vhost_register_drivers(void)
{
  struct metal_init_params params = METAL_INIT_DEFAULTS;
  int ret;

  ret = metal_init(&params);
  if (ret < 0)
    {
      vhosterr("metal_init failed, ret=%d\n", ret);
    }

#ifdef CONFIG_DRIVERS_VHOST_RNG
  ret = vhost_register_rng_driver();
  if (ret < 0)
    {
      vhosterr("vhost_register_rng_driver failed, ret=%d\n", ret);
    }
#endif

  UNUSED(ret);
}
