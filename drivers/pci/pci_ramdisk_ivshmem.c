/****************************************************************************
 * drivers/pci/pci_ramdisk_ivshmem.c
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/drivers/ramdisk.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci_ivshmem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RAMDISK_SECTOR_SIZE 512

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ramdisk_ivshmem_probe(FAR struct ivshmem_device_s *dev);
static void ramdisk_ivshmem_remove(FAR struct ivshmem_device_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramdisk_ivshmem_probe
 ****************************************************************************/

static int ramdisk_ivshmem_probe(FAR struct ivshmem_device_s *dev)
{
  FAR struct ivshmem_driver_s *drv = ivshmem_get_driver(dev);
  struct ramdisk_config_s config;
  size_t buffer_size;
  char name[32];
  int ret;

  snprintf(name, sizeof(name), "/dev/ivshm%d", drv->id);
  config.name     = name;
  config.buffer   = ivshmem_get_shmem(dev, &buffer_size);
  config.nsectors = buffer_size / RAMDISK_SECTOR_SIZE;
  config.sectsize = RAMDISK_SECTOR_SIZE;
  config.mode     = 0666;
  config.rdflags  = RDFLAG_WRENABLED;
  ret = ramdisk_register_with_config(&config);
  if (ret < 0)
    {
      pcierr("ERROR: ramdisk ivshmem probe failed %d\n", ret);
      return ret;
    }

  pciinfo("Ramdisk ivshmem %s buffer: %p size: %zu sectors: %" PRIu32 "\n",
          name, config.buffer, buffer_size, config.nsectors);
  return ret;
}

/****************************************************************************
 * Name: ramdisk_ivshmem_remove
 ****************************************************************************/

static void ramdisk_ivshmem_remove(FAR struct ivshmem_device_s *dev)
{
  pcierr("ERROR: ramdisk ivshmem remove not support for now\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pci_register_ramdisk_ivshmem_driver(void)
{
  FAR struct ivshmem_driver_s *drv;
  FAR char *start = CONFIG_PCI_RAMDISK_IVSHMEM_IDTABLE;

  do
    {
      drv = kmm_zalloc(sizeof(*drv));
      if (drv == NULL)
        {
          return -ENOMEM;
        }

      drv->id = strtoul(start, &start, 0);
      drv->probe = ramdisk_ivshmem_probe;
      drv->remove = ramdisk_ivshmem_remove;
      if (ivshmem_register_driver(drv) < 0)
        {
          kmm_free(drv);
        }

      pciinfo("Register ramdisk ivshmem driver, id=%d\n", drv->id);
    }
  while (*start++ != '\0');

  return 0;
}
