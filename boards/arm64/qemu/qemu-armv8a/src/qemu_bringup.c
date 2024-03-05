/****************************************************************************
 * boards/arm64/qemu/qemu-armv8a/src/qemu_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/virtio/virtio-mmio.h>
#include <nuttx/fdt.h>

#ifdef CONFIG_LIBC_FDT
#  include <libfdt.h>
#endif

#include "qemu-armv8a.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef QEMU_SPI_IRQ_BASE
#define QEMU_SPI_IRQ_BASE     32
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_LIBC_FDT) && defined(CONFIG_DEVICE_TREE)
#ifdef CONFIG_DRIVERS_VIRTIO_MMIO

/****************************************************************************
 * Name: register_virtio_devices_from_fdt
 ****************************************************************************/

static void register_virtio_devices_from_fdt(const void *fdt)
{
  uintptr_t addr;
  int offset = -1;
  int irqnum;

  for (; ; )
    {
      offset = fdt_node_offset_by_compatible(fdt, offset, "virtio,mmio");
      if (offset == -FDT_ERR_NOTFOUND)
        {
          break;
        }

      addr = fdt_get_reg_base(fdt, offset);
      irqnum = fdt_get_irq(fdt, offset, 1, QEMU_SPI_IRQ_BASE);
      if (addr > 0 && irqnum >= 0)
        {
          virtio_register_mmio_device((void *)addr, irqnum);
        }
    }
}

#endif

/****************************************************************************
 * Name: register_devices_from_fdt
 ****************************************************************************/

static void register_devices_from_fdt(void)
{
  const void *fdt = fdt_get();

  if (fdt == NULL)
    {
      return;
    }

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
  register_virtio_devices_from_fdt(fdt);
#endif
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int qemu_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmp file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at /tmp: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_LIBC_FDT) && defined(CONFIG_DEVICE_TREE)
  register_devices_from_fdt();
#endif

  UNUSED(ret);
  return OK;
}
