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

#include "qemu-armv8a.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QEMU_VIRTIO_MMIO_BASE    0x0a000000
#define QEMU_VIRTIO_MMIO_REGSIZE 0x200
#define QEMU_VIRTIO_MMIO_IRQ     48
#define QEMU_VIRTIO_MMIO_NUM     4

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_virtio_register_mmio_devices
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
static void qemu_virtio_register_mmio_devices(void)
{
  uintptr_t virtio = (uintptr_t)QEMU_VIRTIO_MMIO_BASE;
  size_t size = QEMU_VIRTIO_MMIO_REGSIZE;
  int irq = QEMU_VIRTIO_MMIO_IRQ;
  int i;

  for (i = 0; i < QEMU_VIRTIO_MMIO_NUM; i++)
    {
      virtio_register_mmio_device((FAR void *)(virtio + size * i), irq + i);
    }
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

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
  qemu_virtio_register_mmio_devices();
#endif

  UNUSED(ret);
  return OK;
}
