/****************************************************************************
 * boards/arm/qemu/qemu-armv7a/src/qemu_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fdt.h>

#ifdef CONFIG_LIBC_FDT
#  include <libfdt.h>
#endif

#include "chip.h"
#include "qemu-armv7a.h"

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

/****************************************************************************
 * Name: register_devices_from_fdt
 ****************************************************************************/

static void register_devices_from_fdt(void)
{
  const void *fdt = fdt_get();
  int ret;

  if (fdt == NULL)
    {
      return;
    }

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
  ret = fdt_virtio_mmio_devices_register(fdt, QEMU_SPI_IRQ_BASE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "fdt_virtio_mmio_devices_register failed, ret=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_PCI
  ret = fdt_pci_ecam_register(fdt);
  if (ret < 0)
    {
      syslog(LOG_ERR, "fdt_pci_ecam_register failed, ret=%d\n", ret);
    }
#endif

  UNUSED(ret);
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
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
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
