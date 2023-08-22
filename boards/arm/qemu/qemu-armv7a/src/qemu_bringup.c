/****************************************************************************
 * boards/arm/qemu/qemu-armv7a/src/qemu_bringup.c
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
#include <libfdt.h>

#include "qemu-armv7a.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QEMU_SPI_IRQ_BASE            32

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_LIBC_FDT) && defined(CONFIG_DEVICE_TREE)

/****************************************************************************
 * Name: fdt_get_irq
 *
 * Description:
 *   Only can be use when the corresponding node's parent interrupt
 *   controller is intc node.
 *
 ****************************************************************************/

static int unused_code
fdt_get_irq(const void *fdt, int offset)
{
  const fdt32_t *pv;
  int irq = -ENOENT;

  pv = fdt_getprop(fdt, offset, "interrupts", NULL);
  if (pv != NULL)
    {
      irq = fdt32_ld(pv + 1) + QEMU_SPI_IRQ_BASE;
    }

  return irq;
}

/****************************************************************************
 * Name: fdt_get_irq_by_path
 *
 * Description:
 *   Only can be use when the corresponding node's parent interrupt
 *   controller is intc node.
 *
 ****************************************************************************/

static int unused_code
fdt_get_irq_by_path(const void *fdt, const char *path)
{
  return fdt_get_irq(fdt, fdt_path_offset(fdt, path));
}

/****************************************************************************
 * Name: fdt_get_reg_base
 ****************************************************************************/

static uintptr_t unused_code
fdt_get_reg_base(const void *fdt, int offset)
{
  const void *reg;
  uintptr_t addr = 0;
  int parentoff;

  parentoff = fdt_parent_offset(fdt, offset);
  if (parentoff < 0)
    {
      return addr;
    }

  reg = fdt_getprop(fdt, offset, "reg", NULL);
  if (reg != NULL)
    {
      if (fdt_address_cells(fdt, parentoff) == 2)
        {
          addr = fdt64_ld(reg);
        }
      else
        {
          addr = fdt32_ld(reg);
        }
    }

  return addr;
}

/****************************************************************************
 * Name: fdt_get_reg_base_by_path
 ****************************************************************************/

static uintptr_t unused_code
fdt_get_reg_base_by_path(const void *fdt, const char *path)
{
  return fdt_get_reg_base(fdt, fdt_path_offset(fdt, path));
}

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
      irqnum = fdt_get_irq(fdt, offset);
      if (addr > 0 && irqnum >= 0)
        {
          virtio_register_mmio_device((FAR void *)addr, irqnum);
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
