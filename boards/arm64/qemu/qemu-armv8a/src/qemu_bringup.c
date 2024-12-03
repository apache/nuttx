/****************************************************************************
 * boards/arm64/qemu/qemu-armv8a/src/qemu_bringup.c
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
#include <nuttx/virtio/virtio-mmio.h>
#include <nuttx/fdt.h>
#include <nuttx/pci/pci_ecam.h>

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

#define FDT_PCI_TYPE_IO              0x01000000
#define FDT_PCI_TYPE_MEM32           0x02000000
#define FDT_PCI_TYPE_MEM64           0x03000000
#define FDT_PCI_TYPE_MASK            0x03000000
#define FDT_PCI_PREFTCH              0x40000000

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

      addr = fdt_get_reg_base(fdt, offset, 0);
      irqnum = fdt_get_irq(fdt, offset, 1, QEMU_SPI_IRQ_BASE);
      if (addr > 0 && irqnum >= 0)
        {
          virtio_register_mmio_device((void *)addr, irqnum);
        }
    }
}

#endif

/****************************************************************************
 * Name: register_pci_host_from_fdt
 ****************************************************************************/

#ifdef CONFIG_PCI
static void register_pci_host_from_fdt(const void *fdt)
{
  struct pci_resource_s prefetch;
  struct pci_resource_s cfg;
  struct pci_resource_s mem;
  struct pci_resource_s io;
  const fdt32_t *ranges;
  int offset;

  /* #address-size must be 3
   * defined in the PCI Bus Binding to IEEE Std 1275-1994 :
   * Bit#
   *
   * phys.hi cell:  npt000ss bbbbbbbb dddddfff rrrrrrrr
   * phys.mid cell: hhhhhhhh hhhhhhhh hhhhhhhh hhhhhhhh
   * phys.lo cell:  llllllll llllllll llllllll llllllll
   */

  const int na = 3;

  /* #size-cells must be 2 */

  const int ns = 2;
  int rlen;
  int pna;

  memset(&prefetch, 0, sizeof(prefetch));
  memset(&cfg, 0, sizeof(cfg));
  memset(&mem, 0, sizeof(mem));
  memset(&io, 0, sizeof(io));

  offset = fdt_node_offset_by_compatible(fdt, -1,
                                         "pci-host-ecam-generic");
  if (offset < 0)
    {
      return;
    }

  /* Get the reg address, 64 or 32 */

  cfg.start = fdt_get_reg_base(fdt, offset);
  cfg.end = cfg.start + fdt_get_reg_size(fdt, offset);

  /* Get the ranges address */

  ranges = fdt_getprop(fdt, offset, "ranges", &rlen);
  if (ranges < 0)
    {
      return;
    }

  pna = fdt_get_parent_address_cells(fdt, offset);

  for (rlen /= 4; (rlen -= na + pna + ns) >= 0; ranges += na + pna + ns)
    {
      uint32_t type = fdt32_ld(ranges);

      if ((type & FDT_PCI_TYPE_MASK) == FDT_PCI_TYPE_IO)
        {
          io.start = fdt_ld_by_cells(ranges + na, pna);
          io.end = io.start + fdt_ld_by_cells(ranges + na + pna, ns);
        }
      else if ((type & FDT_PCI_PREFTCH) == FDT_PCI_PREFTCH)
        {
          prefetch.start = fdt_ld_by_cells(ranges + na, pna);
          prefetch.end = prefetch.start +
                         fdt_ld_by_cells(ranges + na + pna, ns);
        }
      else
        {
          mem.start = fdt_ld_by_cells(ranges + na, pna);
          mem.end = mem.start + fdt_ld_by_cells(ranges + na + pna, ns);
        }
    }

  pci_ecam_register(&cfg, &io, &mem, NULL);
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

#ifdef CONFIG_PCI
  register_pci_host_from_fdt(fdt);
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
