/****************************************************************************
 * drivers/devicetree/fdt_qemu_fwcfg.c
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

#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/fdt.h>
#include <nuttx/drivers/qemu_fwcfg.h>

#include <libfdt.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_qemu_fwcfg_register
 *
 * Description:
 *   Discover the QEMU fw_cfg device from the device tree and register it as
 *   a character device.  Unlike virtio-mmio, fw_cfg is a system-wide
 *   singleton, so only the first matching node is registered.
 *
 * Input Parameters:
 *   fdt - Device tree handle
 *
 * Returned Value:
 *   Return 0 if success, negative if failed.
 *
 ****************************************************************************/

int fdt_qemu_fwcfg_register(FAR const void *fdt)
{
  uintptr_t base;
  int offset;

  offset = fdt_node_offset_by_compatible(fdt, -1, "qemu,fw-cfg-mmio");
  if (offset < 0)
    {
      return -ENODEV;
    }

  base = fdt_get_reg_base(fdt, offset, 0);
  if (base == 0)
    {
      return -EINVAL;
    }

  return fw_cfg_register(base, "/dev/qemufwcfg");
}
