/****************************************************************************
 * include/nuttx/drivers/qemu_fwcfg.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_QEMU_FWCFG_H
#define __INCLUDE_NUTTX_DRIVERS_QEMU_FWCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: fw_cfg_register
 *
 * Description:
 *   Bind a discovered QEMU fw_cfg MMIO region to a character device node.
 *
 *   The device is verified (SIGNATURE item must read "QEMU") before the
 *   node is created.  Once registered, the named "file" items published by
 *   QEMU (e.g. "-fw_cfg name=opt/xxx,string=...") can be read from user
 *   space using the standard configdata ioctl interface (see
 *   include/nuttx/mtd/configdata.h): CFGDIOC_GETCONFIG, CFGDIOC_FINDCONFIG,
 *   CFGDIOC_FIRSTCONFIG and CFGDIOC_NEXTCONFIG.  The device is read-only, so
 *   CFGDIOC_SETCONFIG and CFGDIOC_DELCONFIG report -ENOSYS.
 *
 * Input Parameters:
 *   base - The MMIO base address of the fw_cfg register block.
 *   path - The device node path to create (e.g. "/dev/qemufwcfg").
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure:
 *     -ENODEV  the region does not carry a valid fw_cfg signature.
 *     -ENOMEM  out of memory.
 *
 ****************************************************************************/

int fw_cfg_register(uintptr_t base, FAR const char *path);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_QEMU_FWCFG_H */
