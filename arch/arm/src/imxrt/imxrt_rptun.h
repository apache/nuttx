/****************************************************************************
 * arch/arm/src/imxrt/imxrt_rptun.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_RPTUN_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/rptun/rptun.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board-supplied description of the CM4 remote.  The CM7 is always the
 * rptun master: it loads the CM4 ELF (whose .resource_table carries the
 * vdev/vring description), fills in the vring addresses, then releases the
 * CM4 from reset.  Kicks are coalesced in the MU mailbox, so the remote
 * must read MU-B RR0 before it scans its virtqueues; a kick that arrives
 * while the mailbox is full is covered by that scan.
 */

struct imxrt_rptun_config_s
{
  const char *cpuname;    /* Remote name, e.g. "cm4" -> /dev/rptun/cm4 */
  const char *firmware;   /* ELF path passed to the rptun loader */

  /* CM4 device address -> CM7 physical address map, terminated by
   * an entry with size 0.  Must cover every ELF segment and the shared
   * memory region.
   */

  const struct rptun_addrenv_s *addrenv;

  uintptr_t boot_addr;    /* Value programmed into CM4_INIT_VTOR */
  bool autostart;         /* Start the remote from rptun_initialize() */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_rptun_init
 *
 * Description:
 *   Register the CM4 as an rptun device.  cfg must stay valid for the life
 *   of the system.  The board maps the memory the CM4 writes (vrings,
 *   rpmsg buffers) as non-cacheable before calling this.  The remote is
 *   not started; use RPTUNIOC_START on the character device or
 *   rptun_boot(cpuname).
 *
 ****************************************************************************/

int imxrt_rptun_init(const struct imxrt_rptun_config_s *cfg);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_RPTUN_H */
