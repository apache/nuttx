/****************************************************************************
 * arch/arm/src/am67/am67_rptun.h
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

/* Extended resource table and rptun init for AM67A / J722S R5F.
 * Declares struct am67_rsc_s (RPMsg vdev + virtio-net vdev) and
 * am67_rptun_init(), which must be called from am67_bringup().
 *
 */

#ifndef __ARCH_ARM_SRC_AM67_AM67_RPTUN_H
#define __ARCH_ARM_SRC_AM67_AM67_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RPTUN

#include <nuttx/rptun/rptun.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CPU name that Linux remoteproc reports for this R5F core.
 * Must match the rproc->name in the Linux ti_k3_r5_remoteproc driver
 * (typically derived from DTS "label" or "compatible" + index).
 * Override with CONFIG_AM67_RPTUN_CPUNAME if needed.
 */

#ifndef AM67_RPTUN_CPUNAME
#  define AM67_RPTUN_CPUNAME "r5f"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Extended resource table: standard RPMsg vdev (id=7) at index 0 +
 * virtio-net vdev (id=1) at index 1.  The table is placed in the
 * .resource_table ELF section at 0xA2100000 so Linux remoteproc can
 * read it from the firmware binary.
 */

struct aligned_data(8) am67_rsc_s
{
  struct rptun_rsc_s       base;        /* RPMsg vdev (index 0) */
  struct fw_rsc_vdev       net_vdev;    /* virtio-net vdev (index 1, id=1) */
  struct fw_rsc_vdev_vring net_vring0;  /* net TX vring */
  struct fw_rsc_vdev_vring net_vring1;  /* net RX vring */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Defined in am67_boot.c, placed in .resource_table section */

extern const struct am67_rsc_s g_am67_rsc_table;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am67_rptun_init
 *
 * Description:
 *   Initialise the AM67 rptun device.  Enables the NAVSS mailbox interrupt
 *   (mailbox0_cluster3/user-3/FIFO-1, VIM IRQ 116) and calls
 *   rptun_initialize().  Must be called from am67_bringup() after basic
 *   platform initialisation.
 *
 ****************************************************************************/

int am67_rptun_init(void);

#endif /* CONFIG_RPTUN */
#endif /* __ARCH_ARM_SRC_AM67_AM67_RPTUN_H */
