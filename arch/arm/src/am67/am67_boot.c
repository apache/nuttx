/****************************************************************************
 * arch/arm/src/am67/am67_boot.c
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
#include <nuttx/init.h>
#include <nuttx/rptun/rptun.h>
#include <arch/board/board.h>

#include "am67_mpuinit.h"
#include "am67_pinmux.h"
#include "am67_rptun.h"
#include "arm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUM_VRINGS          (0x02)
#define RL_BUFFER_COUNT     (0x200)   /* RPMsg vring buffer count (512) */
#define NET_BUFFER_COUNT    (0x100)   /* virtio-net vring buffer count (256) */
#define VRING_ALIGN         (0x1000)
#define RESOURCE_TABLE_BASE (0xa2100000)

/* Vring device addresses are NOT fixed by the firmware.  Linux remoteproc
 * allocates each vring as a carveout from the R5F DMA pool (DT reserved
 * memory main-r5fss-dma-memory-region@a2000000, 1 MB) and cannot honor
 * addresses outside that pool: a fixed da merely produces "Allocated
 * carveout doesn't fit device address request" and the host uses its own
 * allocation while the device stares at empty memory.  FW_RSC_ADDR_ANY
 * asks the host to allocate and WRITE THE CHOSEN ADDRESS BACK into this
 * table before the R5F boots; rptun then reads the live table and attaches
 * to the real rings.  The DMA pool lies inside the non-cacheable MPU
 * window (am67_mpuinit.h), so coherency is preserved.
 */

#define FW_RSC_ADDR_ANY     (0xffffffffu)

/* Resource table has 2 entries: vdev[0]=RPMsg, vdev[1]=virtio-net */
#define NO_RESOURCE_ENTRIES (2)
#define RSC_VDEV_FEATURE_NS (1) /* Support name service announcement */
#define RSC_TABLE_VERSION   (1)

/* virtio device IDs (from virtio spec) */
#define VIRTIO_ID_NET_DEV   (1)  /* VIRTIO_ID_NETWORK */
#define VIRTIO_ID_RPMSG_DEV (7)  /* VIRTIO_ID_RPMSG */

/* notifyid assignments (must be unique across all resources):
 *   rpmsg_vring0: 0
 *   rpmsg_vring1: 1
 *   rpmsg_vdev:   2
 *   net_vring0:   3
 *   net_vring1:   4
 *   net_vdev:     5
 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Place extended resource table in special ELF section.
 * Linux remoteproc reads this section from the R5F firmware binary to
 * discover virtio devices and set up shared-memory vrings.
 *
 * Layout (all offsets from the start of this struct):
 *   [0] rpmsg_vdev  — RPMsg transport (VIRTIO_ID_RPMSG=7)
 *   [1] net_vdev    — virtio-net      (VIRTIO_ID_NETWORK=1)
 */

__attribute__((section(".resource_table")))
const struct am67_rsc_s g_am67_rsc_table =
{
  .base =
  {
    .rsc_tbl_hdr =
    {
      RSC_TABLE_VERSION,
      NO_RESOURCE_ENTRIES,
      {
        0, 0
      }
    },

    /* Offsets from the start of g_am67_rsc_table to each resource */

    .offset =
    {
      offsetof(struct am67_rsc_s, base.rpmsg_vdev),
      offsetof(struct am67_rsc_s, net_vdev),
    },

    .log_trace =
    {
      RSC_TRACE, 0, 0
    },

    .rpmsg_vdev = /* RPMsg virtio device entry */
    {
      RSC_VDEV,
      VIRTIO_ID_RPMSG_DEV,
      2,               /* notifyid */
      RSC_VDEV_FEATURE_NS,
      0,               /* gfeatures */
      0,               /* config_len */
      0,               /* status */
      NUM_VRINGS,
      {
        0, 0
      }
    },

    .rpmsg_vring0 =
    {
      FW_RSC_ADDR_ANY, /* da: host allocates and writes back */
      VRING_ALIGN,
      RL_BUFFER_COUNT,
      0,               /* notifyid */
      0                /* pa */
    },

    .rpmsg_vring1 =
    {
      FW_RSC_ADDR_ANY, /* da: host allocates and writes back */
      VRING_ALIGN,
      RL_BUFFER_COUNT,
      1,               /* notifyid */
      0                /* pa */
    },

    .config =
    {
      0
    }
  },

  /* virtio-net vdev entry — Linux creates a virtual Ethernet interface
   * backed by standard virtio_net.ko.  NuttX's virtio-net driver
   * (CONFIG_DRIVERS_VIRTIO_NET) handles the R5F side.
   */

  .net_vdev =
  {
    RSC_VDEV,
    VIRTIO_ID_NET_DEV,
    5,               /* notifyid */
    0,               /* dfeatures: Linux negotiates */
    0,               /* gfeatures */
    0,               /* config_len: no MAC address config for now */
    0,               /* status */
    NUM_VRINGS,
    {
      0, 0
    }
  },

  .net_vring0 =
  {
    FW_RSC_ADDR_ANY, /* da: host allocates and writes back */
    VRING_ALIGN,
    NET_BUFFER_COUNT,
    3,               /* notifyid */
    0                /* pa */
  },

  .net_vring1 =
  {
    FW_RSC_ADDR_ANY, /* da: host allocates and writes back */
    VRING_ALIGN,
    NET_BUFFER_COUNT,
    4,               /* notifyid */
    0                /* pa */
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 * Boot Sequence
 *
 *   1.  The __start entry point in armv7-r/arm_head.S is invoked upon power-
 *       on reset.
 *   2.  __start prepares CPU for code execution.
 *   3a. If CONFIG_ARMV7R_MEMINIT is not defined, then __start will prepare
 *       memory resources by calling arm_data_initialize() and will then
 *       branch this function.
 *   3b. Otherwise, this function will be called without having initialized
 *       memory resources!  We need to be very careful in this case.  Here,
 *       this function will call tms570_boardinitialize() which, among other
 *       things, must initialize SDRAM memory.  After initializatino of the
 *       memories, this function will call arm_data_initialize() to
 *       initialize the memory resources
 *   4.  This function will then branch to nx_start() to start the operating
 *       system.
 *
 ****************************************************************************/

void arm_boot(void)
{
  /* Configure the MPU to permit user-space access to its
   * ATCM, BTCM and DDR sections
   */

  am67_mpu_init();

  /* Do pinmux to get UART early */

  am67_pinmux_init();

  /* Then start NuttX */

  nx_start();
}

/****************************************************************************
 * Name: up_addrenv_pa_to_va / up_addrenv_va_to_pa
 *
 * The R5F has no MMU (only MPU), so physical == virtual.
 * OpenAMP / libmetal call these unconditionally; provide trivial stubs.
 ****************************************************************************/

FAR void *up_addrenv_pa_to_va(uintptr_t pa)
{
  return (FAR void *)pa;
}

uintptr_t up_addrenv_va_to_pa(FAR void *va)
{
  return (uintptr_t)va;
}
