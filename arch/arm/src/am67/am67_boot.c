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
#include "arm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUM_VRINGS          (0x02)
#define RL_BUFFER_COUNT     (0x200)
#define VRING_ALIGN         (0x1000)
#define VRING_SIZE          (0x8000)
#define VDEV0_VRING_BASE    (0xa2200000)
#define RESOURCE_TABLE_BASE (0xa2100000)

#define NO_RESOURCE_ENTRIES (1)
#define RSC_VDEV_FEATURE_NS (1) /* Support name service announcement */
#define RSC_TABLE_VERSION   (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Place resource table in special ELF section */

__attribute__ ((section(".resource_table")))
const struct rptun_rsc_s g_am67_rsc_table =
{
  .rsc_tbl_hdr =
  {
    RSC_TABLE_VERSION,
    NO_RESOURCE_ENTRIES,
    {
      0, 0
    }
  },

  .offset =
  {
    offsetof(struct rptun_rsc_s, rpmsg_vdev)
  },

  .log_trace =
  {
    RSC_TRACE, 0, 0
  },

  .rpmsg_vdev = /* SRTM virtio device entry */
  {
    RSC_VDEV,
    7,
    2,
    RSC_VDEV_FEATURE_NS,
    0,
    0,
    0,
    NUM_VRINGS,
    {
      0, 0
    }
  },

  .rpmsg_vring0 =
  {
    VDEV0_VRING_BASE,
    VRING_ALIGN,
    RL_BUFFER_COUNT,
    0,
    0
  },

  .rpmsg_vring1 =
  {
    VDEV0_VRING_BASE + VRING_SIZE,
    VRING_ALIGN,
    RL_BUFFER_COUNT,
    1,
    0
  },

  .config =
  {
    0
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
