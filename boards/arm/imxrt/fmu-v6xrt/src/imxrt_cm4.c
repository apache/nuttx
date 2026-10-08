/****************************************************************************
 * boards/arm/imxrt/fmu-v6xrt/src/imxrt_cm4.c
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

#include <nuttx/rptun/rptun.h>

#include "arm_internal.h"
#include "mpu.h"
#include "imxrt_rptun.h"
#include "hardware/imxrt_memorymap.h"
#include "fmu-v6xrt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The CM4 tightly coupled memories as the CM4 sees them (device addresses)
 * and as the CM7 reaches them through the LMEM backdoor window (physical
 * addresses).  Only the code TCM half of the window is reachable, so the
 * CM4 image, its resource table, the vrings and the rpmsg buffers all live
 * there: code in the lower 64 KB, shared memory in the upper 64 KB.  The
 * CM4 boots from the backdoor alias; the CM4-native address does not boot.
 */

#define CM4_CODE_TCM_DA     0x1ffe0000
#define CM4_SYS_TCM_DA      0x20000000
#define CM4_TCM_SIZE        (128 * 1024)
#define CM4_CODE_TCM_PA     IMXRT_OCRAM_M4_BASE
#define CM4_SYS_TCM_PA      (IMXRT_OCRAM_M4_BASE + CM4_TCM_SIZE)
#define CM4_SHM_PA          (CM4_CODE_TCM_PA + (64 * 1024))
#define CM4_SHM_SIZE        (64 * 1024)

#ifdef CONFIG_FMU_V6XRT_CM4_ROMFS
#  define CM4_FIRMWARE      "/etc/cm4.elf"
#else
#  define CM4_FIRMWARE      CONFIG_FMU_V6XRT_CM4_FIRMWARE
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_addrenv_s g_cm4_addrenv[] =
{
  { .pa = CM4_CODE_TCM_PA, .da = CM4_CODE_TCM_DA, .size = CM4_TCM_SIZE },
  { .pa = CM4_SYS_TCM_PA,  .da = CM4_SYS_TCM_DA,  .size = CM4_TCM_SIZE },
  { .size = 0 },
};

static const struct imxrt_rptun_config_s g_cm4_config =
{
  .cpuname   = "cm4",
  .firmware  = CM4_FIRMWARE,
  .addrenv   = g_cm4_addrenv,
  .boot_addr = CM4_CODE_TCM_PA,
  .autostart = false,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_cm4_initialize
 *
 * Description:
 *   Register the CM4 as /dev/rptun/cm4.  The CM4 writes the vrings and the
 *   rpmsg buffers, so the CM7 must not read them through its D-cache: map
 *   the shared window Normal, non-cacheable and shareable.  The region is
 *   added after imxrt_mpu_initialize() and therefore wins over the
 *   cacheable OCRAM_M4 region it overlaps.
 *
 ****************************************************************************/

int imxrt_cm4_initialize(void)
{
  mpu_configure_region(CM4_SHM_PA, CM4_SHM_SIZE,
                       MPU_RASR_AP_RWRW | MPU_RASR_TEX_NOR |
                       MPU_RASR_S | MPU_RASR_XN);

  return imxrt_rptun_init(&g_cm4_config);
}
