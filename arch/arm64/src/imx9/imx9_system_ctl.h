/****************************************************************************
 * arch/arm64/src/imx9/imx9_system_ctl.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_SYSTEM_CTL_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_SYSTEM_CTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>
#include "arm64_internal.h"
#include "hardware/imx9_memorymap.h"

#define SRC_CTRL_OFFSET             0x10
#define SRC_SP_ISO_CTRL_OFFSET      0x10C
#define MEM_CTRL_OFFSET             0x4
#define SRC_SLICE_SW_CTRL_OFFSET    0x20
#define SRC_SLICE_FUNC_STAT_OFFSET  0xb4
#define SRC_SLICE_AUTHEN_CTRL_OFFSET   0x4
#define SRC_SLICE_PSW_ACK_CTRL0_OFFSET 0x80

#define SYS_CTR_CNTFID0             0x20
#define SYS_CTR_CNTCR               0x0

#define SC_CNTCR_ENABLE             0x1
#define SC_CNTCR_HDBG               0x2
#define SC_CNTCR_FREQ0              0x100
#define SC_CNTCR_FREQ1              0x200

#define AON_MIX_LP_HANDSAKE         0x110

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: imx9_mix_powerup
 *
 * Description:
 *  Powercycle ML and media mix and disable isolation
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_mix_powerup(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_SYSTEM_CTL_H */
