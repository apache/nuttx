/****************************************************************************
 * boards/risc-v/spacemit-k1x-rcpu/muse-pi-pro/src/muse_pi_pro.h
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

#ifndef __BOARDS_RISCV_SPACEMIT_K1X_RCPU_MUSE_PI_PRO_SRC_MUSE_PI_PRO_H
#define __BOARDS_RISCV_SPACEMIT_K1X_RCPU_MUSE_PI_PRO_SRC_MUSE_PI_PRO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: spacemit_k1x_rcpu_bringup
 *
 * Description:
 *   Bring up the board's devices.  Called from board_late_initialize() once
 *   the driver framework is available.
 *
 ****************************************************************************/

extern int spacemit_k1x_rcpu_bringup(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_SPACEMIT_K1X_RCPU_MUSE_PI_PRO_SRC_MUSE_PI_PRO_H */
