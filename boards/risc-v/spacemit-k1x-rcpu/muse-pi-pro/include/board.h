/****************************************************************************
 * boards/risc-v/spacemit-k1x-rcpu/muse-pi-pro/include/board.h
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

#ifndef __BOARDS_RISCV_SPACEMIT_K1X_RCPU_MUSE_PI_PRO_INCLUDE_BOARD_H
#define __BOARDS_RISCV_SPACEMIT_K1X_RCPU_MUSE_PI_PRO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking
 *
 * The RCPU does not own its clock tree.  The application processor brings
 * the audio subsystem and the RCPU clocks up before releasing this core, so
 * the frequencies below are not programmed here: they describe what the
 * board is expected to provide.  CONFIG_SPACEMIT_K1X_RCPU_SYSTIMER_FREQ and
 * CONFIG_SPACEMIT_K1X_RCPU_CONSOLE_CLOCK are the two values the port
 * actually consumes, and both carry their derivation in their Kconfig help.
 */

/* Data cache line size, used by the cache maintenance helpers */

#define BOARD_DCACHE_LINESIZE   32

/* Board identification */

#define BOARD_NAME              "muse-pi-pro"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: spacemit_k1x_rcpu_boardinitialize
 *
 * Description:
 *   Board level initialisation, called from the start up path before the
 *   operating system is brought up.
 *
 ****************************************************************************/

extern void spacemit_k1x_rcpu_boardinitialize(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_SPACEMIT_K1X_RCPU_MUSE_PI_PRO_INCLUDE_BOARD_H */
