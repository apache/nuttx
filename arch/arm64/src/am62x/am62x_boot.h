/****************************************************************************
 * arch/arm64/src/am62x/am62x_boot.h
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

#ifndef __ARCH_ARM64_SRC_AM62X_AM62X_BOOT_H
#define __ARCH_ARM64_SRC_AM62X_AM62X_BOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <arch/chip/chip.h>
#include "arm64_internal.h"
#include "arm64_arch.h"

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
 * Name: am62x_board_initialize
 *
 * Description:
 *   Board-specific initialization called from am62x_boot.c after the MMU
 *   and common peripherals are set up.  Board code in
 *   boards/arm64/am62x/<board>/src/ provides this function.
 *
 ****************************************************************************/

void am62x_board_initialize(void);

/****************************************************************************
 * Name: am62x_memory_initialize
 *
 * Description:
 *   Called very early, before .bss is zeroed or .data is copied, to
 *   perform any board-level memory initialization (e.g. DRAM training).
 *   For boards where U-Boot already initialises DRAM this is a no-op.
 *
 ****************************************************************************/

void am62x_memory_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_AM62X_AM62X_BOOT_H */
