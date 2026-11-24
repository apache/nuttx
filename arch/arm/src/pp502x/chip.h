/****************************************************************************
 * arch/arm/src/pp502x/chip.h
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

#ifndef __ARCH_ARM_SRC_PP502X_CHIP_H
#define __ARCH_ARM_SRC_PP502X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include "hardware/pp502x_memorymap.h"
#include "hardware/pp502x_cache.h"
#include "hardware/pp502x_intc.h"
#include "hardware/pp502x_remap.h"
#include "hardware/pp502x_sysctl.h"
#include "hardware/pp502x_timer.h"
#include "hardware/pp502x_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Iterations of the wait for the COP sleep bit in pp502x_head.S, about
 * 0.25 s at the 24 MHz entry clock for 23 cycles per iteration.
 */

#define PP502X_COP_WAIT_COUNT   0x40000

/* SDRAM size probe: two words, 32 MiB apart, in the top 64 KiB of the first
 * 32 MiB, which the image does not reach.  PP502X_SDRAM_PROBE_HIGH is read
 * back from the low word on a 32 MiB part.
 */

#define PP502X_SDRAM_PROBE_OFFSET   0x01fff000
#define PP502X_SDRAM_PROBE_LOW      0xa5a5a5a5
#define PP502X_SDRAM_PROBE_HIGH     0x5a5a5a5a

/* Bound on the wait for the PLL lock.  The time limit is the normal exit;
 * the poll count only covers a stuck USEC_TIMER.
 */

#define PP502X_PLL_LOCK_TIMEOUT_US  10000
#define PP502X_PLL_LOCK_MAX_POLLS   0x10000

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* SDRAM size in bytes, set from the probe by pp502x_head.S */

extern uint32_t g_pp502x_ramsize;

/****************************************************************************
 * Name: pp502x_cache_init
 *
 * Description:
 *   Turn the CPU cache on.  Called from pp502x_head.S before BSS is
 *   cleared, so it uses no static data.
 *
 ****************************************************************************/

void pp502x_cache_init(void);

/****************************************************************************
 * Name: pp502x_lowsetup
 *
 * Description:
 *   Called from pp502x_head.S before the OS starts, on the CPU only.
 *   Sets up the core clock and the console UART.
 *
 ****************************************************************************/

void pp502x_lowsetup(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_PP502X_CHIP_H */
