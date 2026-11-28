/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/chip.h
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

#ifndef __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_CHIP_H
#define __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>
#include <arch/board/board.h>
#include "spacemit_k1x_rcpu_memorymap.h"
#include "hardware/spacemit_k1x_rcpu_eclic.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Return the base address of the ECLIC and of the private machine timer.
 *
 * Both live inside the Nuclei "IRegion", whose base address is not a compile
 * time constant: it is read from the core with CSR_MIRGB_INFO.  K1 does
 * report an IRegion, so these never fall back to the legacy Nuclei defaults
 * on this part; the fallback exists so that the same driver can serve a core
 * that does not implement IRegion.
 *
 * These are cheap: the value is read once and cached.
 */

extern uintptr_t spacemit_k1x_rcpu_systimer_base(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_CHIP_H */
