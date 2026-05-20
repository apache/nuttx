/****************************************************************************
 * arch/arm64/src/am62x/chip.h
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

#ifndef __ARCH_ARM64_SRC_AM62X_CHIP_H
#define __ARCH_ARM64_SRC_AM62X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#endif

/* All GIC addresses and memory-map constants live in the include-layer
 * chip.h (arch/arm64/include/am62x/chip.h) which is pulled in via
 * <arch/chip/chip.h>.  Nothing extra needed here.
 *
 * The hardware register headers are in hardware/ and are included
 * directly by the driver files that need them.
 */

/* Number of GIC SPI target registers for AM62x (480 SPIs / 32 = 15) */

#define CONFIG_NUM_GIC_ITARGETS_REGS  15

#endif /* __ARCH_ARM64_SRC_AM62X_CHIP_H */
