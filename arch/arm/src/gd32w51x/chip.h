/****************************************************************************
 * arch/arm/src/gd32w51x/chip.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_CHIP_H
#define __ARCH_ARM_SRC_GD32W51X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#  include <arch/irq.h>
#endif

/* Include the chip capabilities file */

#include <arch/chip/chip.h>

#include "gd32w51x_memorymap.h"
#include "hardware/gd32w51x_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of external interrupt vectors, for the armv8-m vector table */

#define ARMV8M_PERIPHERAL_INTERRUPTS GD32W51X_IRQ_NEXTINT

#endif /* __ARCH_ARM_SRC_GD32W51X_CHIP_H */
