/****************************************************************************
 * arch/arm/src/kinetis/chip.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_CHIP_H
#define __ARCH_ARM_SRC_KINETIS_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the memory map and the chip definitions file.
 * Other chip hardware files should then include this file for the proper
 * setup.
 */

#include <arch/irq.h>
#include <arch/kinetis/chip.h>
#include "hardware/kinetis_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If the common ARMv7-M vector handling logic is used, then it expects the
 * following definition in this file that provides the number of supported
 * external interrupts which, for this architecture, is provided in the
 * arch/kinetis/chip.h header file.
 */

#define ARMV7M_PERIPHERAL_INTERRUPTS  KINETIS_IRQ_NEXTINTS

#endif /* __ARCH_ARM_SRC_KINETIS_CHIP_H */
