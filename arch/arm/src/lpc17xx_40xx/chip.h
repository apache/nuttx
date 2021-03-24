/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/chip.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_CHIP_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nvic.h"

/* Include the chip capabilities file */

#include <arch/lpc17xx_40xx/chip.h>

/* Include the chip interrupt definition file */

#include <arch/lpc17xx_40xx/irq.h>

/* Include the memory map file.  Other chip hardware files should then
 * include this file for the proper setup.
 */

#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Provide the required number of peripheral interrupt vector definitions as
 * well. The definition LPC17_40_IRQ_NEXTINT simply comes from the
 * chip-specific IRQ header file included by arch/lpc17xx_40xx/irq.h.
 */

#define ARMV7M_PERIPHERAL_INTERRUPTS  LPC17_40_IRQ_NEXTINT

/* Vector Table Offset Register (VECTAB).  Redefine the mask defined in
 * arch/arm/src/armv7-m/nvic.h; The LPC178x/7x User manual definitions
 * do not match the ARMv7M field definitions.  Any bits set above bit
 * 29 would be an error and apparently the register wants 8- not 6-bit
 * alignment.
 */

#undef  NVIC_VECTAB_TBLOFF_MASK
#define NVIC_VECTAB_TBLOFF_MASK         (0x3fffff00)

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_CHIP_H */
