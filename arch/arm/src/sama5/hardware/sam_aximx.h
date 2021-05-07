/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_aximx.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_AXIMX_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_AXIMX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/sama5/chip.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AXIMX Register Offsets ***************************************************/

#define SAM_AXIMX_REMAP_OFFSET 0x0000 /* Remap Register */

/* AXIMX Register Addresses *************************************************/

#define SAM_AXIMX_REMAP        (SAM_AXIMX_VSECTION+SAM_AXIMX_REMAP_OFFSET)

/* AXIMX Register Bit Definitions *******************************************/

/* Remap Register
 *
 * Boot state:    ROM is seen at address 0x00000000
 * Remap State 0: SRAM is seen at address 0x00000000
 *                (through AHB slave interface)
 *                instead of ROM.
 * Remap State 1: HEBI is seen at address 0x00000000
 *                (through AHB slave interface)
 *                instead of ROM for external boot.
 */

#define AXIMX_REMAP_REMAP0     (1 << 0) /* Remap State 0 */

#ifdef ATSAMA5D3
#  define AXIMX_REMAP_REMAP1   (1 << 1) /* Remap State 1 */
#endif

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_AXIMX_H */
