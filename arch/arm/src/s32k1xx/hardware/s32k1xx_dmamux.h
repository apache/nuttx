/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_dmamux.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_DMAMUX_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

#if defined(CONFIG_ARCH_CHIP_S32K11X)
#  include <hardware/s32k11x_dmamux.h>
#elif defined(CONFIG_ARCH_CHIP_S32K14X)
#  include <hardware/s32k14x_dmamux.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of DMA channels */

#if defined(CONFIG_ARCH_CHIP_S32K11X)
#  define S32K1XX_NDMACH  4
#elif defined(CONFIG_ARCH_CHIP_S32K14X)
#  define S32K1XX_NDMACH  16
#endif

/* DMAMUX Register Offsets **************************************************/

#define S32K1XX_DMAMUX_CHCFG_OFFSET(n)  (n)  /* Channel Configuration register, n=1..15 */

/* DMAMUX Register Addresses ************************************************/

#define S32K1XX_DMAMUX_CHCFG(n)         (S32K1XX_DMAMUX_BASE + S32K1XX_DMAMUX_CHCFG_OFFSET(n)) /* n=1..15 */

/* DMAMUX Register Bitfield Definitions *************************************/

/* Channel Configuration register, n=1..15 */

#define DMAMUX_CHCFG_SOURCE_SHIFT       (0)       /* Bits 0-5:  DMA Channel Source (Slot) */
#define DMAMUX_CHCFG_SOURCE_MASK        (0x3f << DMAMUX_CHCFG_SOURCE_SHIFT)
#  define DMAMUX_CHCFG_SOURCE(s)        ((uint32_t)(s) << DMAMUX_CHCFG_SOURCE_SHIFT) /* chip-specific */

#define DMAMUX_CHCFG_TRIG               (1 << 6)  /* Bit 6:  DMA Channel Trigger Enable */
#define DMAMUX_CHCFG_ENBL               (1 << 7)  /* Bit 7:  DMA Channel Enable */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_DMAMUX_H */
