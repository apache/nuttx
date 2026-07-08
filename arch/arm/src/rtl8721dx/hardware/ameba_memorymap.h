/****************************************************************************
 * arch/arm/src/rtl8721dx/hardware/ameba_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RTL8721DX_HARDWARE_AMEBA_MEMORYMAP_H
#define __ARCH_ARM_SRC_RTL8721DX_HARDWARE_AMEBA_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTL8721Dx KM4 application-core memory map.
 *
 * NuttX runs from the KM4 image2 SRAM window; .text/.data/.bss and the heap
 * live in this single SRAM region.  The flash is a SPI NOR reached over the
 * SDK XIP path and shared with the KM0 network processor.
 */

/* SRAM (288KB) */

#define AMEBA_SRAM_START   0x20020000
#define AMEBA_SRAM_SIZE    0x00048000

#define PRIMARY_RAM_START  AMEBA_SRAM_START
#define PRIMARY_RAM_SIZE   AMEBA_SRAM_SIZE
#define PRIMARY_RAM_END    (PRIMARY_RAM_START + PRIMARY_RAM_SIZE)

#endif /* __ARCH_ARM_SRC_RTL8721DX_HARDWARE_AMEBA_MEMORYMAP_H */
