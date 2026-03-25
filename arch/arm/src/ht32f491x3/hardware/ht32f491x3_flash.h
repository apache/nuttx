/****************************************************************************
 * arch/arm/src/ht32f491x3/hardware/ht32f491x3_flash.h
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_FLASH_H
#define __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ht32f491x3_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HT32_FLASH_PSR_OFFSET           0x000

#define HT32_FLASH_PSR                  (HT32_FLASHREG_BASE + HT32_FLASH_PSR_OFFSET)

#define HT32_FLASH_PSR_PROGRAM(wtcyc)   (0x150u | (wtcyc))

#define HT32_FLASH_WAIT_CYCLE_0         0x00
#define HT32_FLASH_WAIT_CYCLE_1         0x01
#define HT32_FLASH_WAIT_CYCLE_2         0x02
#define HT32_FLASH_WAIT_CYCLE_3         0x03
#define HT32_FLASH_WAIT_CYCLE_4         0x04

#endif /* __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_FLASH_H */
