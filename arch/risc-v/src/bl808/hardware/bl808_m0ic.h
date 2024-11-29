/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_m0ic.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_M0IC_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_M0IC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define BL808_M0IC_STATUS_OFFSET(n) (0x00 + 4 * (n))
#define BL808_M0IC_MASK_OFFSET(n)   (0x08 + 4 * (n))
#define BL808_M0IC_CLEAR_OFFSET(n)  (0x10 + 4 * (n))

/* Register locations */

#define BL808_M0IC_STATUS(n) BL808_M0IC_BASE + BL808_M0IC_STATUS_OFFSET(n)
#define BL808_M0IC_MASK(n)   BL808_M0IC_BASE + BL808_M0IC_MASK_OFFSET(n)
#define BL808_M0IC_CLEAR(n)  BL808_M0IC_BASE + BL808_M0IC_CLEAR_OFFSET(n)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_M0IC_H */
