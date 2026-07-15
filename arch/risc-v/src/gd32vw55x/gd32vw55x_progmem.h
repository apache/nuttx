/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_progmem.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PROGMEM_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PROGMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/gd32vw55x_fmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The region of the internal flash that is exposed through the progmem
 * interface is defined by the board:
 *
 *   CONFIG_GD32VW55X_PROGMEM_START_ADDR - Absolute address of the first
 *     byte of the progmem region.  Must be aligned to a 4 KiB page.
 *   CONFIG_GD32VW55X_PROGMEM_SIZE - Size of the progmem region in bytes.
 *     Must be a multiple of the 4 KiB page size.
 *
 * WARNING:  The internal flash of the GD32VW55x holds the running firmware
 * itself (NuttX is linked at 0x08000000) as well as vendor data used by the
 * Wi-Fi/BLE stacks (the NVDS area and the RF/Wi-Fi trim values).  The board
 * must therefore carve out a region that is known to be unused.  There is
 * no default; erasing or programming the wrong region will brick the image
 * or destroy the calibration data.
 */

#ifndef CONFIG_GD32VW55X_PROGMEM_START_ADDR
#  error "CONFIG_GD32VW55X_PROGMEM_START_ADDR is not defined"
#endif

#ifndef CONFIG_GD32VW55X_PROGMEM_SIZE
#  error "CONFIG_GD32VW55X_PROGMEM_SIZE is not defined"
#endif

#define GD32VW55X_PROGMEM_PAGE_SIZE  GD32VW55X_FLASH_PAGE_SIZE
#define GD32VW55X_PROGMEM_START      CONFIG_GD32VW55X_PROGMEM_START_ADDR
#define GD32VW55X_PROGMEM_SIZE       CONFIG_GD32VW55X_PROGMEM_SIZE
#define GD32VW55X_PROGMEM_END        (GD32VW55X_PROGMEM_START + \
                                      GD32VW55X_PROGMEM_SIZE)
#define GD32VW55X_PROGMEM_NPAGES     (GD32VW55X_PROGMEM_SIZE / \
                                      GD32VW55X_PROGMEM_PAGE_SIZE)

#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PROGMEM_H */
