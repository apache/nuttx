/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_crc.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_CRC_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define GD32VW55X_CRC_DATA_OFFSET    0x0000  /* Data register */
#define GD32VW55X_CRC_FDATA_OFFSET   0x0004  /* Free data register */
#define GD32VW55X_CRC_CTL_OFFSET     0x0008  /* Control register */

/* Register addresses *******************************************************/

#define GD32VW55X_CRC_DATA  (GD32VW55X_CRC_BASE + GD32VW55X_CRC_DATA_OFFSET)
#define GD32VW55X_CRC_FDATA (GD32VW55X_CRC_BASE + GD32VW55X_CRC_FDATA_OFFSET)
#define GD32VW55X_CRC_CTL   (GD32VW55X_CRC_BASE + GD32VW55X_CRC_CTL_OFFSET)

/* Register bit definitions *************************************************/

/* Data register: 32-bit calculation result.  Reset value is 0xffffffff */

#define CRC_DATA_RESET               0xffffffff

/* Free data register */

#define CRC_FDATA_SHIFT              (0)       /* Bits 0-7: Free data */
#define CRC_FDATA_MASK               (0xff << CRC_FDATA_SHIFT)

/* Control register */

#define CRC_CTL_RST                  (1 << 0)  /* Reset the data register */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_CRC_H */
