/****************************************************************************
 * arch/arm64/src/a64/hardware/a64_memorymap.h
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

#ifndef __ARCH_ARM64_SRC_A64_HARDWARE_A64_MEMORYMAP_H
#define __ARCH_ARM64_SRC_A64_HARDWARE_A64_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral Base Addresses */

#define A64_DE_ADDR         0x01000000 /* DE              0x0100:0000-0x012f:ffff 3M */
#define A64_SYSCTL_ADDR     0x01c00000 /* System Control  0x01c0:0000-0x01c0:0fff 4K */
#define A64_TCON0_ADDR      0x01c0c000 /* TCON 0          0x01c0:c000-0x01c0:cfff 4K */
#define A64_CCU_ADDR        0x01c20000 /* CCU             0x01c2:0000-0x01c2:03ff 1K */
#define A64_PIO_ADDR        0x01c20800 /* PIO             0x01c2:0800-0x01c2:0bff 1K */
#define A64_DSI_ADDR        0x01ca0000 /* MIPI DSI        0x01ca:0000-0x01ca:0fff 4K */
#define A64_DPHY_ADDR       0x01ca1000 /* MIPI DSI-PHY    0x01ca:1000-0x01ca:1fff 4K */
#define A64_RPIO_ADDR       0x01f02c00 /* R_PIO           0x01f0:2c00-0x01f0:2fff 1K */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM64_SRC_A64_HARDWARE_A64_MEMORYMAP_H */
