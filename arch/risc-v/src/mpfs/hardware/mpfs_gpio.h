/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_gpio.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_GPIO_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mpfs_memorymap.h"

#if defined(CONFIG_ARCH_CHIP_MPFS250T_FCVG484) || defined(CONFIG_ARCH_CHIP_MPFS250T_FCG484)
#include "hardware/mpfs250t_484_pinmap.h"
#else
#error The selected MPFS variant is not impelemented
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define MPFS_GPIO_CONFIG_0_OFFSET     0x0000 /* GPIO Config 0 */
#define MPFS_GPIO_CONFIG_1_OFFSET     0x0004 /* GPIO Config 1 */
#define MPFS_GPIO_CONFIG_2_OFFSET     0x0008 /* GPIO Config 2 */
#define MPFS_GPIO_CONFIG_3_OFFSET     0x000C /* GPIO Config 3 */
#define MPFS_GPIO_CONFIG_4_OFFSET     0x0010 /* GPIO Config 4 */
#define MPFS_GPIO_CONFIG_5_OFFSET     0x0014 /* GPIO Config 5 */
#define MPFS_GPIO_CONFIG_6_OFFSET     0x0018 /* GPIO Config 6 */
#define MPFS_GPIO_CONFIG_7_OFFSET     0x001C /* GPIO Config 7 */
#define MPFS_GPIO_CONFIG_8_OFFSET     0x0020 /* GPIO Config 8 */
#define MPFS_GPIO_CONFIG_9_OFFSET     0x0024 /* GPIO Config 9 */
#define MPFS_GPIO_CONFIG_10_OFFSET    0x0028 /* GPIO Config 10 */
#define MPFS_GPIO_CONFIG_11_OFFSET    0x002C /* GPIO Config 11 */
#define MPFS_GPIO_CONFIG_12_OFFSET    0x0020 /* GPIO Config 12 */
#define MPFS_GPIO_CONFIG_13_OFFSET    0x0024 /* GPIO Config 13 */
#define MPFS_GPIO_CONFIG_14_OFFSET    0x0028 /* GPIO Config 14 */
#define MPFS_GPIO_CONFIG_15_OFFSET    0x002C /* GPIO Config 15 */
#define MPFS_GPIO_CONFIG_16_OFFSET    0x0040 /* GPIO Config 16 */
#define MPFS_GPIO_CONFIG_17_OFFSET    0x0044 /* GPIO Config 17 */
#define MPFS_GPIO_CONFIG_18_OFFSET    0x0048 /* GPIO Config 18 */
#define MPFS_GPIO_CONFIG_19_OFFSET    0x004C /* GPIO Config 19 */
#define MPFS_GPIO_CONFIG_20_OFFSET    0x0050 /* GPIO Config 20 */
#define MPFS_GPIO_CONFIG_21_OFFSET    0x0054 /* GPIO Config 21 */
#define MPFS_GPIO_CONFIG_22_OFFSET    0x0058 /* GPIO Config 22 */
#define MPFS_GPIO_CONFIG_23_OFFSET    0x005C /* GPIO Config 23 */
#define MPFS_GPIO_CONFIG_24_OFFSET    0x0060 /* GPIO Config 24 */
#define MPFS_GPIO_CONFIG_25_OFFSET    0x0064 /* GPIO Config 25 */
#define MPFS_GPIO_CONFIG_26_OFFSET    0x0068 /* GPIO Config 26 */
#define MPFS_GPIO_CONFIG_27_OFFSET    0x006C /* GPIO Config 27 */
#define MPFS_GPIO_CONFIG_28_OFFSET    0x0070 /* GPIO Config 28 */
#define MPFS_GPIO_CONFIG_29_OFFSET    0x0074 /* GPIO Config 29 */
#define MPFS_GPIO_CONFIG_30_OFFSET    0x0078 /* GPIO Config 30 */
#define MPFS_GPIO_CONFIG_31_OFFSET    0x007C /* GPIO Config 31 */
#define MPFS_GPIO_INTR_OFFSET         0x0080 /* GPIO Irq state */
#define MPFS_GPIO_GPIN_OFFSET         0x0084 /* GPIO Input states */
#define MPFS_GPIO_GPOUT_OFFSET        0x0088 /* GPIO Ouput states */
#define MPFS_GPIO_CONFIG_ALL_OFFSET   0x008C /* GPIO set all configs */
#define MPFS_GPIO_CONFIG_BYTE0_OFFSET 0x0090 /* GPIO set all configs in byte-0 */
#define MPFS_GPIO_CONFIG_BYTE1_OFFSET 0x0094 /* GPIO set all configs in byte-1 */
#define MPFS_GPIO_CONFIG_BYTE2_OFFSET 0x0098 /* GPIO set all configs in byte-3 */
#define MPFS_GPIO_CONFIG_BYTE3_OFFSET 0x009C /* GPIO set all configs in byte-4 */
#define MPFS_GPIO_CLEAR_BITS_OFFSET   0x00A0 /* GPIO Clear bits */
#define MPFS_GPIO_SET_BITS_OFFSET     0x00A4 /* GPIO Set bits */

/* Register bit field definitions *******************************************/

/* CONFIG_X */

#define GPIO_CONFIG_EN_OUT          (1 << 0)  /* Output enable */
#define GPIO_CONFIG_EN_IN           (1 << 1)  /* Input enable */
#define GPIO_CONFIG_EN_OE_BUF       (1 << 2)  /* Output buffer enable */
#define GPIO_CONFIG_EN_INT          (1 << 3)  /* Interrupt enable */
#define GPIO_CONFIG_INT_SHIFT       (5)       /* Bits: 5-7: Interrupt Types */
#define GPIO_CONFIG_INT_MASK        (7)
#  define GPIO_CONFIG_INT_HIGH      (0 << GPIO_CONFIG_INT_SHIFT)
#  define GPIO_CONFIG_INT_LOW       (1 << GPIO_CONFIG_INT_SHIFT)
#  define GPIO_CONFIG_INT_EDGE_POS  (2 << GPIO_CONFIG_INT_SHIFT)
#  define GPIO_CONFIG_INT_EDGE_NEG  (3 << GPIO_CONFIG_INT_SHIFT)
#  define GPIO_CONFIG_INT_EDGE_BOTH (4 << GPIO_CONFIG_INT_SHIFT)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_GPIO_H */
