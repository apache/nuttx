/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_glb.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL808_GPIO_CFG0_OFFSET              0x0008c4  /* gpio_cfg0 */
#define BL808_GPIO_CFG1_OFFSET              0x0008c8  /* gpio_cfg1 */
#define BL808_GPIO_CFG2_OFFSET              0x0008cc  /* gpio_cfg2 */
#define BL808_GPIO_CFG3_OFFSET              0x0008d0  /* gpio_cfg3 */
#define BL808_GPIO_CFG4_OFFSET              0x0008d4  /* gpio_cfg4 */
#define BL808_GPIO_CFG5_OFFSET              0x0008d8  /* gpio_cfg5 */
#define BL808_GPIO_CFG6_OFFSET              0x0008dc  /* gpio_cfg6 */
#define BL808_GPIO_CFG7_OFFSET              0x0008e0  /* gpio_cfg7 */
#define BL808_GPIO_CFG8_OFFSET              0x0008e4  /* gpio_cfg8 */
#define BL808_GPIO_CFG9_OFFSET              0x0008e8  /* gpio_cfg9 */
#define BL808_GPIO_CFG10_OFFSET             0x0008ec  /* gpio_cfg10 */
#define BL808_GPIO_CFG11_OFFSET             0x0008f0  /* gpio_cfg11 */
#define BL808_GPIO_CFG12_OFFSET             0x0008f4  /* gpio_cfg12 */
#define BL808_GPIO_CFG13_OFFSET             0x0008f8  /* gpio_cfg13 */
#define BL808_GPIO_CFG14_OFFSET             0x0008fc  /* gpio_cfg14 */
#define BL808_GPIO_CFG15_OFFSET             0x000900  /* gpio_cfg15 */
#define BL808_GPIO_CFG16_OFFSET             0x000904  /* gpio_cfg16 */
#define BL808_GPIO_CFG17_OFFSET             0x000908  /* gpio_cfg17 */
#define BL808_GPIO_CFG18_OFFSET             0x00090c  /* gpio_cfg18 */
#define BL808_GPIO_CFG19_OFFSET             0x000910  /* gpio_cfg19 */
#define BL808_GPIO_CFG20_OFFSET             0x000914  /* gpio_cfg20 */
#define BL808_GPIO_CFG21_OFFSET             0x000918  /* gpio_cfg21 */
#define BL808_GPIO_CFG22_OFFSET             0x00091c  /* gpio_cfg22 */
#define BL808_GPIO_CFG23_OFFSET             0x000920  /* gpio_cfg23 */
#define BL808_GPIO_CFG24_OFFSET             0x000924  /* gpio_cfg24 */
#define BL808_GPIO_CFG25_OFFSET             0x000928  /* gpio_cfg25 */
#define BL808_GPIO_CFG26_OFFSET             0x00092c  /* gpio_cfg26 */
#define BL808_GPIO_CFG27_OFFSET             0x000930  /* gpio_cfg27 */
#define BL808_GPIO_CFG28_OFFSET             0x000934  /* gpio_cfg28 */
#define BL808_GPIO_CFG29_OFFSET             0x000938  /* gpio_cfg29 */
#define BL808_GPIO_CFG30_OFFSET             0x00093c  /* gpio_cfg30 */
#define BL808_GPIO_CFG31_OFFSET             0x000940  /* gpio_cfg31 */
#define BL808_GPIO_CFG32_OFFSET             0x000944  /* gpio_cfg32 */
#define BL808_GPIO_CFG33_OFFSET             0x000948  /* gpio_cfg33 */
#define BL808_GPIO_CFG34_OFFSET             0x00094c  /* gpio_cfg34 */
#define BL808_GPIO_CFG35_OFFSET             0x000950  /* gpio_cfg35 */
#define BL808_GPIO_CFG36_OFFSET             0x000954  /* gpio_cfg36 */
#define BL808_GPIO_CFG37_OFFSET             0x000958  /* gpio_cfg37 */
#define BL808_GPIO_CFG38_OFFSET             0x00095c  /* gpio_cfg38 */
#define BL808_GPIO_CFG39_OFFSET             0x000960  /* gpio_cfg39 */
#define BL808_GPIO_CFG40_OFFSET             0x000964  /* gpio_cfg40 */
#define BL808_GPIO_CFG41_OFFSET             0x000968  /* gpio_cfg41 */
#define BL808_GPIO_CFG42_OFFSET             0x00096c  /* gpio_cfg42 */
#define BL808_GPIO_CFG43_OFFSET             0x000970  /* gpio_cfg43 */
#define BL808_GPIO_CFG44_OFFSET             0x000974  /* gpio_cfg44 */
#define BL808_GPIO_CFG45_OFFSET             0x000978  /* gpio_cfg45 */

/* Register definitions *****************************************************/

#define BL808_GPIO_CFG0           (BL808_GLB_BASE + BL808_GPIO_CFG0_OFFSET)
#define BL808_GPIO_CFG1           (BL808_GLB_BASE + BL808_GPIO_CFG1_OFFSET)
#define BL808_GPIO_CFG2           (BL808_GLB_BASE + BL808_GPIO_CFG2_OFFSET)
#define BL808_GPIO_CFG3           (BL808_GLB_BASE + BL808_GPIO_CFG3_OFFSET)
#define BL808_GPIO_CFG4           (BL808_GLB_BASE + BL808_GPIO_CFG4_OFFSET)
#define BL808_GPIO_CFG5           (BL808_GLB_BASE + BL808_GPIO_CFG5_OFFSET)
#define BL808_GPIO_CFG6           (BL808_GLB_BASE + BL808_GPIO_CFG6_OFFSET)
#define BL808_GPIO_CFG7           (BL808_GLB_BASE + BL808_GPIO_CFG7_OFFSET)
#define BL808_GPIO_CFG8           (BL808_GLB_BASE + BL808_GPIO_CFG8_OFFSET)
#define BL808_GPIO_CFG9           (BL808_GLB_BASE + BL808_GPIO_CFG9_OFFSET)
#define BL808_GPIO_CFG10          (BL808_GLB_BASE + BL808_GPIO_CFG10_OFFSET)
#define BL808_GPIO_CFG11          (BL808_GLB_BASE + BL808_GPIO_CFG11_OFFSET)
#define BL808_GPIO_CFG12          (BL808_GLB_BASE + BL808_GPIO_CFG12_OFFSET)
#define BL808_GPIO_CFG13          (BL808_GLB_BASE + BL808_GPIO_CFG13_OFFSET)
#define BL808_GPIO_CFG14          (BL808_GLB_BASE + BL808_GPIO_CFG14_OFFSET)
#define BL808_GPIO_CFG15          (BL808_GLB_BASE + BL808_GPIO_CFG15_OFFSET)
#define BL808_GPIO_CFG16          (BL808_GLB_BASE + BL808_GPIO_CFG16_OFFSET)
#define BL808_GPIO_CFG17          (BL808_GLB_BASE + BL808_GPIO_CFG17_OFFSET)
#define BL808_GPIO_CFG18          (BL808_GLB_BASE + BL808_GPIO_CFG18_OFFSET)
#define BL808_GPIO_CFG19          (BL808_GLB_BASE + BL808_GPIO_CFG19_OFFSET)
#define BL808_GPIO_CFG20          (BL808_GLB_BASE + BL808_GPIO_CFG20_OFFSET)
#define BL808_GPIO_CFG21          (BL808_GLB_BASE + BL808_GPIO_CFG21_OFFSET)
#define BL808_GPIO_CFG22          (BL808_GLB_BASE + BL808_GPIO_CFG22_OFFSET)
#define BL808_GPIO_CFG23          (BL808_GLB_BASE + BL808_GPIO_CFG23_OFFSET)
#define BL808_GPIO_CFG24          (BL808_GLB_BASE + BL808_GPIO_CFG24_OFFSET)
#define BL808_GPIO_CFG25          (BL808_GLB_BASE + BL808_GPIO_CFG25_OFFSET)
#define BL808_GPIO_CFG26          (BL808_GLB_BASE + BL808_GPIO_CFG26_OFFSET)
#define BL808_GPIO_CFG27          (BL808_GLB_BASE + BL808_GPIO_CFG27_OFFSET)
#define BL808_GPIO_CFG28          (BL808_GLB_BASE + BL808_GPIO_CFG28_OFFSET)
#define BL808_GPIO_CFG29          (BL808_GLB_BASE + BL808_GPIO_CFG29_OFFSET)
#define BL808_GPIO_CFG30          (BL808_GLB_BASE + BL808_GPIO_CFG30_OFFSET)
#define BL808_GPIO_CFG31          (BL808_GLB_BASE + BL808_GPIO_CFG31_OFFSET)
#define BL808_GPIO_CFG32          (BL808_GLB_BASE + BL808_GPIO_CFG32_OFFSET)
#define BL808_GPIO_CFG33          (BL808_GLB_BASE + BL808_GPIO_CFG33_OFFSET)
#define BL808_GPIO_CFG34          (BL808_GLB_BASE + BL808_GPIO_CFG34_OFFSET)
#define BL808_GPIO_CFG35          (BL808_GLB_BASE + BL808_GPIO_CFG35_OFFSET)
#define BL808_GPIO_CFG36          (BL808_GLB_BASE + BL808_GPIO_CFG36_OFFSET)
#define BL808_GPIO_CFG37          (BL808_GLB_BASE + BL808_GPIO_CFG37_OFFSET)
#define BL808_GPIO_CFG38          (BL808_GLB_BASE + BL808_GPIO_CFG38_OFFSET)
#define BL808_GPIO_CFG39          (BL808_GLB_BASE + BL808_GPIO_CFG39_OFFSET)
#define BL808_GPIO_CFG40          (BL808_GLB_BASE + BL808_GPIO_CFG40_OFFSET)
#define BL808_GPIO_CFG41          (BL808_GLB_BASE + BL808_GPIO_CFG41_OFFSET)
#define BL808_GPIO_CFG42          (BL808_GLB_BASE + BL808_GPIO_CFG42_OFFSET)
#define BL808_GPIO_CFG43          (BL808_GLB_BASE + BL808_GPIO_CFG43_OFFSET)
#define BL808_GPIO_CFG44          (BL808_GLB_BASE + BL808_GPIO_CFG44_OFFSET)
#define BL808_GPIO_CFG45          (BL808_GLB_BASE + BL808_GPIO_CFG45_OFFSET)

/* Register bit definitions *************************************************/

//// Check every bit
#define GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT           (8)
#define GPIO_CFGCTL0_GPIO_0_FUNC_SEL_MASK            (0x0f << GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT)
#define GPIO_CFGCTL0_GPIO_0_OE                       (1 << 6)
#define GPIO_CFGCTL0_GPIO_0_PD                       (1 << 5)
#define GPIO_CFGCTL0_GPIO_0_PU                       (1 << 4)
#define GPIO_CFGCTL0_GPIO_0_DRV_SHIFT                (2)
#define GPIO_CFGCTL0_GPIO_0_DRV_MASK                 (0x03 << GPIO_CFGCTL0_GPIO_0_DRV_SHIFT)
#define GPIO_CFGCTL0_GPIO_0_SMT                      (1 << 1)
#define GPIO_CFGCTL0_GPIO_0_IE                       (1 << 0)
////

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H */