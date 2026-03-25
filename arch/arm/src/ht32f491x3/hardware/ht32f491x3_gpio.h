/****************************************************************************
 * arch/arm/src/ht32f491x3/hardware/ht32f491x3_gpio.h
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_GPIO_H
#define __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO register offsets ****************************************************/

#define HT32_GPIO_CFGR_OFFSET          0x0000
#define HT32_GPIO_OMODE_OFFSET         0x0004
#define HT32_GPIO_ODRVR_OFFSET         0x0008
#define HT32_GPIO_PULL_OFFSET          0x000c
#define HT32_GPIO_IDT_OFFSET           0x0010
#define HT32_GPIO_ODT_OFFSET           0x0014
#define HT32_GPIO_SCR_OFFSET           0x0018
#define HT32_GPIO_WPR_OFFSET           0x001c
#define HT32_GPIO_MUXL_OFFSET          0x0020
#define HT32_GPIO_MUXH_OFFSET          0x0024
#define HT32_GPIO_CLR_OFFSET           0x0028

/* GPIO helpers *************************************************************/

#define HT32_GPIO_PIN(n)               (1u << (n))

#define HT32_GPIO_MODE_SHIFT(n)        ((n) << 1)
#define HT32_GPIO_MODE_MASK(n)         (3u << HT32_GPIO_MODE_SHIFT(n))
#define HT32_GPIO_MODE_VALUE(n, v)     ((uint32_t)(v) << HT32_GPIO_MODE_SHIFT(n))

#define HT32_GPIO_ODRVR_SHIFT(n)       ((n) << 1)
#define HT32_GPIO_ODRVR_MASK(n)        (3u << HT32_GPIO_ODRVR_SHIFT(n))
#define HT32_GPIO_ODRVR_VALUE(n, v)    ((uint32_t)(v) << HT32_GPIO_ODRVR_SHIFT(n))

#define HT32_GPIO_PULL_SHIFT(n)        ((n) << 1)
#define HT32_GPIO_PULL_MASK(n)         (3u << HT32_GPIO_PULL_SHIFT(n))
#define HT32_GPIO_PULL_VALUE(n, v)     ((uint32_t)(v) << HT32_GPIO_PULL_SHIFT(n))

#define HT32_GPIO_MUX_SHIFT(n)         (((n) & 7u) << 2)
#define HT32_GPIO_MUX_MASK(n)          (0x0fu << HT32_GPIO_MUX_SHIFT(n))
#define HT32_GPIO_MUX_VALUE(n, af)     ((uint32_t)(af) << HT32_GPIO_MUX_SHIFT(n))
#define HT32_GPIO_MUX_OFFSET(n)        (((n) < 8u) ? HT32_GPIO_MUXL_OFFSET : \
                                        HT32_GPIO_MUXH_OFFSET)

/* Compatibility aliases for board pinmux code that still uses the older
 * CFGLR/CFGHR naming for alternate-function selection.
 */

#define HT32_GPIO_CFGLR_OFFSET         HT32_GPIO_MUXL_OFFSET
#define HT32_GPIO_CFGHR_OFFSET         HT32_GPIO_MUXH_OFFSET
#define HT32_GPIO_CFG_SHIFT(n)         HT32_GPIO_MUX_SHIFT(n)
#define HT32_GPIO_CFG_MASK(n)          HT32_GPIO_MUX_MASK(n)
#define HT32_GPIO_CFG_VALUE(n, af)     HT32_GPIO_MUX_VALUE(n, af)
#define HT32_GPIO_CFG_OFFSET(n)        HT32_GPIO_MUX_OFFSET(n)

#endif /* __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_GPIO_H */
