/****************************************************************************
 * arch/arm/src/ht32f491x3/hardware/ht32f491x3_pwc.h
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_PWC_H
#define __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_PWC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ht32f491x3_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HT32_PWC_LDOOV_OFFSET           0x010

#define HT32_PWC_LDOOV                  (HT32_PWC_BASE + HT32_PWC_LDOOV_OFFSET)

#define HT32_PWC_LDOOVSEL_SHIFT         0
#define HT32_PWC_LDOOVSEL_MASK          (3 << HT32_PWC_LDOOVSEL_SHIFT)
#define HT32_PWC_LDO_OUTPUT_1V1         (1 << HT32_PWC_LDOOVSEL_SHIFT)
#define HT32_PWC_LDO_OUTPUT_1V2         (2 << HT32_PWC_LDOOVSEL_SHIFT)
#define HT32_PWC_LDO_OUTPUT_1V3         (3 << HT32_PWC_LDOOVSEL_SHIFT)

#endif /* __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_PWC_H */
