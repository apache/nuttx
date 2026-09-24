/****************************************************************************
 * arch/arm/src/stm32h5/stm32_edata.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_EDATA_H
#define __ARCH_ARM_SRC_STM32H5_STM32_EDATA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct mtd_dev_s;

/****************************************************************************
 * Name: stm32_edata_initialize
 *
 * Description:
 *   Create an MTD device on the flash high-cycle data (EDATA) area of a
 *   physical flash bank.
 *
 *   If the bank does not already have nsectors of EDATA, the option bytes
 *   are reprogrammed and the affected sectors are erased.  The last
 *   nsectors sectors of the bank are no longer usable as user flash, so the
 *   firmware image and anything else using those sectors (such as a second
 *   image bank) must stay clear of them.
 *
 *   The MTD device has 6 KiB erase blocks, 2 byte read/write blocks, and an
 *   erase state of 0xff.  Each 2 byte block can be written only once after
 *   the erase block that holds it is erased.
 *
 * Input Parameters:
 *   bank     - The physical flash bank (1 or 2)
 *   nsectors - The number of EDATA sectors (1..8)
 *
 * Returned Value:
 *   The MTD device, or NULL on failure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *stm32_edata_initialize(int bank,
                                             unsigned int nsectors);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_EDATA_H */
