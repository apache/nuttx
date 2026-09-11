/****************************************************************************
 * arch/arm/src/stm32h5/stm32_efuse.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_EFUSE_H
#define __ARCH_ARM_SRC_STM32H5_STM32_EFUSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32H5_EFUSE

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

/****************************************************************************
 * Name: stm32_efuse_initialize
 *
 * Description:
 *   Register the OTP area as an efuse character device, built on
 *   stm32_otp_word_read16()/write16() (stm32_flash.h).  Those two are a
 *   bare register access and an nxmutex-protected program sequence
 *   respectively, so unlike this function they have no dependency on
 *   driver init order; this one allocates upper-half driver state and
 *   creates an inode, so call it once from board bring-up, after the
 *   usual driver/GPIO initialization has run.
 *
 * Input Parameters:
 *   devpath - The path to the device, e.g. "/dev/efuse"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_efuse_initialize(FAR const char *devpath);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32H5_EFUSE */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_EFUSE_H */
