/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_i2c.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* There are 2 main types of I2C IP cores among STM32 chips:
 *   1. STM32 I2C IPv1 - F1, F2, F4 and L1
 *   2. STM32 I2C IPv2 - F0, F3, F7, G0, G4, H7, L0 and L4
 */

#if defined(CONFIG_STM32_HAVE_IP_I2C_V1)
#  include "stm32_i2c_v1.h"
#elif defined(CONFIG_STM32_HAVE_IP_I2C_V2)
#  include "stm32_i2c_v2.h"
#else
#  error STM32 I2C IP version not specified
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_H */
