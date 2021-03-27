/****************************************************************************
 * arch/arm/src/max326xx/hardware/max326_fcr.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_FCR_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_FCR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_FCR_REG0_OFFSET      0x0000    /* Function Control Register 0 */

/* Register Addresses *******************************************************/

#define MAX326_FCR_REG0             (MAX326_FCR_BASE + MAX326_FCR_REG0_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* Function Control Register 0 */

#define FCR_REG0_I2C0_SDA_FILTER    (1 << 20) /* Bit 20: I2C0 SDA Filter Enable */
#define FCR_REG0_I2C0_SCL_FILTER    (1 << 21) /* Bit 21: I2C0 SCL Filter Enable */
#define FCR_REG0_I2C1_SDA_FILTER    (1 << 22) /* Bit 22: I2C1 SDA Filter Enable */
#define FCR_REG0_I2C1_SCL_FILTER    (1 << 23) /* Bit 23: I2C1 SCL Filter Enable */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_FCR_H */
