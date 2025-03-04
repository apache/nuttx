/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_memorymap.h
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

/* Reference:
 *   "i.MX 8M Plus Applications Processor Reference Manual",
 *   Document Number: IMX8MPRM Rev. 1, 06/2021. NXP
 */

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_MEMORYMAP_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral Memory Map ****************************************************/

#define MX8M_IOMUXC                  0x30330000u
#define MX8M_CCM_ANALOG              0x30360000u
#define MX8M_CCM                     0x30380000u
#define MX8M_SRC                     0x30390000u
#define MX8M_GPC                     0x303A0000u
#define MX8M_RDC                     0x303D0000u
#define MX8M_CSU                     0x303E0000u

#define MX8M_AUDIOMIX                0x30E20000u

#define MX8M_GPT1                    0x302D0000u
#define MX8M_GPT2                    0x302E0000u
#define MX8M_GPT3                    0x302F0000u
#define MX8M_GPT4                    0x30700000u
#define MX8M_GPT5                    0x306F0000u
#define MX8M_GPT6                    0x306E0000u

#define MX8M_PWM1                    0x30660000u
#define MX8M_PWM2                    0x30670000u
#define MX8M_PWM3                    0x30680000u
#define MX8M_PWM4                    0x30690000u

#define MX8M_I2C1                    0x30A20000u
#define MX8M_I2C2                    0x30A30000u
#define MX8M_I2C3                    0x30A40000u
#define MX8M_I2C4                    0x30A50000u
#define MX8M_I2C5                    0x30AD0000u
#define MX8M_I2C6                    0x30AE0000u

#define MX8M_UART1                   0x30860000u
#define MX8M_UART2                   0x30890000u
#define MX8M_UART3                   0x30880000u
#define MX8M_UART4                   0x30A60000u

#define MX8M_ECSPI1                  0x30820000u
#define MX8M_ECSPI2                  0x30830000u
#define MX8M_ECSPI3                  0x30840000u

#define MX8M_MUB                     0x30AB0000u

#define MX8M_GPIO                    0x30200000u
#define MX8M_GPIO1                   (MX8M_GPIO + 0x10000 * 0)
#define MX8M_GPIO2                   (MX8M_GPIO + 0x10000 * 1)
#define MX8M_GPIO3                   (MX8M_GPIO + 0x10000 * 2)
#define MX8M_GPIO4                   (MX8M_GPIO + 0x10000 * 3)
#define MX8M_GPIO5                   (MX8M_GPIO + 0x10000 * 4)

#define MX8M_DDR                     0x40000000u
#define MX8M_OCRAM                   0x00900000u

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_MEMORYMAP_H */
