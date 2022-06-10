/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_gpio.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_GPIO_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO register definitions */

#define GPIO_SET_PA_ACT_REG            REG_ADDR8(0x586)
#define GPIO_SET_PA_IN_REG             REG_ADDR8(0x580)
#define GPIO_SET_PA_IE_REG             REG_ADDR8(0x581)
#define GPIO_SET_PA_OEN_REG            REG_ADDR8(0x582)
#define GPIO_SET_PA_OUT_REG            REG_ADDR8(0x583)
#define GPIO_SET_PA_POL_REG            REG_ADDR8(0x584)
#define GPIO_SET_PA_DS_REG             REG_ADDR8(0x585)

#define GPIO_SET_PB_ACT_REG            REG_ADDR8(0x586 + 8)
#define GPIO_SET_PB_IN_REG             REG_ADDR8(0x580 + 8)
#define GPIO_SET_PB_IE_REG             REG_ADDR8(0x581 + 8)
#define GPIO_SET_PB_OEN_REG            REG_ADDR8(0x582 + 8)
#define GPIO_SET_PB_OUT_REG            REG_ADDR8(0x583 + 8)
#define GPIO_SET_PB_POL_REG            REG_ADDR8(0x584 + 8)
#define GPIO_SET_PB_DS_REG             REG_ADDR8(0x585 + 8)

#define GPIO_SET_PC_ACT_REG            REG_ADDR8(0x586 + 16)
#define GPIO_SET_PC_IN_REG             REG_ADDR8(0x580 + 16)
#define ANALOG_PC_IE_ADDR              0xc0
#define GPIO_SET_PC_OEN_REG            REG_ADDR8(0x582 + 16)
#define GPIO_SET_PC_OUT_REG            REG_ADDR8(0x583 + 16)
#define GPIO_SET_PC_POL_REG            REG_ADDR8(0x584 + 16)
#define ANALOG_PC_DS_ADDR              0xc2

#define GPIO_SET_PD_ACT_REG            REG_ADDR8(0x586 + 24)
#define GPIO_SET_PD_IN_REG             REG_ADDR8(0x580 + 24)
#define GPIO_SET_PD_IE_REG             REG_ADDR8(0x581 + 24)
#define GPIO_SET_PD_OEN_REG            REG_ADDR8(0x582 + 24)
#define GPIO_SET_PD_OUT_REG            REG_ADDR8(0x583 + 24)
#define GPIO_SET_PD_POL_REG            REG_ADDR8(0x584 + 24)
#define GPIO_SET_PD_DS_REG             REG_ADDR8(0x585 + 24)

#define GPIO_SET_PE_ACT_REG            REG_ADDR8(0x5A6)
#define GPIO_SET_PE_IN_REG             REG_ADDR8(0x5A0)
#define GPIO_SET_PE_IE_REG             REG_ADDR8(0x5A1)
#define GPIO_SET_PE_OEN_REG            REG_ADDR8(0x5A2)
#define GPIO_SET_PE_OUT_REG            REG_ADDR8(0x5A3)
#define GPIO_SET_PE_POL_REG            REG_ADDR8(0x5A4)
#define GPIO_SET_PE_DS_REG             REG_ADDR8(0x5A5)

#define GPIO_SETTING_ACT_REG(group)    REG_ADDR8(0x586 + ((group) << 3))
#define GPIO_SETTING_IN_REG(group)     REG_ADDR8(0x580 + ((group) << 3))
#define GPIO_SETTING_IE_REG(group)     REG_ADDR8(0x581 + ((group) << 3))
#define GPIO_SETTING_OEN_REG(group)    REG_ADDR8(0x582 + ((group) << 3))
#define GPIO_SETTING_OUT_REG(group)    REG_ADDR8(0x583 + ((group) << 3))
#define GPIO_SETTING_POL_REG(group)    REG_ADDR8(0x584 + ((group) << 3))
#define GPIO_SETTING_DS_REG(group)     REG_ADDR8(0x585 + ((group) << 3))

#define GPIO_MUX_REG(group, pin)       REG_ADDR8(0x5a8 + ((group) << 1) \
                                       + (((pin) >= 4) ? 1 : 0))

#define GPIO_IRQ_NORMAL_ALL_REG        REG_ADDR8(0x5b5)
#define GPIO_IRQ_RISC_EN_REG           REG_ADDR8(0x642)

#define GPIO_IRQ_NORMAL_REG(group)     REG_ADDR8(0x587 + ((group) << 3))
#define GPIO_IRQ_M0_REG(group)         REG_ADDR8(0x5b8 + (group))
#define GPIO_IRQ_M1_REG(group)         REG_ADDR8(0x5c0 + (group))
#define GPIO_IRQ_M2_REG(group)         REG_ADDR8(0x5c8 + (group))

#define GPIO_IRQ_NORMAL_ALL_WAKEUP     (1 << 2)
#define GPIO_IRQ_NORMAL_ALL_EN         (1 << 3)
#define GPIO_IRQ_RISC0_EN              (1 << 5)
#define GPIO_IRQ_RISC1_EN              (1 << 6)

#define GPIO_SET_AS_GPIO(group, pin)   BM_SET(GPIO_SETTING_ACT_REG(group), BIT(pin))
#define GPIO_SET_AS_MUX(group, pin)    BM_CLR(GPIO_SETTING_ACT_REG(group), BIT(pin))

#define GPIO_SET_OUT_HIGH(group, pin)
#define GPIO_SET_OUT_LOW(group, pin)

#define GPIO_SET_IN_VAL(group, pin)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_GPIO_H */
