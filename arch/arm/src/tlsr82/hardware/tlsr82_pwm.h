/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_pwm.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_PWM_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PWM_CLKDIV_REG            REG_ADDR8(0x782)

#define PWM_ENABLE0_REG           REG_ADDR8(0x781)
#define PWM_MODE0_REG             REG_ADDR8(0x783)

#define PWM_ENABLE_REG            REG_ADDR8(0x780)
#define PWM_INVERT_REG            REG_ADDR8(0x784)
#define PWM_N_INVERT_REG          REG_ADDR8(0x785)
#define PWM_POL_REG               REG_ADDR8(0x786)

#define PWM_CYCLE_REG(n)          REG_ADDR32(0x794 + ((n) << 2))
#define PWM_CMP_REG(n)            REG_ADDR16(0x794 + ((n) << 2))
#define PWM_MAX_REG(n)            REG_ADDR16(0x796 + ((n) << 2))

#define PWM_PLUSE_NUM_REG         REG_ADDR16(0x7ac)

#define PWM_IRQ_CTRL_REG          REG_ADDR8(0x7b0)
#define PWM_IRQ_STA_REG           REG_ADDR8(0x7b1)

#define PWM_PLUSE_CNT_REG         REG_ADDR8(0x7c0)

/* PWM0 Mode select */

#define PWM_MODE0_NORMAL          0
#define PWM_MODE0_COUNT           1

/* PWM IRQ status */

#define PWM_IRQ_STA_PNUM0         BIT(0)
#define PWM_IRQ_STA_IRFIFO0       BIT(1)
#define PWM_IRQ_STA_CYCLE0        BIT(2)
#define PWM_IRQ_STA_CYCLE1        BIT(3)
#define PWM_IRQ_STA_CYCLE2        BIT(4)
#define PWM_IRQ_STA_CYCLE3        BIT(5)
#define PWM_IRQ_STA_CYCLE4        BIT(6)
#define PWM_IRQ_STA_CYCLE5        BIT(7)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_PWM_H */
