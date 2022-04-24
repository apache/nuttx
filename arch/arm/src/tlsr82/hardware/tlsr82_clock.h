/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_clock.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_CLOCK_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_CLK                      (CONFIG_TLSR82_CPU_CLK_MHZ * 1000000)

#define CLK_EN1_REG                  REG_ADDR8(0x63)
#define CLK_EN2_REG                  REG_ADDR8(0x64)
#define CLK_EN3_REG                  REG_ADDR8(0x65)
#define CLK_SYS_SEL                  REG_ADDR8(0x66)

#define CLK_EN1_SPI                  BIT(0)
#define CLK_EN1_I2C                  BIT(1)
#define CLK_EN1_UART                 BIT(2)
#define CLK_EN1_USB                  BIT(3)
#define CLK_EN1_PWM                  BIT(4)
#define CLK_EN1_QDEC                 BIT(5)
#define CLK_EN1_IR                   BIT(6)
#define CLK_EN1_SWIRE                BIT(7)

#define CLK_EN2_ZB                   BIT(0)
#define CLK_EN2_SYSTIM               BIT(1)
#define CLK_EN2_DMA                  BIT(2)
#define CLK_EN2_ALGM                 BIT(3)
#define CLK_EN2_AES                  BIT(4)
#define CLK_EN2_RSVD1                BIT(5)
#define CLK_EN2_RSVD2                BIT(6)
#define CLK_EN2_PKE                  BIT(7)

#define CLK_EN3_AIF                  BIT(0)
#define CLK_EN3_AUDIO                BIT(1)
#define CLK_EN3_DFIFO                BIT(2)
#define CLK_EN3_TRNG                 BIT(3)
#define CLK_EN3_MC                   BIT(4)
#define CLK_EN3_MCIC                 BIT(5)
#define CLK_EN3_RSVD1                BIT(6)
#define CLK_EN3_RSVD2                BIT(7)

#define CLK_SYS_12M_CRYSTAL          0x44
#define CLK_SYS_16M_CRYSTAL          0x43
#define CLK_SYS_24M_CRYSTAL          0x42
#define CLK_SYS_32M_CRYSTAL          0x60
#define CLK_SYS_48M_CRYSTAL          0x20
#define CLK_SYS_RC_THRES             0x10

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_CLOCK_H */
