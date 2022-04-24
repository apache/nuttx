/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_register.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_REGISTER_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_REGISTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Regisger base address */

#define REG_BASE_ADDR           0x00800000

#define REG_ADDR8(a)            getreg8(REG_BASE_ADDR + (a))
#define REG_ADDR16(a)           getreg16(REG_BASE_ADDR + (a))
#define REG_ADDR32(a)           getreg32(REG_BASE_ADDR + (a))

#define write_reg8(addr,v)      putreg8(v, REG_BASE_ADDR + (addr))
#define write_reg16(addr,v)     putreg16(v, REG_BASE_ADDR + (addr))
#define write_reg32(addr,v)     putreg32(v, REG_BASE_ADDR + (addr))

/* Common macros definition */

#define BIT(n)                  (1 << (n))
#define BIT_MASK_LEN(len)       (BIT(len)-1)
#define BIT_RNG(s, e)           (BIT_MASK_LEN((e) - (s) + 1) << (s))
#define BM_SET(x, m)            ((x) |= (m))
#define BM_CLR(x, m)            ((x) &= ~(m))
#define BM_IS_SET(x, m)         ((x) & (m))
#define BM_IS_CLR(x, m)         ((~(x)) & (m))
#define BM_FLIP(x, m)           ((x) ^= (mask))

/* Reset register definition */

#define RESET_RST0_REG          REG_ADDR8(0x60)
#define RESET_RST1_REG          REG_ADDR8(0x61)
#define RESET_RST2_REG          REG_ADDR8(0x62)
#define RESET_PWDNEN_REG        REG_ADDR8(0x6f)

/* Reset reson definition */

#define RESET_RST0_SPI          BIT(0)
#define RESET_RST0_I2C          BIT(1)
#define RESET_RST0_UART         BIT(2)
#define RESET_RST0_USB          BIT(3)
#define RESET_RST0_PWM          BIT(4)
#define RESET_RST0_QDEC         BIT(5)
#define RESET_RST0_IR           BIT(6)
#define RESET_RST0_SWIRE        BIT(7)

#define RESET_RST1_ZB           BIT(0)
#define RESET_RST1_SYSTIM       BIT(1)
#define RESET_RST1_DMA          BIT(2)
#define RESET_RST1_ALGM         BIT(3)
#define RESET_RST1_AES          BIT(4)
#define RESET_RST1_ADC          BIT(5)
#define RESET_RST1_ALG          BIT(6)
#define RESET_RST1_PKE          BIT(7)

#define RESET_RST2_AIF          BIT(0)
#define RESET_RST2_AUDIO        BIT(1)
#define RESET_RST2_DFIFO        BIT(2)
#define RESET_RST2_TRNG         BIT(3)
#define RESET_RST2_RISC         BIT(4)
#define RESET_RST2_MCIC         BIT(5)
#define RESET_RST2_RSIC1R       BIT(6)
#define RESET_RST2_MCIC1R       BIT(7)

#define RESET_PWDNEN_SUSP_EN    BIT(0)
#define RESET_PWDNEN_RESET_ALL  BIT(5)
#define RESET_PWDNEN_SUSP       BIT(7)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_REGISTER_H */
