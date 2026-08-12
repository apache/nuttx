/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_i2c_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8720F_AMEBA_I2C_CHIP_H
#define __ARCH_ARM_SRC_RTL8720F_AMEBA_I2C_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip I2C wiring for RTL8720F.  The shared driver
 * (arch/arm/src/common/ameba/ameba_i2c.c) includes this header to learn how
 * many I2C controllers the chip exposes and, for each, its register base,
 * peripheral-clock masks and crossbar pad-mux codes.  It also learns the
 * chip's I2C_InitTypeDef layout through AMEBA_I2C_HAS_DMA_FIELDS.
 *
 * The values below come from the RTL8720F fwlib headers:
 *
 *   1. Two controllers (I2C0/I2C1).  The bases are the NON-secure peripheral
 *      aliases (I2C0_REG_BASE / I2C1_REG_BASE in hal_platform.h, 0x401C_8000
 *      / 0x401C_9000; the secure aliases I2Cx_REG_BASE_S live at
 *      0x501C_xxxx).  NuttX runs on the KM4TZ core in the SECURE state, but
 *      the I2C block responds on its non-secure alias, so the driver hands
 *      the fwlib these non-secure bases (see the note in ameba_i2c.c).
 *
 *   2. APBPeriph_I2Cx (function) and APBPeriph_I2Cx_CLOCK masks
 *      (sysreg_lsys.h): group bit30 plus bit10 (I2C0) / bit11 (I2C1).  Equal
 *      for the function and clock arguments on this chip.
 *
 *   3. Crossbar pad-mux: a distinct function code per SCL/SDA signal
 *      (PINMUX_FUNCTION_I2Cx_SCL/SDA in ameba_pinmux.h): 59/60 for I2C0 and
 *      61/62 for I2C1.
 *
 *   4. The RTL8720F I2C_InitTypeDef carries the three DMA request-level
 *      fields (I2CTxDMARqLv / I2CRxDMARqLv / I2CDMAMod) between I2CFilter
 *      and I2CAckAddr1, so AMEBA_I2C_HAS_DMA_FIELDS is 1 to keep the
 *      driver's mirror struct byte-for-byte identical to the fwlib one.
 */

#define AMEBA_NI2C                2

/* NON-secure I2C register bases (I2C0_REG_BASE / I2C1_REG_BASE). */

#define AMEBA_I2C_BASES           { 0x401c8000ul, 0x401c9000ul }

/* APBPeriph_I2Cx (function) and APBPeriph_I2Cx_CLOCK masks.  Equal on this
 * chip; kept as two lists so chips where they differ can supply both.
 */

#define AMEBA_I2C_APBPERIPH       \
        { (((uint32_t)1 << 30) | ((uint32_t)1 << 10)), \
          (((uint32_t)1 << 30) | ((uint32_t)1 << 11)) }

#define AMEBA_I2C_APBPERIPH_CLK   \
        { (((uint32_t)1 << 30) | ((uint32_t)1 << 10)), \
          (((uint32_t)1 << 30) | ((uint32_t)1 << 11)) }

/* Crossbar pad-mux function codes (PINMUX_FUNCTION_I2Cx_SCL/SDA), indexed by
 * controller.
 */

#define AMEBA_I2C_SCLFID          { 59, 61 }  /* I2C0_SCL, I2C1_SCL */
#define AMEBA_I2C_SDAFID          { 60, 62 }  /* I2C0_SDA, I2C1_SDA */

/* The RTL8720F I2C_InitTypeDef carries the DMA request-level fields. */

#define AMEBA_I2C_HAS_DMA_FIELDS  1

#endif /* __ARCH_ARM_SRC_RTL8720F_AMEBA_I2C_CHIP_H */
