/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_i2c_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721DX_AMEBA_I2C_CHIP_H
#define __ARCH_ARM_SRC_RTL8721DX_AMEBA_I2C_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip I2C wiring for RTL8721DX (amebadplus).  The shared driver
 * (arch/arm/src/common/ameba/ameba_i2c.c) includes this header to learn how
 * many I2C controllers the chip exposes and, for each, its register base,
 * peripheral-clock masks and crossbar pad-mux codes.  It also learns the
 * chip's I2C_InitTypeDef layout through AMEBA_I2C_HAS_DMA_FIELDS.
 *
 * Contract for the other Ameba chips (amebalite / amebasmart / amebagreen2 /
 * RTL8720F): supply a same-named header on the chip include path with the
 * macros below.  What differs per chip, from the fwlib audit:
 *
 *   1. Controller count and bases: 2 controllers on amebadplus / amebalite /
 *      amebagreen2 / RTL8720F, 3 on amebasmart (its I2C0 lives in the LP
 *      domain).  The bases below are the NON-secure peripheral aliases; see
 *      the note in ameba_i2c.c on why the secure alias must not be used.
 *
 *   2. APBPeriph "function" and "clock" masks are two separate lists because
 *      RCC_PeriphClockCmd() takes them as distinct arguments.  They are
 *      equal on every current chip, but amebasmart encodes the bits
 *      differently (bit25/26/27, group bit30=0) versus (bit30|bit10/11)
 *      here, so each chip must supply its own values.
 *
 *   3. Pad mux: amebadplus / amebalite / amebagreen2 / RTL8720F use a
 *      crossbar with a distinct function code per SCL/SDA signal (supplied
 *      as two lists).  amebasmart instead has a single generic
 *      PINMUX_FUNCTION_I2C code (7) shared by every I2C pad; that chip fills
 *      both the SCL and SDA lists with 7 and the driver needs no change.
 *
 *   4. I2C_InitTypeDef layout: amebadplus / amebagreen2 / RTL8720F carry
 *      three DMA request-level fields (I2CTxDMARqLv / I2CRxDMARqLv /
 *      I2CDMAMod) between I2CFilter and I2CAckAddr1 (21 u32 fields);
 *      amebalite and amebasmart omit them (18 fields).  Define
 *      AMEBA_I2C_HAS_DMA_FIELDS to 1 when present so the driver's mirror
 *      struct stays byte-for-byte identical to the fwlib struct that
 *      I2C_StructInit()/I2C_Init() use.
 */

#define AMEBA_NI2C                2

/* NON-secure I2C register bases (I2C0_REG_BASE / I2C1_REG_BASE). */

#define AMEBA_I2C_BASES           { 0x41108000ul, 0x4110a000ul }

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
 * controller.  On a chip with one generic I2C code, set both to that code.
 */

#define AMEBA_I2C_SCLFID          { 48, 50 }  /* I2C0_SCL, I2C1_SCL */
#define AMEBA_I2C_SDAFID          { 49, 51 }  /* I2C0_SDA, I2C1_SDA */

/* The amebadplus I2C_InitTypeDef carries the DMA request-level fields. */

#define AMEBA_I2C_HAS_DMA_FIELDS  1

#endif /* __ARCH_ARM_SRC_RTL8721DX_AMEBA_I2C_CHIP_H */
