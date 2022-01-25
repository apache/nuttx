/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/regi2c_bbpll.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_BBPLL_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_BBPLL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definitions for digital PLL (BBPLL). This file lists
 * register fields of BBPLL, located on an internal configuration bus.
 */

#define I2C_BBPLL           0x66
#define I2C_BBPLL_HOSTID    0

#define I2C_BBPLL_MODE_HF        4
#define I2C_BBPLL_MODE_HF_MSB    1
#define I2C_BBPLL_MODE_HF_LSB    1

#define I2C_BBPLL_OC_DCUR        6
#define I2C_BBPLL_OC_DCUR_MSB    2
#define I2C_BBPLL_OC_DCUR_LSB    0

#define I2C_BBPLL_OC_DR1        5
#define I2C_BBPLL_OC_DR1_MSB    2
#define I2C_BBPLL_OC_DR1_LSB    0

#define I2C_BBPLL_OC_DR3        5
#define I2C_BBPLL_OC_DR3_MSB    6
#define I2C_BBPLL_OC_DR3_LSB    4

#define I2C_BBPLL_OC_VCO_DBIAS        9
#define I2C_BBPLL_OC_VCO_DBIAS_MSB    1
#define I2C_BBPLL_OC_VCO_DBIAS_LSB    0

#define I2C_BBPLL_OC_DCHGP        2
#define I2C_BBPLL_OC_DCHGP_MSB    6
#define I2C_BBPLL_OC_DCHGP_LSB    4

#define I2C_BBPLL_OC_DLREF_SEL        6
#define I2C_BBPLL_OC_DLREF_SEL_MSB    7
#define I2C_BBPLL_OC_DLREF_SEL_LSB    6

#define I2C_BBPLL_OC_DHREF_SEL        6
#define I2C_BBPLL_OC_DHREF_SEL_MSB    5
#define I2C_BBPLL_OC_DHREF_SEL_LSB    4

#define I2C_BBPLL_OC_REF_DIV        2
#define I2C_BBPLL_OC_REF_DIV_MSB    3
#define I2C_BBPLL_OC_REF_DIV_LSB    0

#define I2C_BBPLL_OC_DIV_7_0        3
#define I2C_BBPLL_OC_DIV_7_0_MSB    7
#define I2C_BBPLL_OC_DIV_7_0_LSB    0

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_BBPLL_H */
