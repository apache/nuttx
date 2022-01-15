/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/regi2c_lp_bias.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_LP_BIAS_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_LP_BIAS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definitions for analog to calibrate o_code for getting a more
 * precise voltage. This file lists register fields of low power dbais,
 * located on an internal configuration bus.
 */

#define I2C_ULP 0x61
#define I2C_ULP_HOSTID 0

#define I2C_ULP_IR_RESETB 0
#define I2C_ULP_IR_RESETB_MSB 0
#define I2C_ULP_IR_RESETB_LSB 0

#define I2C_ULP_O_DONE_FLAG 3
#define I2C_ULP_O_DONE_FLAG_MSB 0
#define I2C_ULP_O_DONE_FLAG_LSB 0

#define I2C_ULP_BG_O_DONE_FLAG 3
#define I2C_ULP_BG_O_DONE_FLAG_MSB 3
#define I2C_ULP_BG_O_DONE_FLAG_LSB 3

#define I2C_ULP_IR_FORCE_XPD_CK 0
#define I2C_ULP_IR_FORCE_XPD_CK_MSB 2
#define I2C_ULP_IR_FORCE_XPD_CK_LSB 2

#define I2C_ULP_IR_FORCE_CODE 5
#define I2C_ULP_IR_FORCE_CODE_MSB 6
#define I2C_ULP_IR_FORCE_CODE_LSB 6

#define I2C_ULP_EXT_CODE 6
#define I2C_ULP_EXT_CODE_MSB 7
#define I2C_ULP_EXT_CODE_LSB 0

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_LP_BIAS_H */
