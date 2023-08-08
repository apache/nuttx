/****************************************************************************
 * arch/xtensa/src/esp32s3/hardware/regi2c_ctrl.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_REGI2C_CTRL_H
#define __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_REGI2C_CTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "esp32s3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Analog function control register */

#define I2C_MST_ANA_CONF0_REG          0x6000E040
#define I2C_MST_BBPLL_STOP_FORCE_HIGH  (BIT(2))
#define I2C_MST_BBPLL_STOP_FORCE_LOW   (BIT(3))
#define I2C_MST_BBPLL_CAL_DONE         (BIT(24))

/* ROM functions which read/write internal control bus */

extern uint8_t esp_rom_regi2c_read(uint8_t block, uint8_t host_id,
                                   uint8_t reg_add);
extern uint8_t esp_rom_regi2c_read_mask(uint8_t block, uint8_t host_id,
                                        uint8_t reg_add, uint8_t msb,
                                        uint8_t lsb);
extern void esp_rom_regi2c_write(uint8_t block, uint8_t host_id,
                                 uint8_t reg_add, uint8_t data);
extern void esp_rom_regi2c_write_mask(uint8_t block, uint8_t host_id,
                                      uint8_t reg_add, uint8_t msb,
                                      uint8_t lsb, uint8_t data);

/* Convenience macros for the above functions, these use register
 * definitions from regi2c_bbpll.h/regi2c_dig_reg.h/regi2c_lp_bias.h/
 * regi2c_bias.h header files.
 */

#define REGI2C_WRITE_MASK(block, reg_add, indata) \
      esp_rom_regi2c_write_mask(block, block##_HOSTID,  reg_add,  reg_add##_MSB,  reg_add##_LSB,  indata)

#define REGI2C_READ_MASK(block, reg_add) \
      esp_rom_regi2c_read_mask(block, block##_HOSTID,  reg_add,  reg_add##_MSB,  reg_add##_LSB)

#define REGI2C_WRITE(block, reg_add, indata) \
      esp_rom_regi2c_write(block, block##_HOSTID,  reg_add, indata)

#define REGI2C_READ(block, reg_add) \
      esp_rom_regi2c_read(block, block##_HOSTID,  reg_add)

#endif /* __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_REGI2C_CTRL_H */
