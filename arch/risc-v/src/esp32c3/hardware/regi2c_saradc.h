/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/regi2c_saradc.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_SARADC_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_SARADC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * Register definitions for analog to calibrate initial code for getting a
 * more precise voltage of SAR ADC.
 */

#define I2C_ADC                 (0x69)
#define I2C_ADC_HOSTID          (0)

#define I2C_ADC1_ENCAL_GND      (0x7)
#define I2C_ADC1_ENCAL_GND_MSB  (0x5)
#define I2C_ADC1_ENCAL_GND_LSB  (0x5)

#define I2C_ADC1_INITVAL_L      (0x0)
#define I2C_ADC1_INITVAL_L_MSB  (0x7)
#define I2C_ADC1_INITVAL_L_LSB  (0x0)

#define I2C_ADC1_INITVAL_H      (0x1)
#define I2C_ADC1_INITVAL_H_MSB  (0x3)
#define I2C_ADC1_INITVAL_H_LSB  (0x0)

#define I2C_ADC1_DEF            (0x2)
#define I2C_ADC1_DEF_MSB        (0x6)
#define I2C_ADC1_DEF_LSB        (0x4)

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_SARADC_H */
