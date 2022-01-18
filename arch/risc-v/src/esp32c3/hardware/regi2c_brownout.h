/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/regi2c_brownout.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_BROWNOUT_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_BROWNOUT_H

#define I2C_BOD                     0x61
#define I2C_BOD_HOSTID              0

#define I2C_BOD_THRESHOLD           0x5
#define I2C_BOD_THRESHOLD_MSB       2
#define I2C_BOD_THRESHOLD_LSB       0

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_REGI2C_BROWNOUT_H */
