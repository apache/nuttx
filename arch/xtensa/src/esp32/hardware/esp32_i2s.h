/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_i2s.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_I2S_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_I2S_H

#include "esp32_soc.h"

#define REG_I2S_BASE( i )       (DR_REG_I2S_BASE + ((i)*0x1e000))
#define I2S_PD_CONF_REG(i)      (REG_I2S_BASE(i) + 0x00a4)

/* I2S_PLC_MEM_FORCE_PU : R/W ;bitpos:[3] ;default: 1'h1 ; */

#define I2S_PLC_MEM_FORCE_PU    (BIT(3))
#define I2S_PLC_MEM_FORCE_PU_M  (BIT(3))
#define I2S_PLC_MEM_FORCE_PU_V  0x1
#define I2S_PLC_MEM_FORCE_PU_S  3

/* I2S_FIFO_FORCE_PU : R/W ;bitpos:[1] ;default: 1'h1 ; */

#define I2S_FIFO_FORCE_PU       (BIT(1))
#define I2S_FIFO_FORCE_PU_M     (BIT(1))
#define I2S_FIFO_FORCE_PU_V     0x1
#define I2S_FIFO_FORCE_PU_S     1

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_I2S_H */
