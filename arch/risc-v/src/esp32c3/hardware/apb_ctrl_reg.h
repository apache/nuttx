/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/apb_ctrl_reg.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_APB_CTRL_REG_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_APB_CTRL_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32c3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APB_CTRL_FRONT_END_MEM_PD_REG      (DR_REG_APB_CTRL_BASE + 0x09C)

#define APB_CTRL_MEM_POWER_UP_REG          (DR_REG_APB_CTRL_BASE + 0x0AC)

#define APB_CTRL_RETENTION_CTRL_REG        (DR_REG_APB_CTRL_BASE + 0x0A0)

/* APB_CTRL_DC_MEM_FORCE_PU : R/W ;bitpos:[4] ;default: 1'b1 ; */

#define APB_CTRL_DC_MEM_FORCE_PU  (BIT(4))
#define APB_CTRL_DC_MEM_FORCE_PU_M  (BIT(4))
#define APB_CTRL_DC_MEM_FORCE_PU_V  0x1
#define APB_CTRL_DC_MEM_FORCE_PU_S  4

/* APB_CTRL_PBUS_MEM_FORCE_PU : R/W ;bitpos:[2] ;default: 1'b1 ; */

#define APB_CTRL_PBUS_MEM_FORCE_PU  (BIT(2))
#define APB_CTRL_PBUS_MEM_FORCE_PU_M  (BIT(2))
#define APB_CTRL_PBUS_MEM_FORCE_PU_V  0x1
#define APB_CTRL_PBUS_MEM_FORCE_PU_S  2

/* APB_CTRL_AGC_MEM_FORCE_PU : R/W ;bitpos:[0] ;default: 1'b1 ; */

#define APB_CTRL_AGC_MEM_FORCE_PU  (BIT(0))
#define APB_CTRL_AGC_MEM_FORCE_PU_M  (BIT(0))
#define APB_CTRL_AGC_MEM_FORCE_PU_V  0x1
#define APB_CTRL_AGC_MEM_FORCE_PU_S  0

/* APB_CTRL_SRAM_POWER_UP : R/W ;bitpos:[5:2] ;default: ~4'b0 ; */

#define APB_CTRL_SRAM_POWER_UP  0x0000000F
#define APB_CTRL_SRAM_POWER_UP_M  ((APB_CTRL_SRAM_POWER_UP_V)<<(APB_CTRL_SRAM_POWER_UP_S))
#define APB_CTRL_SRAM_POWER_UP_V  0xF
#define APB_CTRL_SRAM_POWER_UP_S  2

/* APB_CTRL_ROM_POWER_UP : R/W ;bitpos:[1:0] ;default: ~2'b0 ; */

#define APB_CTRL_ROM_POWER_UP  0x00000003
#define APB_CTRL_ROM_POWER_UP_M  ((APB_CTRL_ROM_POWER_UP_V)<<(APB_CTRL_ROM_POWER_UP_S))
#define APB_CTRL_ROM_POWER_UP_V  0x3
#define APB_CTRL_ROM_POWER_UP_S  0

/* APB_CTRL_RETENTION_LINK_ADDR : R/W ;bitpos:[26:0] ;default: 27'd0 ; */

#define APB_CTRL_RETENTION_LINK_ADDR  0x07FFFFFF
#define APB_CTRL_RETENTION_LINK_ADDR_M  ((APB_CTRL_RETENTION_LINK_ADDR_V)<<(APB_CTRL_RETENTION_LINK_ADDR_S))
#define APB_CTRL_RETENTION_LINK_ADDR_V  0x7FFFFFF
#define APB_CTRL_RETENTION_LINK_ADDR_S  0

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_APB_CTRL_REG_H */
